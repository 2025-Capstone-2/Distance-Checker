package com.example.rssitest

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.net.wifi.ScanResult
import android.net.wifi.WifiManager
import android.os.Bundle
import android.util.Log
import android.widget.ArrayAdapter
import android.widget.ListView
import androidx.appcompat.app.AppCompatActivity
import java.util.Locale
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Wi-Fi RSSI 스캔 → Unscented Kalman Filter(UKF)로 거리 추정 정확도 향상.
 * 기반 논문: Accuracy Improvement of RSSI-based Distance Localization using UKF, iJIM 2020.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var wifiManager: WifiManager
    private lateinit var wifiListView: ListView
    private lateinit var wifiListAdapter: ArrayAdapter<String>

    /** AP(BSSID) → UKF 인스턴스 매핑  */
    private val ukfMap: MutableMap<String, WifiUkf> = mutableMapOf()

    companion object {
        private const val RSSI_AT_1M = -40         // 1 m에서 RSSI (환경에 맞게 교정)
        private const val PATH_LOSS_EXPONENT = 3.0 // 실내 감쇠 계수 n
        private const val WALL_LOSS = 1            // 벽 1장 당 추가 손실 [dB]
        private const val MIN_RSSI = -60           // 표시 최소 RSSI [dBm]
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        wifiListView = findViewById(R.id.wifiListView)
        wifiListAdapter = ArrayAdapter(this,
            android.R.layout.simple_list_item_1,
            mutableListOf<String>())
        wifiListView.adapter = wifiListAdapter

        wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        if (!wifiManager.isWifiEnabled) wifiManager.isWifiEnabled = true

        /** 스캔 결과 수신 브로드캐스트 등록 */
        registerReceiver(object : BroadcastReceiver() {
            override fun onReceive(context: Context?, intent: Intent?) {
                val scanResults = wifiManager.scanResults
                updateWithUkf(scanResults)
            }
        }, IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION))

        wifiManager.startScan()
    }

    /** RSSI → 거리 (m)  */
    private fun rssiToDistance(rssi: Int, walls: Int = 1): Double {
        val totalLoss = (RSSI_AT_1M - rssi) - (walls * WALL_LOSS)
        return 10.0.pow(totalLoss / (10 * PATH_LOSS_EXPONENT))
    }

    /** 스캔 목록을 UKF로 보정 후 UI 업데이트 */
    private fun updateWithUkf(scanResults: List<ScanResult>) {
        val displayList = mutableListOf<String>()

        scanResults
            .filter { it.level >= MIN_RSSI }
            .forEach { result ->
                val bssid = result.BSSID
                val rawDistance = rssiToDistance(result.level)

                // AP마다 UKF 인스턴스를 재사용
                val ukf = ukfMap.getOrPut(bssid) { WifiUkf(initial = rawDistance) }
                val filteredDistance = ukf.update(rawDistance)

                displayList += "${result.SSID.ifEmpty { bssid }} " +
                        "(${result.level} dBm) - " +
                        "거리: ${String.format(Locale.US, "%.2f m", filteredDistance)}"
            }

        if (displayList.isEmpty()) displayList += "표시할 Wi-Fi 네트워크가 없습니다."

        runOnUiThread {
            wifiListAdapter.clear()
            wifiListAdapter.addAll(displayList)
            wifiListAdapter.notifyDataSetChanged()
        }
    }
}

/* -------------------------------------------------------------------------- */
/* ---------------------------  UKF 1-차원 구현  ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * 단일 스칼라 상태(거리)용 Unscented Kalman Filter.
 *  - 상태전이 f(x)=x  (거리 변화를 느리다고 가정)
 *  - 측정 h(x)=x      (RSSI→거리 변환 후 필터 입력)
 */
class WifiUkf(
    initial: Double,
    private var P: Double = 1.0, // 상태 공분산 초기값
    private val Q: Double = 0.1, // 프로세스 노이즈
    private val R: Double = 0.5  // 측정 노이즈
) {
    private var x: Double = initial               // 추정 거리
    private val alpha = 1e-3
    private val kappa = 0.0
    private val beta = 2.0

    /** 측정값 z(k)를 반영하고 최신 추정치를 반환 */
    fun update(z: Double): Double {
        /* ---------- 예측 단계 ---------- */
        val xPred = x                // f(x)=x
        val PPred = P + Q

        /* ---------- Σ-점 생성 ---------- */
        val l = 1                     // 상태 차원
        val lambda = alpha * alpha * (l + kappa) - l
        val c = l + lambda
        val sqrtTerm = sqrt(c * PPred)
        val sigma = doubleArrayOf(xPred, xPred + sqrtTerm, xPred - sqrtTerm)

        /* ---------- 측정 예측 ---------- */
        val wm0 = lambda / c
        val wc0 = wm0 + (1 - alpha * alpha + beta)
        val wi = 1.0 / (2 * c)
        val wm = doubleArrayOf(wm0, wi, wi)
        val wc = doubleArrayOf(wc0, wi, wi)

        var zPred = 0.0
        for (i in sigma.indices) zPred += wm[i] * sigma[i]         // h(x)=x

        var S = R
        var Cxz = 0.0
        for (i in sigma.indices) {
            val dz = sigma[i] - zPred
            val dx = sigma[i] - xPred
            S += wc[i] * dz * dz
            Cxz += wc[i] * dx * dz
        }

        /* ---------- 업데이트 ---------- */
        val K = Cxz / S                      // Kalman Gain
        x = xPred + K * (z - zPred)          // 상태 업데이트
        P = PPred - K * S * K                // 공분산 업데이트

        return x
    }
}
