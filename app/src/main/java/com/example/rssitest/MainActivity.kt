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
import kotlin.math.log10
import kotlin.math.pow

/**
 * Wi-Fi RSSI → 거리 추정 데모
 *  • 모델 3종(FSPL / ITU Indoor / EWLM) 중 하나를 선택해 사용
 *  • RSSI 노이즈는 단순 칼만 필터로 1차 저감
 *  • 20 초마다 스캔 반복
 *
 * ※  위치 권한(ACCESS_FINE_LOCATION)과 Android 13+ 추가 권한(NEARBY-WIFI-DEVICES) 필요
 */
class MainActivity : AppCompatActivity() {

    /* ---------- Android UI / Wi-Fi ---------- */

    private lateinit var wifiManager: WifiManager
    private lateinit var wifiListView: ListView
    private lateinit var wifiAdapter: ArrayAdapter<String>

    /* ---------- RSSI 후처리 ---------- */

    private val kalman = RssiKalman()                     // 노이즈 억제
    private var model: PathLossModel = PathLossModel.Ewlm() // 기본 모델

    /* ---------- 생명주기 ---------- */

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        wifiListView = findViewById(R.id.wifiListView)
        wifiAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, mutableListOf())
        wifiListView.adapter = wifiAdapter

        wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        if (!wifiManager.isWifiEnabled) wifiManager.isWifiEnabled = true

        /* 스캔 결과 브로드캐스트 수신 */
        registerReceiver(wifiScanReceiver, IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION))

        startPeriodicScan()
    }

    override fun onDestroy() {
        super.onDestroy()
        unregisterReceiver(wifiScanReceiver)
    }

    /* ---------- 브로드캐스트 리시버 ---------- */

    private val wifiScanReceiver = object : BroadcastReceiver() {
        override fun onReceive(ctx: Context?, intent: Intent?) {
            val results = wifiManager.scanResults
            render(results, minRssi = -70)
        }
    }

    /* ---------- 주기적 스캔 ---------- */

    private fun startPeriodicScan() {
        wifiManager.startScan()
        /* 20 초마다 재스캔 */
        wifiListView.postDelayed({ startPeriodicScan() }, 20_000)
    }

    /* ---------- 스캔 결과 처리 & UI ---------- */

    private fun render(results: List<ScanResult>, minRssi: Int) {
        val rows = if (results.isEmpty()) {
            listOf("Wi-Fi 네트워크를 찾을 수 없습니다.")
        } else {
            results.filter { it.level >= minRssi }
                .sortedBy { it.level }
                .map { sr ->
                    val clean = kalman.filter(sr.level)          // RSSI 필터링
                    val d     = model.distance(clean, sr.frequency, walls = 1)
                    val df    = String.format("%.2f", d)
                    "${sr.SSID.ifBlank { "(hidden)" }}  •  ${clean} dBm  •  ${df} m"
                }.ifEmpty { listOf("신호 세기 −70 dBm 이상 AP 없음") }
        }

        wifiAdapter.clear()
        wifiAdapter.addAll(rows)
        wifiAdapter.notifyDataSetChanged()
    }
}

/* ========================================================================== */
/*                                RSSI 모델                                  */
/* ========================================================================== */

sealed class PathLossModel {

    abstract fun distance(rssi: Int, freqMHz: Int, walls: Int = 0): Double

    /** 자유공간손실(FSPL) – LOS 환경 */
    data class Fspl(val txPowerAt1m: Int = -43) : PathLossModel() {
        override fun distance(rssi: Int, freqMHz: Int, walls: Int): Double {
            val exponent = (txPowerAt1m - rssi - 20.0 * log10(freqMHz.toDouble()) + 27.55) / 20.0
            return 10.0.pow(exponent)
        }
    }

    /** ITU Indoor – 사무실/상가 표준 */
    data class ItuIndoor(
        val n: Double = 28.0,       // 감쇠 지수
        val floorLoss: Int = 0      // 층간 손실(필요 시)
    ) : PathLossModel() {
        override fun distance(rssi: Int, freqMHz: Int, walls: Int): Double {
            val pl = -rssi // 송신 전력을 모를 때 PL ≈ −RSSI
            val exponent = (pl + 28 - 20 * log10(freqMHz.toDouble()) - floorLoss) / n
            return 10.0.pow(exponent)
        }
    }

    /** Log-Distance + 벽 보정(EWLM) – 복잡한 실내 */
    data class Ewlm(
        val rssiAt1m: Int = -40,
        val gamma: Double = 3.0,
        val wallLoss: Int = 3       // dB/벽
    ) : PathLossModel() {
        override fun distance(rssi: Int, freqMHz: Int, walls: Int): Double {
            val loss = (rssiAt1m - rssi) - walls * wallLoss
            return 10.0.pow(loss / (10 * gamma))
        }
    }
}

/* ========================================================================== */
/*                              RSSI 칼만 필터                                */
/* ========================================================================== */

class RssiKalman(
    private val q: Double = 0.001,  // 프로세스 노이즈
    private val r: Double = 2.0     // 측정 노이즈
) {
    private var p = 1.0
    private var x = 0.0
    private var init = false

    fun filter(measurement: Int): Int {
        if (!init) { x = measurement.toDouble(); init = true }
        // 예측
        p += q
        // 보정
        val k = p / (p + r)
        x += k * (measurement - x)
        p *= (1 - k)
        return x.toInt()
    }
}
