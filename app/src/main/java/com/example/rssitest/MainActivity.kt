package com.example.rssitest

import android.Manifest
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.net.wifi.ScanResult
import android.net.wifi.WifiManager
import android.os.*
import android.util.Log
import android.widget.ArrayAdapter
import android.widget.ListView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import java.util.Locale
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * 실시간 Wi-Fi RSSI 스캔 → Unscented Kalman Filter(UKF)로 거리 보정.
 * 기반: iJIM 2020 “Accuracy Improvement … using UKF”.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var wifiManager: WifiManager
    private lateinit var wifiListAdapter: ArrayAdapter<String>
    private val ukfMap = mutableMapOf<String, WifiUkf>()

    /* ---------- 스캔 주기 설정 ---------- */
    private val SCAN_INTERVAL_MS = 5_000L                     // 5 초
    private val scanHandler = Handler(Looper.getMainLooper())
    private val scanRunnable = object : Runnable {
        override fun run() {
            try {
                wifiManager.startScan()
            } catch (e: Exception) {
                Log.w("MainActivity", "startScan() 실패: ${e.message}")
            }
            scanHandler.postDelayed(this, SCAN_INTERVAL_MS)
        }
    }

    companion object {
        private const val RSSI_AT_1M = -40     // [dBm] 1 m 기준
        private const val PATH_LOSS_EXPONENT = 3.0
        private const val WALL_LOSS_DB = 1
        private const val MIN_RSSI = -60
    }

    /* ---------- 생명주기 ---------- */
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        /* ListView 설정 */
        val wifiListView: ListView = findViewById(R.id.wifiListView)
        wifiListAdapter = ArrayAdapter(this,
            android.R.layout.simple_list_item_1, mutableListOf())
        wifiListView.adapter = wifiListAdapter

        wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        if (!wifiManager.isWifiEnabled) wifiManager.isWifiEnabled = true

        /* 위치 권한 확인 & 요청 */
        val permissionLauncher = registerForActivityResult(
            ActivityResultContracts.RequestMultiplePermissions()
        ) { granted ->
            if (granted.values.all { it }) startScanningLoop()
        }
        if (!hasLocationPermission()) {
            permissionLauncher.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                )
            )
        } else startScanningLoop()

        /* 스캔 결과 수신 브로드캐스트 */
        registerReceiver(scanReceiver,
            IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION))
    }

    override fun onDestroy() {
        super.onDestroy()
        unregisterReceiver(scanReceiver)
        scanHandler.removeCallbacks(scanRunnable)
    }

    /* ---------- 권한 ---------- */
    private fun hasLocationPermission(): Boolean =
        (ActivityCompat.checkSelfPermission(
            this, Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED)

    /* ---------- 스캔 루프 ---------- */
    private fun startScanningLoop() {
        scanHandler.post(scanRunnable)          // 즉시 시작
    }

    /* ---------- BroadcastReceiver ---------- */
    private val scanReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            val results = wifiManager.scanResults
            updateWithUkf(results)
        }
    }

    /* ---------- RSSI → 거리 ---------- */
    private fun rssiToDistance(rssi: Int, walls: Int = 1): Double {
        val totalLoss = (RSSI_AT_1M - rssi) - (walls * WALL_LOSS_DB)
        return 10.0.pow(totalLoss / (10 * PATH_LOSS_EXPONENT))
    }

    /* ---------- UKF 적용 & UI 갱신 ---------- */
    private fun updateWithUkf(results: List<ScanResult>) {
        val list = mutableListOf<String>()

        results.filter { it.level >= MIN_RSSI }.forEach { res ->
            val raw = rssiToDistance(res.level)
            val ukf = ukfMap.getOrPut(res.BSSID) { WifiUkf(initial = raw) }
            val filt = ukf.update(raw)

            list += "${res.SSID.ifBlank { res.BSSID }}  " +
                    "(${res.level} dBm)  " +
                    "≈ ${String.format(Locale.US, "%.2f m", filt)}"
        }

        if (list.isEmpty()) list += "주변 Wi-Fi 신호가 약합니다."

        runOnUiThread {
            wifiListAdapter.clear()
            wifiListAdapter.addAll(list)
            wifiListAdapter.notifyDataSetChanged()
        }
    }
}

/* -------------------------------------------------------------------------- */
/* ---------------------------  UKF 1-차원 구현  ----------------------------- */
/* -------------------------------------------------------------------------- */

class WifiUkf(
    initial: Double,
    private var P: Double = 1.0,
    private val Q: Double = 0.1,
    private val R: Double = 0.5
) {
    private var x = initial
    private val alpha = 1e-3
    private val kappa = 0.0
    private val beta = 2.0

    fun update(z: Double): Double {
        /* ---------- 예측 ---------- */
        val xPred = x
        val PPred = P + Q

        /* ---------- Σ-점 ---------- */
        val n = 1                    // 상태 차원
        val lambda = alpha * alpha * (n + kappa) - n
        val c = n + lambda
        val sqrtTerm = sqrt(c * PPred)
        val sigma = doubleArrayOf(xPred, xPred + sqrtTerm, xPred - sqrtTerm)

        val wm0 = lambda / c
        val wc0 = wm0 + (1 - alpha * alpha + beta)
        val wi = 1.0 / (2 * c)
        val wm = doubleArrayOf(wm0, wi, wi)
        val wc = doubleArrayOf(wc0, wi, wi)

        var zPred = 0.0
        for (i in sigma.indices) zPred += wm[i] * sigma[i]

        var S = R
        var Cxz = 0.0
        for (i in sigma.indices) {
            val dz = sigma[i] - zPred
            val dx = sigma[i] - xPred
            S += wc[i] * dz * dz
            Cxz += wc[i] * dx * dz
        }

        val K = Cxz / S
        x = xPred + K * (z - zPred)
        P = PPred - K * S * K

        return x
    }
}
