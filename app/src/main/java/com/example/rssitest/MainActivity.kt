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

class MainActivity : AppCompatActivity() {
    private lateinit var wifiManager: WifiManager
    private lateinit var wifiListView: ListView
    private lateinit var wifiListAdapter: ArrayAdapter<String>

    companion object {
        private const val RSSI_AT_1M         = -40         // 1미터 거리에서의 RSSI 값 (환경에 따라 조정)
        private const val PATH_LOSS_EXPONENT = 3.0 // 실내 환경 감쇠 계수
        private const val WALL_LOSS          = 1
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        wifiListView = findViewById(R.id.wifiListView)
        wifiListAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, mutableListOf())
        wifiListView.adapter = wifiListAdapter

        wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager

        if (!wifiManager.isWifiEnabled) {
            wifiManager.isWifiEnabled = true
        }

        // 브로드캐스트 리시버 등록 (스캔 결과 수신 시 filterBySignalStrength 호출)
        registerReceiver(object : BroadcastReceiver() {
            override fun onReceive(context: Context?, intent: Intent?) {
                Log.d("WiFiScan", "Received scan results broadcast")
                val scanResults = wifiManager.scanResults
                filterBySignalStrength(scanResults, -60) // -70dBm 이상의 신호만 필터링
            }
        }, IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION))

        wifiManager.startScan()
        Log.d("WiFiScan", "Started Wi-Fi scan")
    }

    // RSSI 값에 따라 거리를 계산하는 함수 (단위: 미터)
    private fun calculateDistance(rssi: Int, walls: Int): Double {
        val totalLoss = (RSSI_AT_1M - rssi) - (walls * WALL_LOSS)
        return 10.0.pow(totalLoss / (10 * PATH_LOSS_EXPONENT))
    }

    private fun filterBySignalStrength(scanResults: List<ScanResult>, minRssi: Int) {
        Log.d("WiFiScan", "Scan result size: ${scanResults.size}") // 🔥 로그 추가
        val filteredResults = scanResults.filter { it.level >= minRssi }

        if (filteredResults.isEmpty()) {
            Log.d("WiFiScan", "No Wi-Fi networks found.") // 🔥 로그 추가
        }

        val wifiInfoList = if (filteredResults.isEmpty()) {
            listOf("Wi-Fi 네트워크를 찾을 수 없습니다.")
        } else {
            filteredResults.map {
                val distance = calculateDistance(it.level, 1)
                // 소수점 둘째 자리까지 포맷팅
                val distanceFormatted = String.format(Locale.US, "%.2f", distance)
                "${it.SSID} - ${it.level}dBm, 거리: ${distanceFormatted}m"
            }
        }

        runOnUiThread {
            Log.d("WiFiScan", "Updating ListView with ${wifiInfoList.size} items") // 🔥 로그 추가
            wifiListAdapter.clear()
            wifiListAdapter.addAll(wifiInfoList)
            wifiListAdapter.notifyDataSetChanged()
        }
    }
}
