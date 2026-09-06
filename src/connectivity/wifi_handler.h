// ============================================
// connectivity/wifi_handler.h
// WiFi Connection Handler with Fallback
// ============================================

#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <WiFi.h>
#include <AsyncUDP.h>
#include "../config/config.h"
#include "../config/pins_config.h"

// ============================================
// WIFI STATE
// ============================================

struct WiFiState {
    bool connected = false;
    bool apMode = false;
    int networkIndex = 0;
    unsigned long lastConnectAttempt = 0;
    unsigned long lastHeartbeat = 0;
    byte connectAttempts = 0;
    IPAddress localIP;
    IPAddress apIP = IPAddress(192, 168, 1, 1);
};

static WiFiState wifiState;
static AsyncUDP wifiUDP;

// WiFi networks from config
const char* ssidList[] = {
    WIFI_SSID,
    "Backup_Network_1",
    "Backup_Network_2"
};

const char* passList[] = {
    WIFI_PASSWORD,
    "",
    ""
};

const int WIFI_NETWORKS_COUNT = 3;
const int WIFI_CONNECT_TIMEOUT = 15000;  // 15 seconds
const int WIFI_RETRY_INTERVAL = 5000;    // 5 seconds between retries

// ============================================
// WIFI INITIALIZATION
// ============================================

bool wifi_begin_connection() {
    if (wifiState.networkIndex >= WIFI_NETWORKS_COUNT) {
        // All networks tried, start Access Point
        Serial.println("[WiFi] All networks failed, starting Access Point...");
        return wifi_start_ap();
    }
    
    unsigned long now = millis();
    if (now - wifiState.lastConnectAttempt < WIFI_RETRY_INTERVAL) {
        return false;  // Too soon to retry
    }
    
    wifiState.lastConnectAttempt = now;
    wifiState.connectAttempts++;
    
    const char* ssid = ssidList[wifiState.networkIndex];
    const char* pass = passList[wifiState.networkIndex];
    
    Serial.print("[WiFi] Attempting to connect to: ");
    Serial.print(ssid);
    Serial.print(" (attempt ");
    Serial.print(wifiState.connectAttempts);
    Serial.println(")");
    
    if (strlen(pass) > 0) {
        WiFi.begin(ssid, pass);
    } else {
        WiFi.begin(ssid);
    }
    
    return true;
}

// ============================================
// CHECK WIFI STATUS
// ============================================

bool wifi_check_connection() {
    wl_status_t status = WiFi.status();
    
    switch (status) {
        case WL_CONNECTED:
            if (!wifiState.connected) {
                wifiState.connected = true;
                wifiState.localIP = WiFi.localIP();
                Serial.print("[WiFi] Connected! IP: ");
                Serial.println(wifiState.localIP);
            }
            return true;
            
        case WL_NO_SSID_AVAIL:
            Serial.print("[WiFi] Network not found: ");
            Serial.println(ssidList[wifiState.networkIndex]);
            wifiState.networkIndex++;
            return false;
            
        case WL_CONNECT_FAILED:
            Serial.print("[WiFi] Connection failed: ");
            Serial.println(ssidList[wifiState.networkIndex]);
            wifiState.networkIndex++;
            return false;
            
        case WL_DISCONNECTED:
            if (millis() - wifiState.lastConnectAttempt > WIFI_CONNECT_TIMEOUT) {
                Serial.println("[WiFi] Connection timeout, trying next network...");
                wifiState.networkIndex++;
                wifiState.connectAttempts = 0;
                wifi_begin_connection();
            }
            return false;
            
        default:
            return false;
    }
}

// ============================================
// ACCESS POINT MODE
// ============================================

bool wifi_start_ap() {
    Serial.println("[WiFi] Starting Access Point mode...");
    Serial.print("[WiFi] SSID: ");
    Serial.println(WIFI_AP_SSID);
    
    if (WiFi.mode(WIFI_AP)) {
        if (WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, 1, false, 4)) {
            wifiState.apMode = true;
            wifiState.localIP = WiFi.softAPIP();
            Serial.print("[WiFi] AP IP: ");
            Serial.println(wifiState.localIP);
            return true;
        }
    }
    
    Serial.println("[WiFi] ERROR: Failed to start AP!");
    return false;
}

// ============================================
// UDP SERVER FOR WIFI
// ============================================

bool wifi_start_udp() {
    if (!wifiState.connected && !wifiState.apMode) {
        Serial.println("[WiFi] ERROR: Not connected or in AP mode!");
        return false;
    }
    
    Serial.print("[WiFi] Starting UDP listener on port ");
    Serial.println(AOG_PORT);
    
    if (wifiUDP.listen(AOG_PORT)) {
        wifiUDP.onPacket([](AsyncUDPPacket packet) {
            Serial.print("[WiFi] Received from ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(" - ");
            Serial.print(packet.length());
            Serial.println(" bytes");
        });
        
        Serial.println("[WiFi] UDP listener started successfully");
        return true;
    } else {
        Serial.println("[WiFi] ERROR: UDP listener failed!");
        return false;
    }
}

// ============================================
// SEND DATA OVER WIFI
// ============================================

bool wifi_send_data(byte *data, int len, IPAddress destIP, int destPort) {
    if (!wifiState.connected && !wifiState.apMode) {
        return false;
    }
    
    wifiUDP.writeTo(data, len, destIP, destPort);
    return true;
}

// ============================================
// SEND BROADCAST
// ============================================

bool wifi_send_broadcast(byte *data, int len) {
    if (!wifiState.connected && !wifiState.apMode) {
        return false;
    }
    
    IPAddress broadcastIP = WiFi.localIP();
    broadcastIP[3] = 255;
    
    return wifi_send_data(data, len, broadcastIP, AOG_PORT);
}

// ============================================
// HEARTBEAT
// ============================================

bool wifi_send_heartbeat() {
    unsigned long now = millis();
    
    if (now - wifiState.lastHeartbeat < 1000) {
        return true;
    }
    
    wifiState.lastHeartbeat = now;
    
    byte heartbeat[] = {0x80, 0x81, 0x7E, 0xC8, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
    int crc = 0;
    for (int i = 2; i < 9; i++) {
        crc += heartbeat[i];
    }
    heartbeat[9] = crc & 0xFF;
    
    return wifi_send_broadcast(heartbeat, sizeof(heartbeat));
}

// ============================================
// CONNECTION STATUS
// ============================================

bool wifi_is_connected() {
    return wifiState.connected || wifiState.apMode;
}

IPAddress wifi_get_local_ip() {
    return wifiState.localIP;
}

#endif // WIFI_HANDLER_H
