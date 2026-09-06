// ============================================
// connectivity/network_manager.h
// Network Connection Manager
// Handles Ethernet-first with WiFi fallback
// ============================================

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "ethernet_handler.h"
#include "wifi_handler.h"
#include "../config/config.h"

// ============================================
// NETWORK STATE
// ============================================

enum NetworkMode {
    NET_DISCONNECTED,
    NET_ETHERNET,
    NET_WIFI_STA,
    NET_WIFI_AP
};

struct NetworkState {
    NetworkMode mode = NET_DISCONNECTED;
    unsigned long lastModeChange = 0;
    unsigned long ethernetFailTime = 0;
    bool ethernetFailed = false;
    IPAddress destIP;
}

netNetwork;

// ============================================
// INITIALIZATION
// ============================================

bool network_init() {
    Serial.println("\n[NET] Starting network initialization...");
    
    // Try Ethernet first
    if (ETHERNET_ENABLED) {
        Serial.println("[NET] Attempting Ethernet connection...");
        if (ethernet_init()) {
            if (ethernet_start_udp()) {
                netNetwork.mode = NET_ETHERNET;
                netNetwork.lastModeChange = millis();
                Serial.println("[NET] ✓ Ethernet initialized successfully");
                return true;
            }
        }
        Serial.println("[NET] ✗ Ethernet failed, falling back to WiFi");
    }
    
    // Fallback to WiFi
    if (WIFI_ENABLED) {
        Serial.println("[NET] Attempting WiFi connection...");
        wifi_begin_connection();
        return true;  // WiFi init returns true when started (connection in progress)
    }
    
    Serial.println("[NET] ERROR: No network available!");
    return false;
}

// ============================================
// MAINTAIN CONNECTION
// ============================================

void network_maintain() {
    unsigned long now = millis();
    
    switch (netNetwork.mode) {
        case NET_ETHERNET:
            ethernet_maintain();
            
            // Check if Ethernet connection is still valid
            if (!ethernet_is_connected()) {
                Serial.println("[NET] ⚠ Ethernet connection lost! Switching to WiFi...");
                netNetwork.ethernetFailed = true;
                netNetwork.ethernetFailTime = now;
                netNetwork.mode = NET_WIFI_STA;
                wifi_begin_connection();
            } else {
                // Connection OK, send heartbeat
                ethernet_send_heartbeat();
            }
            break;
            
        case NET_WIFI_STA:
        case NET_WIFI_AP:
            // Check WiFi connection status
            if (netNetwork.mode == NET_WIFI_STA) {
                if (!wifi_check_connection()) {
                    // Still trying to connect
                    if (!wifiState.apMode) {
                        wifi_begin_connection();
                    }
                } else {
                    // WiFi connected successfully
                    if (netNetwork.mode != NET_WIFI_STA) {
                        Serial.println("[NET] ✓ WiFi connected");
                        netNetwork.mode = NET_WIFI_STA;
                        netNetwork.lastModeChange = now;
                    }
                }
            }
            
            // If WiFi AP mode, check if Ethernet is back
            if (wifiState.apMode && ETHERNET_ENABLED && netNetwork.ethernetFailed) {
                if (now - netNetwork.ethernetFailTime > ETHERNET_FALLBACK_TO_WIFI_TIME) {
                    Serial.println("[NET] Retrying Ethernet connection...");
                    if (ethernet_init() && ethernet_start_udp()) {
                        netNetwork.mode = NET_ETHERNET;
                        netNetwork.lastModeChange = now;
                        netNetwork.ethernetFailed = false;
                        Serial.println("[NET] ✓ Ethernet reconnected");
                    }
                }
            }
            
            if (wifi_is_connected()) {
                wifi_send_heartbeat();
            }
            break;
            
        case NET_DISCONNECTED:
            // Try to connect
            if (ETHERNET_ENABLED && !netNetwork.ethernetFailed) {
                if (ethernet_init() && ethernet_start_udp()) {
                    netNetwork.mode = NET_ETHERNET;
                    netNetwork.lastModeChange = now;
                }
            } else if (WIFI_ENABLED) {
                wifi_begin_connection();
            }
            break;
    }
}

// ============================================
// SEND DATA
// ============================================

bool network_send_data(byte *data, int len) {
    switch (netNetwork.mode) {
        case NET_ETHERNET:
            return ethernet_send_broadcast(data, len);
            
        case NET_WIFI_STA:
        case NET_WIFI_AP:
            return wifi_send_broadcast(data, len);
            
        default:
            return false;
    }
}

bool network_send_to_address(byte *data, int len, IPAddress destIP, int destPort) {
    switch (netNetwork.mode) {
        case NET_ETHERNET:
            return ethernet_send_data(data, len, destIP, destPort);
            
        case NET_WIFI_STA:
        case NET_WIFI_AP:
            return wifi_send_data(data, len, destIP, destPort);
            
        default:
            return false;
    }
}

// ============================================
// GET STATUS
// ============================================

bool network_is_connected() {
    return netNetwork.mode != NET_DISCONNECTED;
}

const char* network_get_mode_string() {
    switch (netNetwork.mode) {
        case NET_ETHERNET:
            return "Ethernet";
        case NET_WIFI_STA:
            return "WiFi (Station)";
        case NET_WIFI_AP:
            return "WiFi (Access Point)";
        case NET_DISCONNECTED:
            return "Disconnected";
        default:
            return "Unknown";
    }
}

IPAddress network_get_local_ip() {
    switch (netNetwork.mode) {
        case NET_ETHERNET:
            return ethState.localIP;
        case NET_WIFI_STA:
        case NET_WIFI_AP:
            return wifiState.localIP;
        default:
            return IPAddress(0, 0, 0, 0);
    }
}

#endif // NETWORK_MANAGER_H
