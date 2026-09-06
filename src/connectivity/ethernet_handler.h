// ============================================
// connectivity/ethernet_handler.h
// W5500 Ethernet Module Handler
// ============================================

#ifndef ETHERNET_HANDLER_H
#define ETHERNET_HANDLER_H

#include <Ethernet.h>
#include <EthernetUdp.h>
#include "../config/config.h"
#include "../config/pins_config.h"

// ============================================
// ETHERNET STATE
// ============================================

struct EthernetState {
    bool connected = false;
    bool initialized = false;
    unsigned long lastConnectAttempt = 0;
    unsigned long lastHeartbeat = 0;
    byte connectAttempts = 0;
    IPAddress localIP;
    IPAddress gatewayIP;
    IPAddress dnsIP;
};

EthernetState ethState;
EthernetUDP ethUdpServer;
EthernetUDP ethUdpClient;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// ============================================
// ETHERNET INITIALIZATION
// ============================================

bool ethernet_init() {
    Serial.println("[ETH] Initializing W5500 SPI...");
    
    // Configure SPI pins
    pinMode(W5500_CS_PIN, OUTPUT);
    digitalWrite(W5500_CS_PIN, HIGH);
    pinMode(W5500_RST_PIN, OUTPUT);
    digitalWrite(W5500_RST_PIN, HIGH);
    delay(100);
    
    // Reset W5500
    digitalWrite(W5500_RST_PIN, LOW);
    delay(100);
    digitalWrite(W5500_RST_PIN, HIGH);
    delay(500);
    
    Serial.println("[ETH] W5500 reset complete");
    
    // Initialize Ethernet with DHCP or static IP
    if (ETHERNET_DYNAMIC_IP) {
        Serial.println("[ETH] Starting DHCP...");
        if (Ethernet.begin(mac)) {
            ethState.localIP = Ethernet.localIP();
            ethState.gatewayIP = Ethernet.gatewayIP();
            ethState.dnsIP = Ethernet.dnsServerIP();
            
            Serial.print("[ETH] DHCP OK - IP: ");
            Serial.println(ethState.localIP);
            
            ethState.initialized = true;
            return true;
        } else {
            Serial.println("[ETH] ERROR: DHCP failed!");
            return false;
        }
    } else {
        // Static IP
        IPAddress staticIP(ETHERNET_IP_1, ETHERNET_IP_2, ETHERNET_IP_3, ETHERNET_IP_4);
        IPAddress gateway(ETHERNET_GATEWAY_1, ETHERNET_GATEWAY_2, ETHERNET_GATEWAY_3, ETHERNET_GATEWAY_4);
        IPAddress subnet(ETHERNET_SUBNET_1, ETHERNET_SUBNET_2, ETHERNET_SUBNET_3, ETHERNET_SUBNET_4);
        
        Ethernet.begin(mac, staticIP, gateway, gateway, subnet);
        ethState.localIP = staticIP;
        ethState.gatewayIP = gateway;
        
        Serial.print("[ETH] Static IP configured - IP: ");
        Serial.println(staticIP);
        
        ethState.initialized = true;
        return true;
    }
}

// ============================================
// UDP SERVER START
// ============================================

bool ethernet_start_udp() {
    if (!ethState.initialized) {
        Serial.println("[ETH] ERROR: Ethernet not initialized!");
        return false;
    }
    
    Serial.print("[ETH] Starting UDP server on port ");
    Serial.println(AOG_PORT);
    
    if (ethUdpServer.begin(AOG_PORT)) {
        Serial.println("[ETH] UDP server started successfully");
        ethState.connected = true;
        return true;
    } else {
        Serial.println("[ETH] ERROR: UDP server failed to start!");
        return false;
    }
}

// ============================================
// RECEIVE DATA FROM AOG
// ============================================

int ethernet_receive_data(byte *buffer, int maxLen) {
    int packetSize = ethUdpServer.parsePacket();
    
    if (packetSize) {
        Serial.print("[ETH] Received packet: ");
        Serial.print(packetSize);
        Serial.println(" bytes");
        
        // Read packet into buffer
        int len = ethUdpServer.read(buffer, maxLen);
        
        // Store sender IP for response
        IPAddress remoteIP = ethUdpServer.remoteIP();
        Serial.print("[ETH] From: ");
        Serial.println(remoteIP);
        
        return len;
    }
    
    return 0;
}

// ============================================
// SEND DATA TO AOG
// ============================================

bool ethernet_send_data(byte *data, int len, IPAddress destIP, int destPort) {
    if (!ethState.connected) {
        Serial.println("[ETH] ERROR: Not connected!");
        return false;
    }
    
    ethUdpClient.beginPacket(destIP, destPort);
    ethUdpClient.write(data, len);
    ethUdpClient.endPacket();
    
    return true;
}

// ============================================
// SEND BROADCAST TO AOG
// ============================================

bool ethernet_send_broadcast(byte *data, int len) {
    if (!ethState.connected) {
        return false;
    }
    
    // Calculate broadcast IP (x.x.x.255)
    IPAddress broadcastIP = ethState.localIP;
    broadcastIP[3] = 255;
    
    return ethernet_send_data(data, len, broadcastIP, AOG_PORT);
}

// ============================================
// HEARTBEAT - AgIO DISCOVERY
// ============================================

bool ethernet_send_heartbeat() {
    unsigned long now = millis();
    
    // Send heartbeat every 1 second
    if (now - ethState.lastHeartbeat < 1000) {
        return true;
    }
    
    ethState.lastHeartbeat = now;
    
    // Heartbeat packet: [0x80, 0x81, moduleID, subID, length, ...data, CRC]
    byte heartbeat[] = {0x80, 0x81, 0x7E, 0xC8, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
    // CRC is calculated on bytes 2 to length-2
    int crc = 0;
    for (int i = 2; i < 9; i++) {
        crc += heartbeat[i];
    }
    heartbeat[9] = crc & 0xFF;
    
    return ethernet_send_broadcast(heartbeat, sizeof(heartbeat));
}

// ============================================
// CONNECTION CHECK
// ============================================

bool ethernet_is_connected() {
    if (!ethState.initialized) {
        return false;
    }
    
    // Check Ethernet link status
    if (Ethernet.linkStatus() == LinkON) {
        ethState.connected = true;
        return true;
    } else {
        ethState.connected = false;
        return false;
    }
}

// ============================================
// MAINTAIN ETHERNET CONNECTION
// ============================================

void ethernet_maintain() {
    // Call Ethernet.maintain() periodically
    Ethernet.maintain();
}

#endif // ETHERNET_HANDLER_H
