// ============================================
// protocol/aog_protocol.h
// AgOpenGPS Communication Protocol
// Data packaging and CRC handling
// ============================================

#ifndef AOG_PROTOCOL_H
#define AOG_PROTOCOL_H

#include <stdint.h>
#include <string.h>
#include "../config/config.h"

// ============================================
// PROTOCOL CONSTANTS
// ============================================

// Frame headers
const byte FRAME_HEADER_1 = 0x80;
const byte FRAME_HEADER_2 = 0x81;

// Autosteer message IDs
const byte AUTOSTEER_FROM_AOG_HEADER = 0x7F;
const byte AUTOSTEER_TO_AOG_HEADER = 0x7F;
const byte AUTOSTEER_DATA_ID = 0xA0;

// GPS message IDs
const byte GPS_FROM_AOG_HEADER = 0x80;
const byte GPS_TO_AOG_HEADER = 0x80;
const byte GPS_DATA_ID = 0xB8;

// IMU message IDs
const byte IMU_FROM_AOG_HEADER = 0x7B;
const byte IMU_TO_AOG_HEADER = 0x7B;
const byte IMU_DATA_ID = 0xC0;

// Section Control message IDs
const byte SECTION_FROM_AOG_HEADER = 0x7B;
const byte SECTION_TO_AOG_HEADER = 0x7B;
const byte SECTION_DATA_ID = 0xEA;

// Diagnostic message IDs
const byte DIAG_DATA_ID = 0xCC;

// Buffer sizes
const int AOG_MAX_PACKET_SIZE = 256;
const int AOG_RX_BUFFER_SIZE = 512;

// ============================================
// AUTOSTEER PACKET
// ============================================

#pragma pack(push, 1)
struct AutosteerPacketToAOG {
    byte header1 = FRAME_HEADER_1;        // 0x80
    byte header2 = FRAME_HEADER_2;        // 0x81
    byte deviceID = AUTOSTEER_FROM_AOG_HEADER;  // 0x7F
    byte dataID = AUTOSTEER_DATA_ID;      // 0xA0
    byte payloadLength;                   // Data length (excluding headers, length, CRC)
    
    // Payload
    int16_t steerAngle = 0;               // Steering angle (-32768 to 32767, 0.01 degree units)
    int16_t headingError = 0;             // Heading error (-32768 to 32767, 0.01 degree units)
    uint8_t pwmValue = 0;                 // Motor PWM (0-255)
    uint8_t motorDirection = 0;           // 0=LEFT, 1=RIGHT
    uint16_t steerAngleFeedback = 0;      // Feedback steering angle
    uint16_t steerAngleVelocity = 0;      // Rate of steering change
    uint16_t systemStatus = 0;            // Status flags
    
    byte checksumHigh;                    // CRC high byte
    byte checksumLow;                     // CRC low byte
};
#pragma pack(pop)

// ============================================
// GPS PACKET
// ============================================

#pragma pack(push, 1)
struct GPSPacketToAOG {
    byte header1 = FRAME_HEADER_1;        // 0x80
    byte header2 = FRAME_HEADER_2;        // 0x81
    byte deviceID = GPS_FROM_AOG_HEADER;  // 0x80
    byte dataID = GPS_DATA_ID;            // 0xB8
    byte payloadLength;                   // Data length
    
    // Payload
    double latitude = 0.0;                // Latitude (degrees)
    double longitude = 0.0;               // Longitude (degrees)
    float speed = 0.0f;                   // Speed (km/h)
    float heading = 0.0f;                 // Heading (degrees)
    uint16_t gpsQuality = 0;              // GPS fix quality (0=none, 1=GPS, 2=DGPS, 4=RTK)
    uint8_t numSatellites = 0;            // Number of satellites
    uint16_t hdop = 0;                    // Horizontal dilution (0.01 precision)
    uint16_t altitude = 0;                // Altitude (0.1m precision)
    uint8_t glonassEnabled = 0;           // GLONASS flag
    
    byte checksumHigh;                    // CRC high byte
    byte checksumLow;                     // CRC low byte
};
#pragma pack(pop)

// ============================================
// IMU PACKET
// ============================================

#pragma pack(push, 1)
struct IMUPacketToAOG {
    byte header1 = FRAME_HEADER_1;        // 0x80
    byte header2 = FRAME_HEADER_2;        // 0x81
    byte deviceID = IMU_FROM_AOG_HEADER;  // 0x7B
    byte dataID = IMU_DATA_ID;            // 0xC0
    byte payloadLength;                   // Data length
    
    // Payload
    float heading = 0.0f;                 // Heading (degrees, 0-360)
    float roll = 0.0f;                    // Roll (degrees, -90 to +90)
    float pitch = 0.0f;                   // Pitch (degrees, -90 to +90)
    uint8_t calibrationStatus = 0;        // Calibration status (0-3)
    uint16_t systemStatus = 0;            // IMU system status
    
    byte checksumHigh;                    // CRC high byte
    byte checksumLow;                     // CRC low byte
};
#pragma pack(pop)

// ============================================
// SECTION CONTROL PACKET
// ============================================

#pragma pack(push, 1)
struct SectionPacketToAOG {
    byte header1 = FRAME_HEADER_1;        // 0x80
    byte header2 = FRAME_HEADER_2;        // 0x81
    byte deviceID = SECTION_FROM_AOG_HEADER;  // 0x7B
    byte dataID = SECTION_DATA_ID;        // 0xEA
    byte payloadLength;                   // Data length
    
    // Payload
    uint16_t sectionStates = 0;           // Bitmask: 16 sections
    uint8_t mainSwitchState = 0;          // Main ON/OFF
    uint16_t pressureLeft = 0;            // Pressure sensor left (0.1 bar precision)
    uint16_t pressureRight = 0;           // Pressure sensor right (0.1 bar precision)
    uint16_t flowRateLeft = 0;            // Flow rate left (0.1 LPM precision)
    uint16_t flowRateRight = 0;           // Flow rate right (0.1 LPM precision)
    uint8_t uTurnState = 0;               // U-turn relay state
    
    byte checksumHigh;                    // CRC high byte
    byte checksumLow;                     // CRC low byte
};
#pragma pack(pop)

// ============================================
// DIAGNOSTIC PACKET
// ============================================

#pragma pack(push, 1)
struct DiagnosticPacketToAOG {
    byte header1 = FRAME_HEADER_1;        // 0x80
    byte header2 = FRAME_HEADER_2;        // 0x81
    byte deviceID = 0x7B;                 // Generic
    byte dataID = DIAG_DATA_ID;           // 0xCC
    byte payloadLength;                   // Data length
    
    // Payload
    uint32_t uptime = 0;                  // Uptime (seconds)
    uint16_t freeRam = 0;                 // Free memory (bytes)
    uint8_t systemStatus = 0;             // Overall status
    uint16_t errorCount = 0;              // Total errors
    uint16_t packetsSent = 0;             // Packets transmitted
    uint16_t packetsReceived = 0;         // Packets received
    uint16_t crcErrors = 0;               // CRC errors
    float cpuLoadPercent = 0.0f;          // CPU load
    
    byte checksumHigh;                    // CRC high byte
    byte checksumLow;                     // CRC low byte
};
#pragma pack(pop)

// ============================================
// CRC16 CALCULATION
// ============================================

uint16_t crc16_calculate(const byte *data, int length) {
    uint16_t crc = 0;
    
    for (int i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x10000) {
                crc ^= 0x1021;  // CRC-CCITT polynomial
            }
        }
    }
    
    return crc;
}

void crc16_set_checksum(byte *packet, int dataLength) {
    // Calculate CRC over header + payload (excluding CRC bytes)
    uint16_t crc = crc16_calculate(packet, dataLength);
    
    // Set CRC bytes at the end
    packet[dataLength] = (crc >> 8) & 0xFF;      // High byte
    packet[dataLength + 1] = crc & 0xFF;          // Low byte
}

bool crc16_verify_checksum(const byte *packet, int packetLength) {
    if (packetLength < 7) return false;  // Minimum packet size
    
    int dataLength = packetLength - 2;  // Exclude CRC bytes
    uint16_t crc = crc16_calculate(packet, dataLength);
    
    byte crcHigh = packet[dataLength];
    byte crcLow = packet[dataLength + 1];
    
    return ((crc >> 8) == crcHigh) && ((crc & 0xFF) == crcLow);
}

// ============================================
// GLOBAL STATE
// ============================================

struct AOGProtocolState {
    uint32_t packetsSent = 0;
    uint32_t packetsReceived = 0;
    uint16_t crcErrors = 0;
    uint16_t malformedPackets = 0;
    unsigned long lastPacketTime = 0;
    
    // Outgoing buffers
    byte autosteerBuffer[AOG_MAX_PACKET_SIZE];
    byte gpsBuffer[AOG_MAX_PACKET_SIZE];
    byte imuBuffer[AOG_MAX_PACKET_SIZE];
    byte sectionBuffer[AOG_MAX_PACKET_SIZE];
    byte diagnosticBuffer[AOG_MAX_PACKET_SIZE];
    
    // Incoming buffer
    byte rxBuffer[AOG_RX_BUFFER_SIZE];
    int rxBufferIndex = 0;
};

AOGProtocolState aog_state;

// ============================================
// PROTOCOL INITIALIZATION
// ============================================

bool aog_protocol_init() {
    Serial.println("[AOG] Initializing protocol handler...");
    
    memset(&aog_state, 0, sizeof(aog_state));
    
    Serial.println("[AOG] Packet structures initialized:");
    Serial.print("[AOG]   Autosteer: ");
    Serial.print(sizeof(AutosteerPacketToAOG));
    Serial.println(" bytes");
    
    Serial.print("[AOG]   GPS: ");
    Serial.print(sizeof(GPSPacketToAOG));
    Serial.println(" bytes");
    
    Serial.print("[AOG]   IMU: ");
    Serial.print(sizeof(IMUPacketToAOG));
    Serial.println(" bytes");
    
    Serial.print("[AOG]   Sections: ");
    Serial.print(sizeof(SectionPacketToAOG));
    Serial.println(" bytes");
    
    Serial.print("[AOG]   Diagnostic: ");
    Serial.print(sizeof(DiagnosticPacketToAOG));
    Serial.println(" bytes");
    
    return true;
}

// ============================================
// BUILD AUTOSTEER PACKET
// ============================================

int aog_build_autosteer_packet(float steerAngle, float headingError, uint8_t pwm, bool direction) {
    AutosteerPacketToAOG *pkt = (AutosteerPacketToAOG *)aog_state.autosteerBuffer;
    
    pkt->steerAngle = (int16_t)(steerAngle * 100.0f);       // Convert to 0.01 degree units
    pkt->headingError = (int16_t)(headingError * 100.0f);   // Convert to 0.01 degree units
    pkt->pwmValue = pwm;
    pkt->motorDirection = direction ? 1 : 0;
    pkt->payloadLength = sizeof(AutosteerPacketToAOG) - 6;  // Exclude headers and length
    
    int totalLength = sizeof(AutosteerPacketToAOG);
    crc16_set_checksum(aog_state.autosteerBuffer, totalLength - 2);
    
    return totalLength;
}

// ============================================
// BUILD GPS PACKET
// ============================================

int aog_build_gps_packet(double latitude, double longitude, float speed, float heading,
                         uint8_t fixQuality, uint8_t numSats, float hdop) {
    GPSPacketToAOG *pkt = (GPSPacketToAOG *)aog_state.gpsBuffer;
    
    pkt->latitude = latitude;
    pkt->longitude = longitude;
    pkt->speed = speed;
    pkt->heading = heading;
    pkt->gpsQuality = fixQuality;
    pkt->numSatellites = numSats;
    pkt->hdop = (uint16_t)(hdop * 100.0f);  // Convert to 0.01 precision
    pkt->payloadLength = sizeof(GPSPacketToAOG) - 6;  // Exclude headers and length
    
    int totalLength = sizeof(GPSPacketToAOG);
    crc16_set_checksum(aog_state.gpsBuffer, totalLength - 2);
    
    return totalLength;
}

// ============================================
// BUILD IMU PACKET
// ============================================

int aog_build_imu_packet(float heading, float roll, float pitch, uint8_t calibStatus) {
    IMUPacketToAOG *pkt = (IMUPacketToAOG *)aog_state.imuBuffer;
    
    pkt->heading = heading;
    pkt->roll = roll;
    pkt->pitch = pitch;
    pkt->calibrationStatus = calibStatus;
    pkt->payloadLength = sizeof(IMUPacketToAOG) - 6;  // Exclude headers and length
    
    int totalLength = sizeof(IMUPacketToAOG);
    crc16_set_checksum(aog_state.imuBuffer, totalLength - 2);
    
    return totalLength;
}

// ============================================
// BUILD SECTION PACKET
// ============================================

int aog_build_section_packet(uint16_t sectionStates, uint8_t mainSwitch,
                              uint16_t pressureLeft, uint16_t pressureRight) {
    SectionPacketToAOG *pkt = (SectionPacketToAOG *)aog_state.sectionBuffer;
    
    pkt->sectionStates = sectionStates;
    pkt->mainSwitchState = mainSwitch;
    pkt->pressureLeft = pressureLeft;
    pkt->pressureRight = pressureRight;
    pkt->payloadLength = sizeof(SectionPacketToAOG) - 6;  // Exclude headers and length
    
    int totalLength = sizeof(SectionPacketToAOG);
    crc16_set_checksum(aog_state.sectionBuffer, totalLength - 2);
    
    return totalLength;
}

// ============================================
// BUILD DIAGNOSTIC PACKET
// ============================================

int aog_build_diagnostic_packet(uint32_t uptime, uint16_t freeRam, uint8_t status) {
    DiagnosticPacketToAOG *pkt = (DiagnosticPacketToAOG *)aog_state.diagnosticBuffer;
    
    pkt->uptime = uptime;
    pkt->freeRam = freeRam;
    pkt->systemStatus = status;
    pkt->errorCount = 0;  // Would be filled with real error counts
    pkt->packetsSent = (aog_state.packetsSent & 0xFFFF);
    pkt->packetsReceived = (aog_state.packetsReceived & 0xFFFF);
    pkt->crcErrors = aog_state.crcErrors;
    pkt->payloadLength = sizeof(DiagnosticPacketToAOG) - 6;  // Exclude headers and length
    
    int totalLength = sizeof(DiagnosticPacketToAOG);
    crc16_set_checksum(aog_state.diagnosticBuffer, totalLength - 2);
    
    return totalLength;
}

// ============================================
// SEND PACKETS
// ============================================

void aog_send_autosteer_data(float steerAngle, float headingError, uint8_t pwm, bool direction) {
    int pktLen = aog_build_autosteer_packet(steerAngle, headingError, pwm, direction);
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[AOG] Sending autosteer packet (");
        Serial.print(pktLen);
        Serial.println(" bytes)...");
    }
    
    // Would send via Serial/WiFi/Ethernet here
    // Serial.write(aog_state.autosteerBuffer, pktLen);
    
    aog_state.packetsSent++;
    aog_state.lastPacketTime = millis();
}

void aog_send_gps_data(double latitude, double longitude, float speed, float heading,
                       uint8_t fixQuality, uint8_t numSats, float hdop) {
    int pktLen = aog_build_gps_packet(latitude, longitude, speed, heading, fixQuality, numSats, hdop);
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[AOG] Sending GPS packet (");
        Serial.print(pktLen);
        Serial.println(" bytes)...");
    }
    
    // Would send via Serial/WiFi/Ethernet here
    
    aog_state.packetsSent++;
    aog_state.lastPacketTime = millis();
}

void aog_send_imu_data(float heading, float roll, float pitch, uint8_t calibStatus) {
    int pktLen = aog_build_imu_packet(heading, roll, pitch, calibStatus);
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[AOG] Sending IMU packet (");
        Serial.print(pktLen);
        Serial.println(" bytes)...");
    }
    
    // Would send via Serial/WiFi/Ethernet here
    
    aog_state.packetsSent++;
    aog_state.lastPacketTime = millis();
}

void aog_send_section_data(uint16_t sectionStates, uint8_t mainSwitch) {
    int pktLen = aog_build_section_packet(sectionStates, mainSwitch, 0, 0);
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[AOG] Sending section packet (");
        Serial.print(pktLen);
        Serial.println(" bytes)...");
    }
    
    // Would send via Serial/WiFi/Ethernet here
    
    aog_state.packetsSent++;
    aog_state.lastPacketTime = millis();
}

void aog_send_diagnostic_data(uint32_t uptime, uint16_t freeRam, uint8_t status) {
    int pktLen = aog_build_diagnostic_packet(uptime, freeRam, status);
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[AOG] Sending diagnostic packet (");
        Serial.print(pktLen);
        Serial.println(" bytes)...");
    }
    
    // Would send via Serial/WiFi/Ethernet here
    
    aog_state.packetsSent++;
    aog_state.lastPacketTime = millis();
}

// ============================================
// PARSE INCOMING COMMANDS FROM AOG
// ============================================

bool aog_parse_incoming_data(byte data) {
    aog_state.rxBuffer[aog_state.rxBufferIndex++] = data;
    
    if (aog_state.rxBufferIndex >= AOG_RX_BUFFER_SIZE) {
        aog_state.rxBufferIndex = 0;
    }
    
    // Look for complete packet (frame headers + data + CRC)
    if (aog_state.rxBufferIndex >= 7) {
        // Check for frame headers
        if (aog_state.rxBuffer[0] == FRAME_HEADER_1 && 
            aog_state.rxBuffer[1] == FRAME_HEADER_2) {
            
            byte dataID = aog_state.rxBuffer[3];
            byte length = aog_state.rxBuffer[4];
            int totalLen = length + 6;  // Headers + length + CRC
            
            if (aog_state.rxBufferIndex >= totalLen) {
                // Verify CRC
                if (!crc16_verify_checksum(aog_state.rxBuffer, totalLen)) {
                    aog_state.crcErrors++;
                    if (DEBUG_LEVEL >= 2) {
                        Serial.println("[AOG] CRC error in incoming packet!");
                    }
                } else {
                    aog_state.packetsReceived++;
                    
                    if (DEBUG_LEVEL >= 3) {
                        Serial.print("[AOG] Received command (ID: 0x");
                        Serial.print(dataID, HEX);
                        Serial.println(")");
                    }
                }
                
                // Reset buffer
                aog_state.rxBufferIndex = 0;
                return true;
            }
        }
    }
    
    return false;
}

// ============================================
// STATISTICS AND DIAGNOSTICS
// ============================================

void aog_print_packet_stats() {
    Serial.println("\n[AOG] Protocol Statistics:");
    Serial.println("======================================");
    
    Serial.print("Packets Sent: ");
    Serial.println(aog_state.packetsSent);
    
    Serial.print("Packets Received: ");
    Serial.println(aog_state.packetsReceived);
    
    Serial.print("CRC Errors: ");
    Serial.println(aog_state.crcErrors);
    
    Serial.print("Malformed Packets: ");
    Serial.println(aog_state.malformedPackets);
    
    Serial.print("Last Packet Time: ");
    Serial.print((millis() - aog_state.lastPacketTime) / 1000.0f);
    Serial.println(" s ago");
    
    if (aog_state.packetsSent > 0) {
        float errorRate = (float)aog_state.crcErrors / (aog_state.packetsSent + aog_state.packetsReceived) * 100.0f;
        Serial.print("Error Rate: ");
        Serial.print(errorRate, 2);
        Serial.println("%");
    }
    
    Serial.println("======================================\n");
}

#endif // AOG_PROTOCOL_H
