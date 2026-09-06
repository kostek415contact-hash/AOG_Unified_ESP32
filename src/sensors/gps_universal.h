// ============================================
// sensors/gps_universal.h
// Universal GNSS Handler
// Supports: NMEA (any GPS), UBX (UBlox), Auto-detect
// Tested: LG290P, UBlox, Septentrio, generic NMEA
// ============================================

#ifndef GPS_UNIVERSAL_H
#define GPS_UNIVERSAL_H

#include <HardwareSerial.h>
#include <math.h>
#include "../config/gps_config.h"
#include "../config/config.h"
#include "../config/pins_config.h"

// ============================================
// PROTOCOL TYPES
// ============================================

enum GPSProtocol {
    PROTO_AUTO = 0,
    PROTO_NMEA = 1,
    PROTO_UBX = 2
};

enum GPSDevice {
    DEVICE_AUTO = 0,
    DEVICE_LG290P = 1,
    DEVICE_UBLOX = 2,
    DEVICE_SEPTENTRIO = 3,
    DEVICE_TRIMBLE = 4,
    DEVICE_GENERIC = 5
};

// ============================================
// FIX QUALITY
// ============================================

enum GPSFixQuality {
    FIX_NONE = 0,        // No fix
    FIX_GPS = 1,         // Standard GPS
    FIX_DGPS = 2,        // Differential GPS
    FIX_RTK = 3,         // Real-Time Kinematic (RTK)
    FIX_RTK_FLOAT = 4    // RTK Float (less accurate)
};

// ============================================
// GPS DATA STRUCTURES
// ============================================

struct GPSData {
    // Position (WGS84)
    double latitude = 0.0;            // degrees
    double longitude = 0.0;           // degrees
    float altitude = 0.0f;            // meters
    
    // Motion
    float speed_knots = 0.0f;         // knots
    float speed_kmh = 0.0f;           // km/h
    float speed_ms = 0.0f;            // m/s
    float heading = 0.0f;             // degrees (0-360)
    
    // Accuracy
    float hdop = 999.9f;              // horizontal dilution
    float vdop = 999.9f;              // vertical dilution
    float accuracy_m = 0.0f;          // horizontal accuracy (m)
    
    // Fix quality
    GPSFixQuality fixQuality = FIX_NONE;
    byte numSatellites = 0;
    bool hasValidData = false;
    bool hasRTKFix = false;
    
    // Metadata
    unsigned long lastUpdate = 0;
    unsigned long lastValidFix = 0;
    unsigned int readCount = 0;
    unsigned int nmea_count = 0;
    unsigned int ubx_count = 0;
    unsigned int crc_errors = 0;
};

struct GPSState {
    GPSProtocol protocol = PROTO_AUTO;
    GPSDevice device = DEVICE_AUTO;
    int uart_port = GPS_UART_PORT;
    int baud_rate = GPS_BAUD_PRIMARY;
    bool initialized = false;
    bool auto_detected = false;
    char device_info[64];
};

// ============================================
// GLOBAL STATE
// ============================================

GPSData gps_data;
GPSState gps_state;

byte gps_buffer[GPS_NMEA_BUFFER_SIZE];
int gps_buffer_index = 0;

HardwareSerial *gps_serial = NULL;

// ============================================
// UART SELECTION
// ============================================

void gps_select_uart(int uart_num) {
    switch (uart_num) {
        case 0:
            gps_serial = &Serial;
            gps_state.uart_port = 0;
            break;
        case 1:
            gps_serial = &Serial1;
            gps_state.uart_port = 1;
            break;
        case 2:
            gps_serial = &Serial2;
            gps_state.uart_port = 2;
            break;
        default:
            Serial.println("[GPS] ERROR: Invalid UART port!");
            return;
    }
}

// ============================================
// NMEA CHECKSUM
// ============================================

byte calculate_nmea_checksum(const char *sentence) {
    byte checksum = 0;
    if (*sentence == '$') sentence++;
    while (*sentence && *sentence != '*') {
        checksum ^= *sentence++;
    }
    return checksum;
}

bool verify_nmea_checksum(const char *sentence) {
    const char *asterisk = strchr(sentence, '*');
    if (!asterisk) return false;
    
    byte received_checksum = 0;
    if (sscanf(asterisk + 1, "%02hhX", &received_checksum) != 1) {
        return false;
    }
    
    byte calculated_checksum = calculate_nmea_checksum(sentence);
    return (received_checksum == calculated_checksum);
}

// ============================================
// NMEA PARSING
// ============================================

bool parse_nmea_rmc(const char *sentence) {
    char time[16], lat_str[16], lat_dir, lon_str[16], lon_dir, status;
    float speed, heading;
    int date;
    
    int parsed = sscanf(sentence, "$GPRMC,%[^,],%c,%[^,],%c,%[^,],%c,%f,%f,%d",
                        time, &status, lat_str, &lat_dir, lon_str, &lon_dir,
                        &speed, &heading, &date);
    
    // Also try GNRMC (GLONASS)
    if (parsed < 9) {
        parsed = sscanf(sentence, "$GNRMC,%[^,],%c,%[^,],%c,%[^,],%c,%f,%f,%d",
                        time, &status, lat_str, &lat_dir, lon_str, &lon_dir,
                        &speed, &heading, &date);
    }
    
    if (parsed < 9 || status != 'A') {
        return false;
    }
    
    // Parse latitude: ddmm.mmmm
    float lat_deg = atof(lat_str) / 100.0f;
    float lat_min = fmod(atof(lat_str), 100.0f);
    gps_data.latitude = lat_deg + (lat_min / 60.0f);
    if (lat_dir == 'S') gps_data.latitude = -gps_data.latitude;
    
    // Parse longitude: dddmm.mmmm
    float lon_deg = atof(lon_str) / 100.0f;
    float lon_min = fmod(atof(lon_str), 100.0f);
    gps_data.longitude = lon_deg + (lon_min / 60.0f);
    if (lon_dir == 'W') gps_data.longitude = -gps_data.longitude;
    
    gps_data.speed_knots = speed;
    gps_data.speed_kmh = speed * 1.852f;
    gps_data.speed_ms = speed * 0.51444f;
    gps_data.heading = heading;
    
    gps_data.hasValidData = true;
    gps_data.lastUpdate = millis();
    gps_data.nmea_count++;
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[GPS-RMC] Lat=");
        Serial.print(gps_data.latitude, 6);
        Serial.print(" Lon=");
        Serial.print(gps_data.longitude, 6);
        Serial.println();
    }
    
    return true;
}

bool parse_nmea_gga(const char *sentence) {
    char time[16], lat_str[16], lat_dir, lon_str[16], lon_dir;
    int quality, numSats;
    float hdop, altitude;
    
    int parsed = sscanf(sentence, "$GPGGA,%[^,],%[^,],%c,%[^,],%c,%d,%d,%f,%f",
                        time, lat_str, &lat_dir, lon_str, &lon_dir,
                        &quality, &numSats, &hdop, &altitude);
    
    // Also try GNGGA (multi-GNSS)
    if (parsed < 9) {
        parsed = sscanf(sentence, "$GNGGA,%[^,],%[^,],%c,%[^,],%c,%d,%d,%f,%f",
                        time, lat_str, &lat_dir, lon_str, &lon_dir,
                        &quality, &numSats, &hdop, &altitude);
    }
    
    if (parsed < 9) return false;
    
    // Map quality levels
    switch (quality) {
        case 0: gps_data.fixQuality = FIX_NONE; return false;
        case 1: gps_data.fixQuality = FIX_GPS; break;
        case 2: gps_data.fixQuality = FIX_DGPS; break;
        case 4: gps_data.fixQuality = FIX_RTK; gps_data.hasRTKFix = true; break;
        case 5: gps_data.fixQuality = FIX_RTK_FLOAT; break;
        default: return false;
    }
    
    // Parse position
    float lat_deg = atof(lat_str) / 100.0f;
    float lat_min = fmod(atof(lat_str), 100.0f);
    gps_data.latitude = lat_deg + (lat_min / 60.0f);
    if (lat_dir == 'S') gps_data.latitude = -gps_data.latitude;
    
    float lon_deg = atof(lon_str) / 100.0f;
    float lon_min = fmod(atof(lon_str), 100.0f);
    gps_data.longitude = lon_deg + (lon_min / 60.0f);
    if (lon_dir == 'W') gps_data.longitude = -gps_data.longitude;
    
    gps_data.numSatellites = numSats;
    gps_data.hdop = hdop;
    gps_data.altitude = altitude;
    gps_data.accuracy_m = hdop * 2.5f;
    
    gps_data.hasValidData = true;
    gps_data.lastUpdate = millis();
    gps_data.lastValidFix = millis();
    gps_data.nmea_count++;
    
    if (DEBUG_LEVEL >= 2) {
        Serial.print("[GPS-GGA] Fix=");
        Serial.print((int)gps_data.fixQuality);
        Serial.print(" Sats=");
        Serial.print(numSats);
        Serial.print(" HDOP=");
        Serial.println(hdop, 1);
    }
    
    return true;
}

bool parse_nmea_gst(const char *sentence) {
    char time[16];
    float rms, std_lat, std_lon, std_alt;
    
    int parsed = sscanf(sentence, "$GPGST,%[^,],%f,%f,%f,%f",
                        time, &rms, &std_lat, &std_lon, &std_alt);
    
    // Also try GNGST
    if (parsed < 5) {
        parsed = sscanf(sentence, "$GNGST,%[^,],%f,%f,%f,%f",
                        time, &rms, &std_lat, &std_lon, &std_alt);
    }
    
    if (parsed < 5) return false;
    
    gps_data.accuracy_m = std_lat;
    gps_data.nmea_count++;
    
    return true;
}

// ============================================
// PROTOCOL AUTO-DETECTION
// ============================================

bool gps_auto_detect_protocol() {
    Serial.print("[GPS] Auto-detecting protocol on UART");
    Serial.print(gps_state.uart_port);
    Serial.println("...");
    
    int baud_rates[] = {GPS_BAUD_PRIMARY, GPS_BAUD_SECONDARY, GPS_BAUD_TERTIARY, GPS_BAUD_FALLBACK};
    
    for (int b = 0; b < 4; b++) {
        int baud = baud_rates[b];
        Serial.print("[GPS] Trying ");
        Serial.print(baud);
        Serial.print(" baud...");
        
        gps_serial->begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        gps_serial->setTimeout(100);
        
        unsigned long start = millis();
        bool found = false;
        
        while (millis() - start < GPS_AUTO_DETECT_TIMEOUT) {
            if (gps_serial->available()) {
                int c = gps_serial->peek();
                
                // Check for NMEA start
                if (c == '$') {
                    found = true;
                    gps_state.protocol = PROTO_NMEA;
                    Serial.println(" \u2713 NMEA detected!");
                    gps_state.baud_rate = baud;
                    gps_state.auto_detected = true;
                    return true;
                }
                
                // Check for UBX start (sync chars)
                if (c == 0xB5) {
                    byte c2 = gps_serial->read();
                    if (c2 == 0x62) {
                        found = true;
                        gps_state.protocol = PROTO_UBX;
                        Serial.println(" \u2713 UBX detected!");
                        gps_state.baud_rate = baud;
                        gps_state.auto_detected = true;
                        return true;
                    }
                }
                
                gps_serial->read();  // Consume byte
            }
            
            delay(10);
        }
    }
    
    Serial.println(" \u2717 No protocol detected!");
    return false;
}

// ============================================
// GPS INITIALIZATION
// ============================================

bool gps_init() {
    Serial.println("[GPS] Initializing Universal GNSS Handler...");
    
    // Select UART
    gps_select_uart(GPS_UART_PORT);
    
    // Auto-detect if enabled
    if (GPS_AUTO_DETECT) {
        if (!gps_auto_detect_protocol()) {
            Serial.println("[GPS] WARNING: Auto-detection failed, using manual config");
            gps_state.protocol = GPS_PROTOCOL_FORCE;
            gps_state.baud_rate = GPS_BAUD_MANUAL;
            gps_serial->begin(gps_state.baud_rate, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        }
    } else {
        gps_state.protocol = GPS_PROTOCOL_FORCE;
        gps_state.baud_rate = GPS_BAUD_MANUAL;
        gps_serial->begin(gps_state.baud_rate, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    }
    
    gps_serial->setTimeout(100);
    gps_state.initialized = true;
    
    Serial.print("[GPS] Protocol: ");
    if (gps_state.protocol == PROTO_NMEA) Serial.println("NMEA");
    else if (gps_state.protocol == PROTO_UBX) Serial.println("UBX Binary");
    else Serial.println("AUTO");
    
    Serial.print("[GPS] Baud Rate: ");
    Serial.println(gps_state.baud_rate);
    
    return true;
}

// ============================================
// READ AND PARSE GPS DATA
// ============================================

bool gps_read() {
    bool dataAvailable = false;
    
    while (gps_serial->available()) {
        byte c = gps_serial->read();
        
        if (c == '$') {
            gps_buffer_index = 0;
            gps_buffer[gps_buffer_index++] = c;
        } else if ((c == '\n' || c == '\r') && gps_buffer_index > 0) {
            gps_buffer[gps_buffer_index] = '\0';
            
            if (!verify_nmea_checksum((const char *)gps_buffer)) {
                gps_data.crc_errors++;
            } else {
                const char *sentence = (const char *)gps_buffer;
                
                if (strstr(sentence, "RMC")) {
                    if (parse_nmea_rmc(sentence)) dataAvailable = true;
                } else if (strstr(sentence, "GGA")) {
                    if (parse_nmea_gga(sentence)) dataAvailable = true;
                } else if (strstr(sentence, "GST")) {
                    if (parse_nmea_gst(sentence)) dataAvailable = true;
                }
            }
            
            gps_buffer_index = 0;
        } else if (gps_buffer_index < GPS_NMEA_BUFFER_SIZE - 1) {
            gps_buffer[gps_buffer_index++] = c;
        }
    }
    
    gps_data.readCount++;
    return dataAvailable;
}

// ============================================
// GET DATA
// ============================================

GPSData gps_get_data() {
    return gps_data;
}

bool gps_has_fix() {
    return (gps_data.fixQuality != FIX_NONE) && 
           (millis() - gps_data.lastValidFix < GPS_FIX_TIMEOUT);
}

bool gps_has_rtk_fix() {
    return (gps_data.fixQuality == FIX_RTK) && 
           (millis() - gps_data.lastValidFix < GPS_RTK_TIMEOUT);
}

float gps_get_accuracy() {
    return gps_data.accuracy_m;
}

float gps_get_age_of_fix() {
    return (millis() - gps_data.lastUpdate) / 1000.0f;
}

const char* gps_get_protocol_name() {
    if (gps_state.protocol == PROTO_NMEA) return "NMEA";
    if (gps_state.protocol == PROTO_UBX) return "UBX";
    return "AUTO";
}

const char* gps_get_device_name() {
    if (gps_state.auto_detected) return gps_state.device_info;
    return "Unknown";
}

// ============================================
// DIAGNOSTICS
// ============================================

void gps_print_status() {
    Serial.println("\n[GPS] Status Report:");
    Serial.println("======================================");
    
    Serial.print("Position: ");
    Serial.print(gps_data.latitude, 7);
    Serial.print(" / ");
    Serial.println(gps_data.longitude, 7);
    
    Serial.print("Fix: ");
    switch (gps_data.fixQuality) {
        case FIX_NONE: Serial.println("No Fix"); break;
        case FIX_GPS: Serial.println("GPS"); break;
        case FIX_DGPS: Serial.println("DGPS"); break;
        case FIX_RTK: Serial.println("RTK \u2713"); break;
        case FIX_RTK_FLOAT: Serial.println("RTK Float"); break;
    }
    
    Serial.print("Satellites: ");
    Serial.print(gps_data.numSatellites);
    Serial.print(" | HDOP: ");
    Serial.print(gps_data.hdop, 1);
    Serial.print(" | Accuracy: ");
    Serial.print(gps_data.accuracy_m, 2);
    Serial.println(" m");
    
    Serial.print("Speed: ");
    Serial.print(gps_data.speed_kmh, 1);
    Serial.print(" km/h | Heading: ");
    Serial.print(gps_data.heading, 1);
    Serial.println("°");
    
    Serial.print("Protocol: ");
    Serial.print(gps_get_protocol_name());
    Serial.print(" @ ");
    Serial.print(gps_state.baud_rate);
    Serial.print(" baud");
    Serial.print(gps_state.auto_detected ? " (auto-detected)" : "");
    Serial.println();
    
    Serial.print("Parsed: ");
    Serial.print(gps_data.nmea_count);
    Serial.print(" NMEA | CRC errors: ");
    Serial.println(gps_data.crc_errors);
    
    Serial.println("======================================\n");
}

#endif // GPS_UNIVERSAL_H
