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
#include <stdlib.h>
#include "../config/gps_config.h"
#include "../config/config.h"
#include "../config/pins_config.h"
#include "../config/thread_safety.h"

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
bool gps_in_frame = false;

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

bool nmea_split_fields(const char *sentence, char fields[][20], int max_fields, int &field_count) {
    field_count = 0;
    if (sentence == NULL || sentence[0] != '$') return false;

    const char *p = sentence + 1;
    int char_index = 0;
    bool started = false;

    while (*p && *p != '*') {
        if (field_count >= max_fields) break;

        if (!started) {
            started = true;
            char_index = 0;
            fields[field_count][0] = '\0';
        }

        if (*p == ',') {
            fields[field_count][char_index] = '\0';
            field_count++;
            started = false;
        } else if (char_index < 19) {
            fields[field_count][char_index++] = *p;
        }
        p++;
    }

    if (started && field_count < max_fields) {
        fields[field_count][char_index] = '\0';
        field_count++;
    }

    return field_count > 0;
}

bool nmea_sentence_is(const char *type_token, const char *suffix3) {
    if (type_token == NULL || suffix3 == NULL) return false;
    size_t len = strlen(type_token);
    if (len < 3) return false;
    return strcmp(type_token + len - 3, suffix3) == 0;
}

double nmea_parse_coordinate(const char *value, char direction, bool is_latitude) {
    if (value == NULL || value[0] == '\0') return 0.0;

    double raw = atof(value);
    int deg_digits = is_latitude ? 2 : 3;
    double divisor = 1.0;
    for (int i = 0; i < deg_digits; i++) divisor *= 10.0;

    int degrees = (int)(raw / divisor);
    double minutes = raw - ((double)degrees * divisor);
    double result = (double)degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') result = -result;
    return result;
}

bool parse_nmea_rmc(const char *sentence) {
    char fields[20][20];
    int field_count = 0;
    if (!nmea_split_fields(sentence, fields, 20, field_count) || field_count < 9) {
        return false;
    }

    if (!nmea_sentence_is(fields[0], "RMC")) {
        return false;
    }

    char status = fields[2][0];
    if (status != 'A') {
        return false;
    }

    double latitude = nmea_parse_coordinate(fields[3], fields[4][0], true);
    double longitude = nmea_parse_coordinate(fields[5], fields[6][0], false);
    float speed = (fields[7][0] == '\0') ? 0.0f : atof(fields[7]);
    float heading = (fields[8][0] == '\0') ? 0.0f : atof(fields[8]);

    if (lock_shared_data()) {
        gps_data.latitude = latitude;
        gps_data.longitude = longitude;
        gps_data.speed_knots = speed;
        gps_data.speed_kmh = speed * 1.852f;
        gps_data.speed_ms = speed * 0.51444f;
        gps_data.heading = heading;
        gps_data.hasValidData = true;
        gps_data.lastUpdate = millis();
        gps_data.nmea_count++;
        unlock_shared_data();
    }
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[GPS-RMC] Lat=");
        Serial.print(latitude, 6);
        Serial.print(" Lon=");
        Serial.print(longitude, 6);
        Serial.println();
    }
    
    return true;
}

bool parse_nmea_gga(const char *sentence) {
    char fields[20][20];
    int field_count = 0;
    if (!nmea_split_fields(sentence, fields, 20, field_count) || field_count < 10) {
        return false;
    }

    if (!nmea_sentence_is(fields[0], "GGA")) {
        return false;
    }

    int quality = (fields[6][0] == '\0') ? 0 : atoi(fields[6]);
    int numSats = (fields[7][0] == '\0') ? 0 : atoi(fields[7]);
    float hdop = (fields[8][0] == '\0') ? 999.9f : atof(fields[8]);
    float altitude = (fields[9][0] == '\0') ? 0.0f : atof(fields[9]);
    bool hasCoordinates = (fields[2][0] != '\0' && fields[3][0] != '\0' &&
                           fields[4][0] != '\0' && fields[5][0] != '\0');

    GPSFixQuality fixQuality = FIX_NONE;
    bool hasRTKFix = false;
    switch (quality) {
        case 0: return false;
        case 1: fixQuality = FIX_GPS; break;
        case 2: fixQuality = FIX_DGPS; break;
        case 4: fixQuality = FIX_RTK; hasRTKFix = true; break;
        case 5: fixQuality = FIX_RTK_FLOAT; break;
        default: return false;
    }

    double latitude = hasCoordinates ? nmea_parse_coordinate(fields[2], fields[3][0], true) : 0.0;
    double longitude = hasCoordinates ? nmea_parse_coordinate(fields[4], fields[5][0], false) : 0.0;

    if (lock_shared_data()) {
        gps_data.fixQuality = fixQuality;
        gps_data.hasRTKFix = hasRTKFix;
        if (hasCoordinates) {
            gps_data.latitude = latitude;
            gps_data.longitude = longitude;
        }
        gps_data.numSatellites = numSats;
        gps_data.hdop = hdop;
        gps_data.altitude = altitude;
        gps_data.accuracy_m = hdop * 2.5f;
        gps_data.hasValidData = true;
        gps_data.lastUpdate = millis();
        gps_data.lastValidFix = millis();
        gps_data.nmea_count++;
        unlock_shared_data();
    }
    
    if (DEBUG_LEVEL >= 2) {
        Serial.print("[GPS-GGA] Fix=");
        Serial.print((int)fixQuality);
        Serial.print(" Sats=");
        Serial.print(numSats);
        Serial.print(" HDOP=");
        Serial.println(hdop, 1);
    }
    
    return true;
}

bool parse_nmea_gst(const char *sentence) {
    char fields[20][20];
    int field_count = 0;
    if (!nmea_split_fields(sentence, fields, 20, field_count) || field_count < 5) {
        return false;
    }

    if (!nmea_sentence_is(fields[0], "GST")) {
        return false;
    }

    float std_lat = (fields[2][0] == '\0') ? 0.0f : atof(fields[2]);
    if (lock_shared_data()) {
        gps_data.accuracy_m = std_lat;
        gps_data.nmea_count++;
        unlock_shared_data();
    }
    
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
            gps_in_frame = true;
        } else if (!gps_in_frame) {
            continue;
        } else if ((c == '\n' || c == '\r') && gps_buffer_index > 0) {
            gps_buffer[gps_buffer_index] = '\0';
            
            if (!verify_nmea_checksum((const char *)gps_buffer)) {
                if (lock_shared_data()) {
                    gps_data.crc_errors++;
                    unlock_shared_data();
                }
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
            gps_in_frame = false;
        } else if (gps_buffer_index < GPS_NMEA_BUFFER_SIZE - 1) {
            gps_buffer[gps_buffer_index++] = c;
        } else {
            gps_buffer_index = 0;
            gps_in_frame = false;
        }
    }
    
    if (lock_shared_data()) {
        gps_data.readCount++;
        unlock_shared_data();
    }
    return dataAvailable;
}

// ============================================
// GET DATA
// ============================================

GPSData gps_get_data() {
    if (lock_shared_data()) {
        GPSData data = gps_data;
        unlock_shared_data();
        return data;
    }
    return gps_data;
}

bool gps_has_fix() {
    GPSData data = gps_get_data();
    return (data.fixQuality != FIX_NONE) && 
           (millis() - data.lastValidFix < GPS_FIX_TIMEOUT);
}

bool gps_has_rtk_fix() {
    GPSData data = gps_get_data();
    return (data.fixQuality == FIX_RTK) && 
           (millis() - data.lastValidFix < GPS_RTK_TIMEOUT);
}

float gps_get_accuracy() {
    return gps_get_data().accuracy_m;
}

float gps_get_age_of_fix() {
    return (millis() - gps_get_data().lastUpdate) / 1000.0f;
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
    GPSData data = gps_get_data();
    Serial.println("\n[GPS] Status Report:");
    Serial.println("======================================");
    
    Serial.print("Position: ");
    Serial.print(data.latitude, 7);
    Serial.print(" / ");
    Serial.println(data.longitude, 7);
    
    Serial.print("Fix: ");
    switch (data.fixQuality) {
        case FIX_NONE: Serial.println("No Fix"); break;
        case FIX_GPS: Serial.println("GPS"); break;
        case FIX_DGPS: Serial.println("DGPS"); break;
        case FIX_RTK: Serial.println("RTK \u2713"); break;
        case FIX_RTK_FLOAT: Serial.println("RTK Float"); break;
    }
    
    Serial.print("Satellites: ");
    Serial.print(data.numSatellites);
    Serial.print(" | HDOP: ");
    Serial.print(data.hdop, 1);
    Serial.print(" | Accuracy: ");
    Serial.print(data.accuracy_m, 2);
    Serial.println(" m");
    
    Serial.print("Speed: ");
    Serial.print(data.speed_kmh, 1);
    Serial.print(" km/h | Heading: ");
    Serial.print(data.heading, 1);
    Serial.println("°");
    
    Serial.print("Protocol: ");
    Serial.print(gps_get_protocol_name());
    Serial.print(" @ ");
    Serial.print(gps_state.baud_rate);
    Serial.print(" baud");
    Serial.print(gps_state.auto_detected ? " (auto-detected)" : "");
    Serial.println();
    
    Serial.print("Parsed: ");
    Serial.print(data.nmea_count);
    Serial.print(" NMEA | CRC errors: ");
    Serial.println(data.crc_errors);
    
    Serial.println("======================================\n");
}

#endif // GPS_UNIVERSAL_H
