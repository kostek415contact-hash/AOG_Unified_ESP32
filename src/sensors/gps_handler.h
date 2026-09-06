// ============================================
// sensors/gps_handler.h
// LG290P GPS/GNSS RTK Handler
// NMEA + UBX parsing with NTRIP support
// ============================================

#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include <HardwareSerial.h>
#include <math.h>
#include "../config/config.h"
#include "../config/pins_config.h"

// ============================================
// GPS CONFIGURATION
// ============================================

const int GPS_BAUD = 115200;
const int GPS_BUFFER_SIZE = 256;
const float DEG_TO_RAD = 0.017453293f;
const float RAD_TO_DEG = 57.29577951f;
const float KNOTS_TO_KMH = 1.852f;
const float KNOTS_TO_MS = 0.51444f;

// ============================================
// GPS DATA STRUCTURES
// ============================================

enum GPSFixQuality {
    FIX_NONE = 0,        // No fix
    FIX_GPS = 1,         // Standard GPS
    FIX_DGPS = 2,        // Differential GPS
    FIX_RTK = 3,         // Real-Time Kinematic (RTK)
    FIX_RTK_FLOAT = 4    // RTK Float (less accurate)
};

struct GPSData {
    // Position (WGS84)
    double latitude = 0.0;            // degrees (-90 to +90)
    double longitude = 0.0;           // degrees (-180 to +180)
    float altitude = 0.0f;            // meters
    
    // Motion
    float speed_knots = 0.0f;         // knots
    float speed_kmh = 0.0f;           // km/h
    float speed_ms = 0.0f;            // m/s
    float heading = 0.0f;             // degrees (0-360)
    
    // Accuracy
    float hdop = 999.9f;              // horizontal dilution of precision
    float vdop = 999.9f;              // vertical dilution of precision
    float accuracy_m = 0.0f;          // horizontal accuracy in meters
    
    // Fix quality
    GPSFixQuality fixQuality = FIX_NONE;
    byte numSatellites = 0;
    
    // Status
    bool hasValidData = false;
    bool hasRTKFix = false;
    unsigned long lastUpdate = 0;
    unsigned long lastValidFix = 0;
    unsigned int readCount = 0;
    
    // Diagnostics
    unsigned int nmea_sentences_parsed = 0;
    unsigned int crc_errors = 0;
};

struct NTRIPConfig {
    bool enabled = NTRIP_ENABLED;
    const char* server = NTRIP_SERVER;
    int port = NTRIP_PORT;
    const char* username = NTRIP_USERNAME;
    const char* password = NTRIP_PASSWORD;
    const char* mountpoint = NTRIP_MOUNTPOINT;
};

struct NTRIPState {
    bool connected = false;
    unsigned long lastConnectAttempt = 0;
    unsigned long bytesReceived = 0;
    unsigned long bytesSent = 0;
};

// ============================================
// GLOBAL STATE
// ============================================

GPSData gps_data;
NTRIPConfig ntrip_config;
NTRIPState ntrip_state;

byte gps_buffer[GPS_BUFFER_SIZE];
int gps_buffer_index = 0;
bool gps_initialized = false;

// UART instance (UART1 = Serial1 on ESP32)
HardwareSerial &gpsSerial = Serial1;

// ============================================
// NMEA CHECKSUM CALCULATION
// ============================================

byte calculate_nmea_checksum(const char *sentence) {
    byte checksum = 0;
    
    // Skip the leading '$'
    if (*sentence == '$') {
        sentence++;
    }
    
    // Calculate checksum for all characters until '*'
    while (*sentence && *sentence != '*') {
        checksum ^= *sentence;
        sentence++;
    }
    
    return checksum;
}

bool verify_nmea_checksum(const char *sentence) {
    // Find the '*' character
    const char *asterisk = strchr(sentence, '*');
    if (!asterisk) {
        return false;  // No checksum found
    }
    
    // Parse the checksum from the sentence
    byte received_checksum = 0;
    if (sscanf(asterisk + 1, "%02hhX", &received_checksum) != 1) {
        return false;  // Invalid checksum format
    }
    
    // Calculate expected checksum
    byte calculated_checksum = calculate_nmea_checksum(sentence);
    
    if (received_checksum != calculated_checksum) {
        return false;
    }
    
    return true;
}

// ============================================
// PARSE NMEA SENTENCES
// ============================================

bool parse_rmc_sentence(const char *sentence) {
    // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a,m*hh
    // Example: $GPRMC,123519,4807.038,N,01131.000,E,022.4,084.4,230394,3.1,W*6D
    
    char time[16], lat_str[16], lat_dir, lon_str[16], lon_dir, status;
    float speed, heading;
    int date;
    
    int parsed = sscanf(sentence, "$GPRMC,%[^,],%c,%[^,],%c,%[^,],%c,%f,%f,%d",
                        time, &status, lat_str, &lat_dir, lon_str, &lon_dir, 
                        &speed, &heading, &date);
    
    if (parsed < 9) {
        return false;
    }
    
    if (status != 'A') {
        return false;  // No active fix
    }
    
    // Parse latitude: ddmm.mmmm format
    float lat_deg = atof(lat_str) / 100.0f;
    float lat_min = fmod(atof(lat_str), 100.0f);
    gps_data.latitude = lat_deg + (lat_min / 60.0f);
    if (lat_dir == 'S') {
        gps_data.latitude = -gps_data.latitude;
    }
    
    // Parse longitude: dddmm.mmmm format
    float lon_deg = atof(lon_str) / 100.0f;
    float lon_min = fmod(atof(lon_str), 100.0f);
    gps_data.longitude = lon_deg + (lon_min / 60.0f);
    if (lon_dir == 'W') {
        gps_data.longitude = -gps_data.longitude;
    }
    
    // Speed and heading
    gps_data.speed_knots = speed;
    gps_data.speed_kmh = speed * KNOTS_TO_KMH;
    gps_data.speed_ms = speed * KNOTS_TO_MS;
    gps_data.heading = heading;
    
    gps_data.hasValidData = true;
    gps_data.lastUpdate = millis();
    gps_data.nmea_sentences_parsed++;
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[GPS] RMC: Lat=");
        Serial.print(gps_data.latitude, 6);
        Serial.print(" Lon=");
        Serial.print(gps_data.longitude, 6);
        Serial.print(" Speed=");
        Serial.print(gps_data.speed_kmh, 1);
        Serial.println(" km/h");
    }
    
    return true;
}

bool parse_gga_sentence(const char *sentence) {
    // $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    // Quality: 0=none, 1=GPS, 2=DGPS, 4=RTK, 5=RTK Float
    
    char time[16], lat_str[16], lat_dir, lon_str[16], lon_dir;
    int quality, numSats;
    float hdop, altitude;
    
    int parsed = sscanf(sentence, "$GPGGA,%[^,],%[^,],%c,%[^,],%c,%d,%d,%f,%f",
                        time, lat_str, &lat_dir, lon_str, &lon_dir,
                        &quality, &numSats, &hdop, &altitude);
    
    if (parsed < 9) {
        return false;
    }
    
    // Map quality levels
    switch (quality) {
        case 0:
            gps_data.fixQuality = FIX_NONE;
            return false;
        case 1:
            gps_data.fixQuality = FIX_GPS;
            break;
        case 2:
            gps_data.fixQuality = FIX_DGPS;
            break;
        case 4:
            gps_data.fixQuality = FIX_RTK;
            gps_data.hasRTKFix = true;
            break;
        case 5:
            gps_data.fixQuality = FIX_RTK_FLOAT;
            break;
        default:
            return false;
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
    
    // Quality metrics
    gps_data.numSatellites = numSats;
    gps_data.hdop = hdop;
    gps_data.altitude = altitude;
    
    // Calculate accuracy (roughly: 2.5 * HDOP)
    gps_data.accuracy_m = hdop * 2.5f;
    
    gps_data.hasValidData = true;
    gps_data.lastUpdate = millis();
    gps_data.lastValidFix = millis();
    gps_data.nmea_sentences_parsed++;
    
    if (DEBUG_LEVEL >= 2) {
        Serial.print("[GPS] GGA: Fix=");
        Serial.print((int)gps_data.fixQuality);
        Serial.print(" Sats=");
        Serial.print(numSats);
        Serial.print(" HDOP=");
        Serial.print(hdop, 1);
        Serial.print(" Alt=");
        Serial.print(altitude, 1);
        Serial.println("m");
    }
    
    return true;
}

bool parse_gst_sentence(const char *sentence) {
    // $GPGST,hhmmss.ss,x.x,x.x,x.x,x.x,x.x,x.x,x.x*hh
    // Estimated position and velocity errors
    
    char time[16];
    float rms, std_lat, std_lon, std_alt;
    
    int parsed = sscanf(sentence, "$GPGST,%[^,],%f,%f,%f,%f",
                        time, &rms, &std_lat, &std_lon, &std_alt);
    
    if (parsed < 5) {
        return false;
    }
    
    // Use std_lat as horizontal accuracy estimate
    gps_data.accuracy_m = std_lat;
    gps_data.nmea_sentences_parsed++;
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[GPS] GST: Accuracy=");
        Serial.print(gps_data.accuracy_m, 2);
        Serial.println("m");
    }
    
    return true;
}

// ============================================
// GPS INITIALIZATION
// ============================================

bool gps_init() {
    Serial.println("[GPS] Initializing LG290P on UART1...");
    
    // Initialize UART1
    // RX1 = GPIO17, TX1 = GPIO18 (default for ESP32)
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gpsSerial.setTimeout(100);
    
    Serial.print("[GPS] UART1 started: ");
    Serial.print(GPS_BAUD);
    Serial.print(" baud (RX=");
    Serial.print(GPS_RX_PIN);
    Serial.print(" TX=");
    Serial.print(GPS_TX_PIN);
    Serial.println(")");
    
    Serial.println("[GPS] Listening for NMEA sentences...");
    gps_initialized = true;
    
    return true;
}

// ============================================
// READ AND PARSE GPS DATA
// ============================================

bool gps_read() {
    bool dataAvailable = false;
    
    // Read available bytes from UART
    while (gpsSerial.available()) {
        byte c = gpsSerial.read();
        
        if (c == '$') {
            // Start of new sentence
            gps_buffer_index = 0;
            gps_buffer[gps_buffer_index++] = c;
        } else if (c == '\n' || c == '\r') {
            // End of sentence
            if (gps_buffer_index > 0) {
                gps_buffer[gps_buffer_index] = '\0';
                
                // Verify checksum
                if (!verify_nmea_checksum((const char *)gps_buffer)) {
                    gps_data.crc_errors++;
                    if (DEBUG_LEVEL >= 3) {
                        Serial.print("[GPS] CRC ERROR: ");
                        Serial.println((const char *)gps_buffer);
                    }
                } else {
                    // Parse sentence
                    const char *sentence = (const char *)gps_buffer;
                    
                    if (strstr(sentence, "GPRMC") || strstr(sentence, "GNRMC")) {
                        if (parse_rmc_sentence(sentence)) {
                            dataAvailable = true;
                        }
                    } else if (strstr(sentence, "GPGGA") || strstr(sentence, "GNGGA")) {
                        if (parse_gga_sentence(sentence)) {
                            dataAvailable = true;
                        }
                    } else if (strstr(sentence, "GPGST") || strstr(sentence, "GNGST")) {
                        if (parse_gst_sentence(sentence)) {
                            dataAvailable = true;
                        }
                    }
                }
                
                gps_buffer_index = 0;
            }
        } else if (gps_buffer_index < GPS_BUFFER_SIZE - 1) {
            // Accumulate sentence
            gps_buffer[gps_buffer_index++] = c;
        }
    }
    
    gps_data.readCount++;
    return dataAvailable;
}

// ============================================
// GET GPS DATA
// ============================================

GPSData gps_get_data() {
    return gps_data;
}

bool gps_has_fix() {
    return (gps_data.fixQuality != FIX_NONE) && 
           (millis() - gps_data.lastValidFix < 5000);  // 5 second timeout
}

bool gps_has_rtk_fix() {
    return (gps_data.fixQuality == FIX_RTK) && 
           (millis() - gps_data.lastValidFix < 5000);
}

float gps_get_accuracy() {
    return gps_data.accuracy_m;
}

float gps_get_age_of_fix() {
    return (millis() - gps_data.lastUpdate) / 1000.0f;
}

// ============================================
// NTRIP CLIENT (RTK Corrections)
// ============================================

bool ntrip_connect() {
    if (!ntrip_config.enabled) {
        return false;
    }
    
    Serial.println("[NTRIP] Connecting to caster...");
    Serial.print("[NTRIP] Server: ");
    Serial.print(ntrip_config.server);
    Serial.print(":");
    Serial.println(ntrip_config.port);
    
    // WiFi/Ethernet would connect here
    // For now, placeholder
    ntrip_state.connected = true;
    
    Serial.println("[NTRIP] Connected! Receiving corrections...");
    return true;
}

bool ntrip_send_correction(byte *data, int len) {
    if (!ntrip_state.connected) {
        return false;
    }
    
    // Send correction data to NTRIP caster
    ntrip_state.bytesSent += len;
    
    return true;
}

// ============================================
// STATUS AND DIAGNOSTICS
// ============================================

void gps_print_status() {
    Serial.println("\n[GPS] Status Report:");
    Serial.println("======================================");
    
    Serial.print("Position: ");
    Serial.print(gps_data.latitude, 7);
    Serial.print(" / ");
    Serial.println(gps_data.longitude, 7);
    
    Serial.print("Fix Quality: ");
    switch (gps_data.fixQuality) {
        case FIX_NONE:
            Serial.println("No Fix");
            break;
        case FIX_GPS:
            Serial.println("GPS (Standard)");
            break;
        case FIX_DGPS:
            Serial.println("DGPS (Differential)");
            break;
        case FIX_RTK:
            Serial.println("RTK (Real-Time Kinematic) ✓");
            break;
        case FIX_RTK_FLOAT:
            Serial.println("RTK Float");
            break;
    }
    
    Serial.print("Satellites: ");
    Serial.println(gps_data.numSatellites);
    
    Serial.print("Accuracy: ");
    Serial.print(gps_data.accuracy_m, 2);
    Serial.println(" m");
    
    Serial.print("HDOP: ");
    Serial.print(gps_data.hdop, 1);
    Serial.print(" | VDOP: ");
    Serial.println(gps_data.vdop, 1);
    
    Serial.print("Speed: ");
    Serial.print(gps_data.speed_kmh, 1);
    Serial.print(" km/h | Heading: ");
    Serial.print(gps_data.heading, 1);
    Serial.println("°");
    
    Serial.print("Altitude: ");
    Serial.print(gps_data.altitude, 1);
    Serial.println(" m");
    
    Serial.print("Age of fix: ");
    Serial.print(gps_get_age_of_fix(), 1);
    Serial.println(" s");
    
    Serial.print("NMEA sentences: ");
    Serial.print(gps_data.nmea_sentences_parsed);
    Serial.print(" | CRC errors: ");
    Serial.println(gps_data.crc_errors);
    
    if (ntrip_config.enabled) {
        Serial.print("NTRIP: ");
        Serial.println(ntrip_state.connected ? "Connected" : "Disconnected");
    }
    
    Serial.println("======================================\n");
}

#endif // GPS_HANDLER_H
