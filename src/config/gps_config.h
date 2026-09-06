// ============================================
// config/gps_config.h
// Universal GPS Configuration
// Supports: LG290P, UBlox, Septentrio, Trimble, etc.
// ============================================

#ifndef GPS_CONFIG_H
#define GPS_CONFIG_H

// ============================================
// AUTO-DETECTION SETTINGS
// ============================================

#define GPS_AUTO_DETECT    true         // Auto-detect protocol & baud
#define GPS_UART_PORT      1            // 0=Serial, 1=Serial1, 2=Serial2
#define GPS_RX_PIN         5            // RX from pins_config.h
#define GPS_TX_PIN         4            // TX from pins_config.h

// Baud rate priorities (tries in order)
#define GPS_BAUD_PRIMARY   460800       // LG290P default
#define GPS_BAUD_SECONDARY 115200       
#define GPS_BAUD_TERTIARY  38400        
#define GPS_BAUD_FALLBACK  9600         

// Auto-detect timeout
#define GPS_AUTO_DETECT_TIMEOUT 3000    // 3 seconds per baud rate

// ============================================
// MANUAL OVERRIDE (if auto-detect fails)
// ============================================

// Protocol: AUTO, NMEA, UBX
#define GPS_PROTOCOL_FORCE PROTO_AUTO

// Device: AUTO, LG290P, UBLOX, SEPTENTRIO, TRIMBLE, OTHER
#define GPS_DEVICE_FORCE DEVICE_AUTO

// Manual baud if auto-detect disabled
#define GPS_BAUD_MANUAL 460800

// ============================================
// DEVICE-SPECIFIC SETTINGS
// ============================================

// LG290P (default for AOG)
#define LG290P_BAUD 460800
#define LG290P_PROTOCOL NMEA

// UBlox (RTK-capable alternative)
#define UBLOX_BAUD 38400
#define UBLOX_PROTOCOL UBX

// Septentrio (multi-constellation GNSS)
#define SEPTENTRIO_BAUD 115200
#define SEPTENTRIO_PROTOCOL NMEA  // Supports both NMEA and SBF

// Generic NMEA receiver
#define GENERIC_NMEA_BAUD 9600
#define GENERIC_NMEA_PROTOCOL NMEA

// ============================================
// NTRIP CONFIGURATION
// ============================================

#define NTRIP_ENABLED       true
#define NTRIP_SERVER        "rtk.example.com"
#define NTRIP_PORT          2101
#define NTRIP_USERNAME      "user"
#define NTRIP_PASSWORD      "pass"
#define NTRIP_MOUNTPOINT    "BASE1"

// ============================================
// BUFFER SIZES
// ============================================

#define GPS_NMEA_BUFFER_SIZE     256    // NMEA sentence max length
#define GPS_UBX_BUFFER_SIZE      512    // UBX message buffer
#define GPS_CIRCULAR_BUFFER_SIZE 2048   // Input circular buffer

// ============================================
// PARSING OPTIONS
// ============================================

#define GPS_PARSE_RMC   true           // Recommended Minimum
#define GPS_PARSE_GGA   true           // Global Positioning System Fix Data
#define GPS_PARSE_GST   true           // Position Error
#define GPS_PARSE_VTG   true           // Track & Ground Speed
#define GPS_PARSE_GSA   true           // DOP and Active Satellites
#define GPS_PARSE_UBX   true           // UBlox binary protocol

// ============================================
// RTK CONFIGURATION
// ============================================

#define GPS_RTK_ENABLED  true
#define GPS_RTK_TIMEOUT  5000           // 5 seconds to lose RTK fix
#define GPS_FIX_TIMEOUT  10000          // 10 seconds to lose any fix

// ============================================
// DIAGNOSTICS
// ============================================

#define GPS_ENABLE_STATS true           // Track parsed sentences, errors
#define GPS_LOG_SENTENCES false         // Debug: log raw NMEA sentences
#define GPS_LOG_UBX false              // Debug: log UBX frames
#define GPS_PRINT_UNKNOWN false         // Debug: print unrecognized sentences

#endif // GPS_CONFIG_H
