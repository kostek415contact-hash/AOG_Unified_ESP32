#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// KONFIGURACJA GŁÓWNA - AOG UNIFIED ESP32-S3
// ============================================

// Wersja oprogramowania
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

// ============================================
// SERIAL - UART0 (USB)
// ============================================

#define SERIAL_BAUD 115200

// ============================================
// ŁĄCZNOŚĆ
// ============================================

// DataTransVia: 0=USB, 10=Ethernet, 20=WiFi
#define DATA_TRANS_PRIORITY_ETHERNET 1  // Ethernet jako priorytet
#define DATA_TRANS_ETHERNET 10
#define DATA_TRANS_WIFI 20
#define DATA_TRANS_USB 0

// Timeout łączności (ms)
#define ETHERNET_CONNECT_TIMEOUT 5000
#define WIFI_CONNECT_TIMEOUT 10000
#define ETHERNET_FALLBACK_TO_WIFI_TIME 10000  // Chwilka zanim fallback

// ============================================
// SIEĆ ETHERNET
// ============================================

#define ETHERNET_ENABLED 1
#define ETHERNET_DYNAMIC_IP 1  // DHCP

#if ETHERNET_DYNAMIC_IP
    // DHCP będzie automatycznie przydzielać IP
#else
    #define ETHERNET_IP_1 192
    #define ETHERNET_IP_2 168
    #define ETHERNET_IP_3 1
    #define ETHERNET_IP_4 10   // x.x.x.10
    
    #define ETHERNET_GATEWAY_1 192
    #define ETHERNET_GATEWAY_2 168
    #define ETHERNET_GATEWAY_3 1
    #define ETHERNET_GATEWAY_4 1
    
    #define ETHERNET_SUBNET_1 255
    #define ETHERNET_SUBNET_2 255
    #define ETHERNET_SUBNET_3 255
    #define ETHERNET_SUBNET_4 0
#endif

// ============================================
// SIEĆ WiFi
// ============================================

#define WIFI_ENABLED 1
#define WIFI_SSID "YourSSID"
#define WIFI_PASSWORD "YourPassword"

// Backupowe sieci WiFi
#define WIFI_NETWORKS_COUNT 1
const char* WiFi_SSIDs[] = {"YourSSID"};
const char* WiFi_Passwords[] = {"YourPassword"};

// DHCP dla WiFi
#define WIFI_DYNAMIC_IP 1

#if !WIFI_DYNAMIC_IP
    #define WIFI_IP_1 192
    #define WIFI_IP_2 168
    #define WIFI_IP_3 1
    #define WIFI_IP_4 11   // x.x.x.11
#endif

// Access Point (fallback jeśli brak WiFi)
#define WIFI_AP_SSID "AOG_Unified"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_IP_1 192
#define WIFI_AP_IP_2 168
#define WIFI_AP_IP_3 1
#define WIFI_AP_IP_4 1

// ============================================
// AGOPENGPS PROTOCOL
// ============================================

#define AOG_PORT 9999          // UDP port
#define AOG_NTRIP_PORT 2101    // NTRIP port

// ============================================
// GPS/GNSS
// ============================================

#define GPS_ENABLED 1
#define GPS_TYPE_UBLOX 1       // 1=U-Blox F9P, 0=inny
#define GPS_UART 1             // UART1
#define GPS_BAUD 460800        // LG290P default
#define GPS_RX 5               // GPIO 5
#define GPS_TX 4               // GPIO 4
#define GPS_NTRIP_ENABLED 1    // RTK corrections

// ============================================
// IMU - DUAL BNO085
// ============================================

#define IMU_ENABLED 1
#define IMU_COUNT 2            // DUAL IMU!

// IMU #1 - Główny (Heading + Roll z GPS)
#define IMU1_TYPE_BNO085 1
#define IMU1_BNO085_ADDR 0x4A  // Default BNO085
#define IMU1_FUSION_MODE 9     // Full 9-DOF
#define IMU1_ENABLED 1

// IMU #2 - Czujnik skrętu osi
#define IMU2_TYPE_BNO085 1
#define IMU2_BNO085_ADDR 0x4B  // Alternate address (ADDR pin high)
#define IMU2_FUSION_MODE 9     // Full 9-DOF
#define IMU2_ENABLED 1

// ============================================
// I2C
// ============================================

#define I2C_SDA_PIN 7          // I2C SDA
#define I2C_SCL_PIN 8          // I2C SCL
#define I2C_FREQ 400000        // 400kHz

// ============================================
// STEROWANIE SEKCJAMI
// ============================================

#define SECTIONS_ENABLED 1
#define SECTIONS_COUNT 16      // 16 sekcji
#define GPIO_EXPANDER_MCP23017_ADDR 0x20  // MCP23017

// ============================================
// AUTOSTEER
// ============================================

#define AUTOSTEER_ENABLED 1
#define MOTOR_PWM_PIN 2        // PWM do Cytron
#define MOTOR_DIR_PIN 3        // Kierunek
#define MOTOR_PWM_FREQ 500     // Hz
#define MOTOR_PWM_RESOLUTION 8 // 0-255
#define MOTOR_MIN_PWM 50       // Min prędkość
#define MOTOR_MAX_PWM 255      // Max prędkość

// Slow drive (małe korektury)
#define MOTOR_SLOW_DRIVE_DEGREES 2.0
#define MOTOR_HIGH_PWM 200
#define MOTOR_LOW_PWM 50

// PID
#define PID_P 25.0
#define PID_I 0.5
#define PID_D 150.0

// ============================================
// CZUJNIKI PRĄDU
// ============================================

#define CURRENT_SENSOR_ENABLED 1
#define CURRENT_SENSOR_PIN 1   // GPIO 1 ADC
#define CURRENT_SENSOR_RANGE 30.0  // ±30A
#define CURRENT_SENSOR_OFFSET 3.3 / 2  // Połowa napięcia
#define CURRENT_OVERLOAD_THRESHOLD 25.0  // A - wyłącz przy 25A

// ============================================
// WEBINTERFACE
// ============================================

#define WEBSERVER_ENABLED 1
#define WEBSERVER_PORT 80
#define WEBSERVER_UPDATE_INTERVAL 1000  // ms

// ============================================
// DEBUGOWANIE
// ============================================

#define DEBUG_SERIAL 1
#define DEBUG_LEVEL 2  // 0=off, 1=errors, 2=info, 3=verbose
#define DEBUG_GPS 0
#define DEBUG_IMU 1    // Debugowanie obu IMU
#define DEBUG_SECTIONS 0
#define DEBUG_AUTOSTEER 0

// ============================================
// EEPROM
// ============================================

#define EEPROM_SIZE 4096
#define EEPROM_CLEAR_ON_BOOT 0  // Ustaw na 1 do reset ustawień
#define EEPROM_VERSION 1        // Zmień jeśli zmieni się struktura

// ============================================
// TASKING (FreeRTOS)
// ============================================

#define TASK_PRIORITY_HIGH 2
#define TASK_PRIORITY_NORMAL 1
#define TASK_PRIORITY_LOW 0

#define TASK_STACK_SIZE_LARGE 5000
#define TASK_STACK_SIZE_NORMAL 3000
#define TASK_STACK_SIZE_SMALL 2000

#endif // CONFIG_H
