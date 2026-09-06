#ifndef SETTINGS_H
#define SETTINGS_H

// ============================================
// DOMYŚLNE USTAWIENIA
// ============================================

// Struktura zawierająca wszystkie ustawienia
struct Settings {
    // WiFi
    uint8_t wifiEnabled;
    char wifiSSID[32];
    char wifiPassword[32];
    uint8_t wifiDynamicIP;
    uint8_t wifiAPMode;
    
    // Ethernet
    uint8_t ethernetEnabled;
    uint8_t ethernetDynamicIP;
    uint32_t ethernetIP;
    uint32_t ethernetGateway;
    uint32_t ethernetSubnet;
    
    // GPS
    uint8_t gpsEnabled;
    uint32_t gpsBaud;
    uint8_t gpsUart;
    uint8_t gpsNtripEnabled;
    
    // IMU #1 (Główny)
    uint8_t imu1Enabled;
    uint8_t imu1I2CAddr;
    uint8_t imu1FusionMode;
    float imu1CalibrationX;
    float imu1CalibrationY;
    float imu1CalibrationZ;
    
    // IMU #2 (Czujnik skrętu osi)
    uint8_t imu2Enabled;
    uint8_t imu2I2CAddr;
    uint8_t imu2FusionMode;
    float imu2CalibrationX;
    float imu2CalibrationY;
    float imu2CalibrationZ;
    
    // Autosteer
    uint8_t autosteerEnabled;
    float pidP;
    float pidI;
    float pidD;
    uint16_t motorPWMFreq;
    uint8_t motorMinPWM;
    uint8_t motorMaxPWM;
    float motorSlowDriveDegrees;
    
    // Sekcje
    uint8_t sectionsEnabled;
    uint8_t sectionsCount;
    
    // Czujnik prądu
    uint8_t currentSensorEnabled;
    float currentOverloadThreshold;
    
    // Debugowanie
    uint8_t debugLevel;
    uint8_t debugGPS;
    uint8_t debugIMU;
    
} __attribute__((packed));

// Instancja globalna
extern Settings Set;

// Wartości domyślne
const Settings SETTINGS_DEFAULT = {
    // WiFi
    .wifiEnabled = WIFI_ENABLED,
    .wifiSSID = WIFI_SSID,
    .wifiPassword = WIFI_PASSWORD,
    .wifiDynamicIP = WIFI_DYNAMIC_IP,
    .wifiAPMode = 0,
    
    // Ethernet
    .ethernetEnabled = ETHERNET_ENABLED,
    .ethernetDynamicIP = ETHERNET_DYNAMIC_IP,
    .ethernetIP = (ETHERNET_IP_1 << 24) | (ETHERNET_IP_2 << 16) | (ETHERNET_IP_3 << 8) | ETHERNET_IP_4,
    .ethernetGateway = (ETHERNET_GATEWAY_1 << 24) | (ETHERNET_GATEWAY_2 << 16) | (ETHERNET_GATEWAY_3 << 8) | ETHERNET_GATEWAY_4,
    .ethernetSubnet = (ETHERNET_SUBNET_1 << 24) | (ETHERNET_SUBNET_2 << 16) | (ETHERNET_SUBNET_3 << 8) | ETHERNET_SUBNET_4,
    
    // GPS
    .gpsEnabled = GPS_ENABLED,
    .gpsBaud = GPS_BAUD,
    .gpsUart = GPS_UART_NUM,
    .gpsNtripEnabled = GPS_NTRIP_ENABLED,
    
    // IMU #1
    .imu1Enabled = IMU1_ENABLED,
    .imu1I2CAddr = IMU1_BNO085_ADDR,
    .imu1FusionMode = IMU1_FUSION_MODE,
    .imu1CalibrationX = 0.0f,
    .imu1CalibrationY = 0.0f,
    .imu1CalibrationZ = 0.0f,
    
    // IMU #2
    .imu2Enabled = IMU2_ENABLED,
    .imu2I2CAddr = IMU2_BNO085_ADDR,
    .imu2FusionMode = IMU2_FUSION_MODE,
    .imu2CalibrationX = 0.0f,
    .imu2CalibrationY = 0.0f,
    .imu2CalibrationZ = 0.0f,
    
    // Autosteer
    .autosteerEnabled = AUTOSTEER_ENABLED,
    .pidP = PID_P,
    .pidI = PID_I,
    .pidD = PID_D,
    .motorPWMFreq = MOTOR_PWM_FREQ,
    .motorMinPWM = MOTOR_MIN_PWM,
    .motorMaxPWM = MOTOR_MAX_PWM,
    .motorSlowDriveDegrees = MOTOR_SLOW_DRIVE_DEGREES,
    
    // Sekcje
    .sectionsEnabled = SECTIONS_ENABLED,
    .sectionsCount = SECTIONS_COUNT,
    
    // Czujnik prądu
    .currentSensorEnabled = CURRENT_SENSOR_ENABLED,
    .currentOverloadThreshold = CURRENT_OVERLOAD_THRESHOLD,
    
    // Debugowanie
    .debugLevel = DEBUG_LEVEL,
    .debugGPS = DEBUG_GPS,
    .debugIMU = DEBUG_IMU,
};

#endif // SETTINGS_H
