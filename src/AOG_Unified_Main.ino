// ============================================
// AOG_Unified_Main.ino
// Unified AgOpenGPS Controller for ESP32-S3
// ============================================
// Combined functionality:
// - GPS/GNSS RTK (LG290P)
// - Dual IMU (BNO085 x2)
// - Autosteer control
// - Section control (16 sections)
// - Ethernet-first with WiFi fallback
// ============================================

// Version
byte VERSION_MAJOR = 1;
byte VERSION_MINOR = 0;
char VersionTXT[120] = " - Unified AOG Controller ESP32-S3";

// Include all configurations and headers
#include "config/config.h"
#include "config/pins_config.h"
#include "config/settings.h"

// ============================================
// SYSTEM STATE VARIABLES
// ============================================

// Connection status
bool ethConnected = false;
bool wifiConnected = false;
IPAddress ipDestination;
byte activeDataTransport = 0;  // 0=USB, 10=Ethernet, 20=WiFi

// Timing
unsigned long lastLoopTime = 0;
const unsigned long LOOP_TIME_MS = 100;  // 10 Hz main loop

// Data buffers
byte incommingBytes[500];
unsigned int incommingDataLength = 0;
byte incommingBytesArrayNr = 0;
byte incommingBytesArrayNrToParse = 0;

// GPS data
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsSpeed = 0.0;
float gpsHeading = 0.0;
float gpsRoll = 0.0;
int gpsQuality = 0;  // 0=no fix, 1=GPS, 2=DGPS, 3=RTK

// IMU data
struct IMUData {
    float heading;   // degrees 0-360
    float roll;      // degrees -180 to +180
    float pitch;     // degrees
    bool dataReady;
    unsigned long lastUpdate;
} imu1Data, imu2Data;  // IMU#1 main, IMU#2 steering column

// Autosteer data
struct AutosteerData {
    float heading_error;  // degrees
    float roll_error;     // degrees
    float pidOutput;      // PWM value
    bool active;
} autosteerData;

// Section control data
byte sectionState[2] = {0, 0};  // 16 sections as bits
byte sectionStateFromAOG[2] = {0, 0};

// Protocol buffers
const byte FromAOGSentenceHeader[3] = {0x80, 0x81, 0x7F};
byte dataToAOG[64];  // Buffer for data to send to AgOpenGPS

// ============================================
// TASK HANDLES (FreeRTOS)
// ============================================

TaskHandle_t taskHandle_Eth_connect = NULL;
TaskHandle_t taskHandle_WiFi_connect = NULL;
TaskHandle_t taskHandle_DataFromAOG = NULL;
TaskHandle_t taskHandle_IMU_read = NULL;
TaskHandle_t taskHandle_GPS_read = NULL;
TaskHandle_t taskHandle_WebServer = NULL;
TaskHandle_t taskHandle_Autosteer = NULL;
TaskHandle_t taskHandle_Sections = NULL;

// ============================================
// LIBRARIES
// ============================================

#include "EEPROM.h"
#include "Update.h"
#include "Wire.h"
#include <WiFi.h>
#include <AsyncUDP.h>
#include <WebServer.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_ADS1X15.h>

// ============================================
// INSTANCES
// ============================================

AsyncUDP wifiUDP;
EthernetUDP ethUDP;
WebServer webServer(80);
Adafruit_BNO08x imu1(50);
Adafruit_BNO08x imu2(50);
Adafruit_ADS1115 ads;

// ============================================
// FORWARD DECLARATIONS
// ============================================

void restoreSettings();
void initEthernet();
void initWiFi();
void initSensors();
void task_ReadIMU(void *pvParameters);
void task_ReadGPS(void *pvParameters);
void task_ReadDataFromAOG(void *pvParameters);
void task_Autosteer(void *pvParameters);
void task_SectionControl(void *pvParameters);
void task_WebServer(void *pvParameters);

// ============================================
// SETUP
// ============================================

void setup() {
    delay(300);  // Wait for power to stabilize
    delay(300);  // Wait for IO chips to be ready

    // Initialize Serial
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println();
    Serial.print("\n\n===== AOG UNIFIED ESP32-S3 V");
    Serial.print(VERSION_MAJOR);
    Serial.print(".");
    Serial.print(VERSION_MINOR);
    Serial.println(" =====");
    Serial.println(VersionTXT);

    // Restore settings from EEPROM
    Serial.println("[SETUP] Loading settings from EEPROM...");
    restoreSettings();
    delay(100);

    // Initialize GPIO pins
    Serial.println("[SETUP] Initializing GPIO pins...");
    initGPIO();
    delay(50);

    // Initialize I2C and sensors
    Serial.println("[SETUP] Initializing I2C sensors...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    delay(100);
    initSensors();
    delay(100);

    // Initialize Ethernet (primary)
    Serial.println("[SETUP] Starting Ethernet...");
    if (ETHERNET_ENABLED) {
        xTaskCreate(task_EthernetConnect, "Eth_Connect", TASK_STACK_SIZE_NORMAL, NULL, 
                    TASK_PRIORITY_HIGH, &taskHandle_Eth_connect);
        delay(500);
    }

    // Initialize WiFi (fallback)
    Serial.println("[SETUP] Starting WiFi...");
    if (WIFI_ENABLED) {
        xTaskCreate(task_WiFiConnect, "WiFi_Connect", TASK_STACK_SIZE_NORMAL, NULL, 
                    TASK_PRIORITY_HIGH, &taskHandle_WiFi_connect);
        delay(500);
    }

    // Start sensor reading tasks
    Serial.println("[SETUP] Starting sensor tasks...");
    xTaskCreate(task_ReadIMU, "Read_IMU", TASK_STACK_SIZE_NORMAL, NULL, 
                TASK_PRIORITY_NORMAL, &taskHandle_IMU_read);
    delay(100);
    
    xTaskCreate(task_ReadGPS, "Read_GPS", TASK_STACK_SIZE_LARGE, NULL, 
                TASK_PRIORITY_NORMAL, &taskHandle_GPS_read);
    delay(100);

    // Data communication task
    Serial.println("[SETUP] Starting data communication...");
    xTaskCreate(task_ReadDataFromAOG, "Data_From_AOG", TASK_STACK_SIZE_LARGE, NULL, 
                TASK_PRIORITY_HIGH, &taskHandle_DataFromAOG);
    delay(100);

    // Autosteer task
    if (AUTOSTEER_ENABLED) {
        xTaskCreate(task_Autosteer, "Autosteer", TASK_STACK_SIZE_NORMAL, NULL, 
                    TASK_PRIORITY_HIGH, &taskHandle_Autosteer);
        delay(100);
    }

    // Section control task
    if (SECTIONS_ENABLED) {
        xTaskCreate(task_SectionControl, "Sections", TASK_STACK_SIZE_NORMAL, NULL, 
                    TASK_PRIORITY_NORMAL, &taskHandle_Sections);
        delay(100);
    }

    // Web server task
    if (WEBSERVER_ENABLED) {
        xTaskCreate(task_WebServer, "WebServer", TASK_STACK_SIZE_LARGE, NULL, 
                    TASK_PRIORITY_LOW, &taskHandle_WebServer);
        delay(100);
    }

    Serial.println("[SETUP] Initialization complete!");
    Serial.println("[INFO] Waiting for sensor calibration and network connection...");
    vTaskDelay(5000);
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
    unsigned long now = millis();

    // Check timing for main loop (10 Hz)
    if (now - lastLoopTime >= LOOP_TIME_MS) {
        lastLoopTime = now;

        // Main processing happens in tasks
        // This is kept simple for stability
        vTaskDelay(1);
    } else {
        vTaskDelay(1);
    }
}

// ============================================
// PLACEHOLDER FUNCTIONS (to be implemented)
// ============================================

void restoreSettings() {
    // Load settings from EEPROM
    // For now, use defaults from config.h
    Serial.println("  [INFO] Using default settings");
}

void initGPIO() {
    // Initialize LED pins
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("  [INFO] GPIO initialized");
}

void initSensors() {
    // Initialize BNO085 #1
    Serial.print("  [INFO] Initializing BNO085 #1 (0x");
    Serial.print(IMU1_BNO085_ADDR, HEX);
    Serial.println(")...");
    if (!imu1.begin_I2C(IMU1_BNO085_ADDR)) {
        Serial.println("    ERROR: BNO085 #1 not found!");
    } else {
        Serial.println("    OK: BNO085 #1 detected");
        imu1.enableReport(SH2_GAME_ROTATION_VECTOR, 100);  // 10 Hz
        imu1.enableReport(SH2_ACCELEROMETER, 100);
        imu1.enableReport(SH2_GYROSCOPE, 100);
    }

    // Initialize BNO085 #2
    Serial.print("  [INFO] Initializing BNO085 #2 (0x");
    Serial.print(IMU2_BNO085_ADDR, HEX);
    Serial.println(")...");
    if (!imu2.begin_I2C(IMU2_BNO085_ADDR)) {
        Serial.println("    ERROR: BNO085 #2 not found!");
    } else {
        Serial.println("    OK: BNO085 #2 detected");
        imu2.enableReport(SH2_GAME_ROTATION_VECTOR, 100);  // 10 Hz
        imu2.enableReport(SH2_ACCELEROMETER, 100);
        imu2.enableReport(SH2_GYROSCOPE, 100);
    }

    // Initialize ADS1115
    Serial.println("  [INFO] Initializing ADS1115...");
    if (!ads.begin(ADC_ADS1115_ADDR)) {
        Serial.println("    ERROR: ADS1115 not found!");
    } else {
        Serial.println("    OK: ADS1115 detected");
        ads.setGain(GAIN_TWOTHIRDS);  // +/- 6.144V range
    }
}

void task_EthernetConnect(void *pvParameters) {
    Serial.println("[TASK] Ethernet connection task started");
    vTaskDelete(NULL);
}

void task_WiFiConnect(void *pvParameters) {
    Serial.println("[TASK] WiFi connection task started");
    vTaskDelete(NULL);
}

void task_ReadIMU(void *pvParameters) {
    Serial.println("[TASK] IMU reading task started");
    vTaskDelete(NULL);
}

void task_ReadGPS(void *pvParameters) {
    Serial.println("[TASK] GPS reading task started");
    vTaskDelete(NULL);
}

void task_ReadDataFromAOG(void *pvParameters) {
    Serial.println("[TASK] Data from AOG task started");
    vTaskDelete(NULL);
}

void task_Autosteer(void *pvParameters) {
    Serial.println("[TASK] Autosteer task started");
    vTaskDelete(NULL);
}

void task_SectionControl(void *pvParameters) {
    Serial.println("[TASK] Section control task started");
    vTaskDelete(NULL);
}

void task_WebServer(void *pvParameters) {
    Serial.println("[TASK] Web server task started");
    vTaskDelete(NULL);
}
