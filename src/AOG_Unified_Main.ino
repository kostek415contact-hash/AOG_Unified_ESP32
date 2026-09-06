// ============================================
// AOG_Unified_Main.ino
// Unified AgOpenGPS Controller for ESP32-S3
// ============================================
// Combined functionality:
// - GPS/GNSS RTK (Universal: LG290P, UBlox, Septentrio, etc.)
// - Dual IMU (BNO085 x2)
// - Autosteer control
// - Section control (16 sections)
// - Ethernet-first with WiFi fallback
// ============================================

// Version
byte VERSION_MAJOR = 1;
byte VERSION_MINOR = 1;
char VersionTXT[120] = " - Unified AOG Controller ESP32-S3 (Universal GPS)";

// ============================================
// CONFIGURATION & INCLUDES
// ============================================

#include "config/config.h"
#include "config/pins_config.h"
#include "config/gps_config.h"
#include "config/settings.h"

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
// SENSOR HANDLERS (Include order matters!)
// ============================================

#include "sensors/gps_universal.h"    // ✅ UNIVERSAL GPS - Auto-detect enabled
#include "sensors/imu_handler.h"
#include "connectivity/ethernet_handler.h"
#include "connectivity/wifi_handler.h"
#include "connectivity/network_manager.h"

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

// GPS data (synchronized with gps_universal.h)
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsSpeed = 0.0;
float gpsHeading = 0.0;
int gpsQuality = 0;  // 0=no fix, 1=GPS, 2=DGPS, 3=RTK

// IMU data
struct IMUData {
    float heading = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    bool dataReady = false;
    unsigned long lastUpdate = 0;
} imu1_data, imu2_data;

// Autosteer data
struct AutosteerData {
    float heading_error = 0.0f;
    float roll_error = 0.0f;
    byte pwmValue = 0;
    bool active = false;
    unsigned long lastCommand = 0;
} autosteer;

// Section control data
uint16_t sectionStateFromAOG = 0;

// Protocol buffers
const byte FromAOGSentenceHeader[3] = {0x80, 0x81, 0x7F};

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
void initGPIO();
void initSensors();
void task_EthernetConnect(void *pvParameters);
void task_WiFiConnect(void *pvParameters);
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
    delay(300);
    delay(300);

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
    Serial.println("[SETUP] Universal GPS with auto-detect enabled");

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

    // Start sensor tasks
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
    Serial.println("[INFO] System ready for operation");
    vTaskDelay(1000);
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
        vTaskDelay(1);
    } else {
        vTaskDelay(1);
    }
}

// ============================================
// INITIALIZATION FUNCTIONS
// ============================================

void restoreSettings() {
    Serial.println("  [INFO] Using default settings from config.h");
}

void initGPIO() {
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
        imu1.enableReport(SH2_GAME_ROTATION_VECTOR, 100);
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
        imu2.enableReport(SH2_GAME_ROTATION_VECTOR, 100);
        imu2.enableReport(SH2_ACCELEROMETER, 100);
        imu2.enableReport(SH2_GYROSCOPE, 100);
    }

    // Initialize ADS1115
    Serial.println("  [INFO] Initializing ADS1115...");
    if (!ads.begin(ADC_ADS1115_ADDR)) {
        Serial.println("    ERROR: ADS1115 not found!");
    } else {
        Serial.println("    OK: ADS1115 detected");
        ads.setGain(GAIN_TWOTHIRDS);
    }
}

// ============================================
// TASK: ETHERNET CONNECTION
// ============================================

void task_EthernetConnect(void *pvParameters) {
    Serial.println("[TASK] Ethernet connection task started");
    unsigned long lastAttempt = 0;

    while (1) {
        if (!ethConnected) {
            if (millis() - lastAttempt > 5000) {
                if (ethernet_init()) {
                    if (ethernet_start_udp()) {
                        ethConnected = true;
                        Serial.println("[ETH] ✓ Connected");
                    }
                }
                lastAttempt = millis();
            }
        } else {
            ethernet_maintain();
            ethernet_send_heartbeat();
        }
        vTaskDelay(1000);
    }
}

// ============================================
// TASK: WIFI CONNECTION
// ============================================

void task_WiFiConnect(void *pvParameters) {
    Serial.println("[TASK] WiFi connection task started");

    while (1) {
        if (!ethConnected && !wifiConnected) {
            if (!wifi_check_connection()) {
                wifi_begin_connection();
            } else {
                wifiConnected = true;
                wifi_start_udp();
                Serial.println("[WiFi] ✓ Connected");
            }
        }
        vTaskDelay(1000);
    }
}

// ============================================
// TASK: GPS READING (UNIVERSAL AUTO-DETECT)
// ============================================

void task_ReadGPS(void *pvParameters) {
    Serial.println("[TASK] GPS reading task started");

    // Initialize universal GPS handler
    if (!gps_init()) {
        Serial.println("[GPS] Initialization failed!");
    } else {
        Serial.print("[GPS] Protocol: ");
        Serial.println(gps_get_protocol_name());
        Serial.print("[GPS] Device: ");
        Serial.println(gps_get_device_name());
        Serial.print("[GPS] Baud: ");
        Serial.print(gps_state.baud_rate);
        Serial.println(gps_state.auto_detected ? " (auto-detected)" : " (manual)");
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 Hz

    while (1) {
        if (gps_read()) {
            GPSData current_gps = gps_get_data();

            gpsLatitude = current_gps.latitude;
            gpsLongitude = current_gps.longitude;
            gpsSpeed = current_gps.speed_kmh;
            gpsHeading = current_gps.heading;
            gpsQuality = (int)current_gps.fixQuality;

            if (DEBUG_LEVEL >= 2) {
                Serial.print("[GPS] Fix:");
                Serial.print((int)current_gps.fixQuality);
                Serial.print(" Lat:");
                Serial.print(current_gps.latitude, 6);
                Serial.print(" Lon:");
                Serial.print(current_gps.longitude, 6);
                Serial.print(" Sats:");
                Serial.println(current_gps.numSatellites);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================
// TASK: IMU READING (10 Hz)
// ============================================

void task_ReadIMU(void *pvParameters) {
    Serial.println("[TASK] IMU reading task started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

    while (1) {
        // Read IMU data
        sh2_SensorValue_t event;

        if (imu1.getSensorEvent(&event)) {
            if (event.sensorId == SH2_GAME_ROTATION_VECTOR) {
                float qw = event.un.gameRotationVector.real;
                float qx = event.un.gameRotationVector.i;
                float qy = event.un.gameRotationVector.j;
                float qz = event.un.gameRotationVector.k;

                // Convert quaternion to Euler angles
                float sinr_cosp = 2 * (qw * qx + qy * qz);
                float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
                imu1_data.roll = atan2(sinr_cosp, cosr_cosp) * 57.2958f;

                float sinp = sqrt(1 + 2 * (qw * qy - qz * qx));
                float cosp = sqrt(1 - 2 * (qw * qy - qz * qx));
                imu1_data.pitch = 2 * atan2(sinp, cosp) * 57.2958f - 90;

                float siny_cosp = 2 * (qw * qz + qx * qy);
                float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
                imu1_data.heading = atan2(siny_cosp, cosy_cosp) * 57.2958f;
                if (imu1_data.heading < 0) imu1_data.heading += 360;

                imu1_data.dataReady = true;
                imu1_data.lastUpdate = millis();
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================
// TASK: AUTOSTEER CONTROL
// ============================================

void task_Autosteer(void *pvParameters) {
    Serial.println("[TASK] Autosteer task started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

    while (1) {
        // Autosteer control logic here
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================
// TASK: SECTION CONTROL
// ============================================

void task_SectionControl(void *pvParameters) {
    Serial.println("[TASK] Section control task started");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

    while (1) {
        // Section control logic here
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================
// TASK: DATA FROM AOG
// ============================================

void task_ReadDataFromAOG(void *pvParameters) {
    Serial.println("[TASK] Data from AOG task started");

    unsigned long lastDataTime = millis();
    const unsigned long DATA_TIMEOUT = 2000;

    while (1) {
        if (ethConnected) {
            int packetSize = ethUDP.parsePacket();
            if (packetSize > 0) {
                byte buffer[256];
                int len = ethUDP.read(buffer, sizeof(buffer));

                if (buffer[0] == 0x80 && buffer[1] == 0x81) {
                    lastDataTime = millis();
                }
            }
        }

        vTaskDelay(10);
    }
}

// ============================================
// TASK: WEB SERVER
// ============================================

void task_WebServer(void *pvParameters) {
    Serial.println("[TASK] Web server task started");

    while (1) {
        vTaskDelay(100);
    }
}
