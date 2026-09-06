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

// ============================================
// INCLUDES & CONFIGURATION
// ============================================

#include "config/config.h"
#include "config/pins_config.h"
#include "config/settings.h"
#include "connectivity/ethernet_handler.h"
#include "connectivity/wifi_handler.h"
#include "connectivity/network_manager.h"
#include "sensors/gps_handler.h"
#include "sensors/imu_handler.h"
#include "protocol/aog_protocol.h"
#include "control/autosteer_handler.h"
#include "control/section_control.h"
#include "webinterface/webserver.h"

// Libraries
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
// SYSTEM STATE VARIABLES
// ============================================

bool ethConnected = false;
bool wifiConnected = false;
IPAddress ipDestination;
byte activeDataTransport = 0;  // 0=USB, 10=Ethernet, 20=WiFi

unsigned long lastLoopTime = 0;
const unsigned long LOOP_TIME_MS = 100;  // 10 Hz main loop

byte incommingBytes[500];
unsigned int incommingDataLength = 0;

float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsSpeed = 0.0;
float gpsHeading = 0.0;
int gpsQuality = 0;

struct IMUData {
    float heading = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    bool dataReady = false;
    unsigned long lastUpdate = 0;
} imu1_data, imu2_data;

struct AutosteerData {
    float heading_error = 0.0f;
    float roll_error = 0.0f;
    byte pwmValue = 0;
    bool active = false;
    unsigned long lastCommand = 0;
} autosteer;

uint16_t sectionStateFromAOG = 0;

const byte FromAOGSentenceHeader[3] = {0x80, 0x81, 0x7F};

// ============================================
// TASK HANDLES (FreeRTOS)
// ============================================

#define TASK_STACK_SIZE_SMALL 2048
#define TASK_STACK_SIZE_NORMAL 4096
#define TASK_STACK_SIZE_LARGE 8192
#define TASK_PRIORITY_LOW 1
#define TASK_PRIORITY_NORMAL 2
#define TASK_PRIORITY_HIGH 3

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

EthernetUDP ethUdpServer;
AsyncUDP wifiUDP;
WebServer webServer(80);
Adafruit_BNO08x imu1(50), imu2(50);

// ============================================
// FORWARD DECLARATIONS
// ============================================

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

    // Initialize GPIO pins
    Serial.println("[SETUP] Initializing GPIO pins...");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Initialize I2C
    Serial.println("[SETUP] Initializing I2C sensors...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    delay(100);

    // Initialize Ethernet
    Serial.println("[SETUP] Starting Ethernet...");
    if (ETHERNET_ENABLED) {
        xTaskCreate(task_EthernetConnect, "Eth_Connect", TASK_STACK_SIZE_NORMAL, NULL,
                    TASK_PRIORITY_HIGH, &taskHandle_Eth_connect);
        delay(500);
    }

    // Initialize WiFi
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

    // Data communication
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

    // Section control
    if (SECTIONS_ENABLED) {
        xTaskCreate(task_SectionControl, "Sections", TASK_STACK_SIZE_NORMAL, NULL,
                    TASK_PRIORITY_NORMAL, &taskHandle_Sections);
        delay(100);
    }

    // Web server
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
    vTaskDelay(100);
}

// ============================================
// ETHERNET CONNECTION TASK
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
// WIFI CONNECTION TASK
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
// GPS READING TASK
// ============================================

void task_ReadGPS(void *pvParameters) {
    Serial.println("[TASK] GPS reading task started");
    Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    gps_init();

    while (1) {
        if (Serial1.available()) {
            gps_process_byte(Serial1.read());
        }
        vTaskDelay(10);
    }
}

// ============================================
// IMU READING TASK (10 Hz)
// ============================================

void task_ReadIMU(void *pvParameters) {
    Serial.println("[TASK] IMU reading task started");

    if (!imu_init()) {
        Serial.println("[IMU] Initialization failed!");
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 Hz

    while (1) {
        imu_read();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================
// AUTOSTEER CONTROL TASK (10 Hz)
// ============================================

void task_Autosteer(void *pvParameters) {
    Serial.println("[TASK] Autosteer task started");
    autosteer_init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 Hz
    unsigned long lastTime = millis();

    while (1) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;

        autosteer_update(imu1_data.heading, imu1_data.roll, dt);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================
// SECTION CONTROL TASK (10 Hz)
// ============================================

void task_SectionControl(void *pvParameters) {
    Serial.println("[TASK] Section control task started");

    if (!section_control_init()) {
        Serial.println("[SECTIONS] Initialization failed!");
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 Hz

    while (1) {
        section_control_set_state(sectionStateFromAOG);
        section_control_is_online();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ============================================
// WEB SERVER TASK
// ============================================

void task_WebServer(void *pvParameters) {
    Serial.println("[TASK] Web server task started");

    if (!webserver_init()) {
        Serial.println("[WEB] Initialization failed!");
    }

    while (1) {
        webserver_handle_clients();
        vTaskDelay(10);
    }
}

// ============================================
// DATA FROM AOG TASK
// ============================================

void task_ReadDataFromAOG(void *pvParameters) {
    Serial.println("[TASK] Data from AOG task started");

    unsigned long lastDataTime = millis();
    const unsigned long DATA_TIMEOUT = 2000;  // 2 seconds

    while (1) {
        // Check Ethernet
        if (ethConnected) {
            int packetSize = ethUdpServer.parsePacket();
            if (packetSize > 0) {
                byte buffer[256];
                int len = ethUdpServer.read(buffer, sizeof(buffer));

                // Parse AOG protocol
                if (buffer[0] == 0x80 && buffer[1] == 0x81) {
                    // Section Control packet (0x7B, 0xEA)
                    if (buffer[2] == 0x7B && buffer[3] == 0xEA && len >= 8) {
                        sectionStateFromAOG = (buffer[6] << 8) | buffer[5];
                    }
                    lastDataTime = millis();
                }
            }
        }

        // Watchdog - emergency stop if no data
        if (millis() - lastDataTime > DATA_TIMEOUT) {
            autosteer_emergency_stop();
        }

        vTaskDelay(10);
    }
}
