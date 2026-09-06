#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

// ============================================
// PINOUT - ESP32-S3-DEV-KIT-N16R8
// ============================================

// UWAGA: GPIO 0, 6-12 ZAREZERWOWANE dla flash!
// GPIO 33-39 tylko wejście
// Wszystkie piny I/O na 3.3V!

// ============================================
// UART - SERIAL
// ============================================

#define SERIAL_BAUD 115200
#define SERIAL_RX_PIN 44  // USB RX
#define SERIAL_TX_PIN 43  // USB TX

// ============================================
// UART1 - GPS/GNSS
// ============================================

#define GPS_UART_NUM 1
#define GPS_RX_PIN 5      // RX1
#define GPS_TX_PIN 4      // TX1
#define GPS_BAUD 460800   // LG290P

// ============================================
// I2C - CZUJNIKI (DUAL IMU!)
// ============================================

#define I2C_SDA_PIN 7     // I2C SDA
#define I2C_SCL_PIN 8     // I2C SCL
#define I2C_FREQ 400000   // 400kHz

// Adresy I2C
#define IMU1_BNO085_ADDR 0x4A         // IMU #1 - Główny
#define IMU2_BNO085_ADDR 0x4B         // IMU #2 - Czujnik skrętu osi
#define GPIO_EXPANDER_MCP23017_ADDR 0x20  // A0,A1,A2 = GND
#define ADC_ADS1115_ADDR 0x48             // ADDR = GND

// I2C int pins (opcjonalnie)
#define IMU1_INT_PIN 34   // Interrupt IMU #1 (input only)
#define IMU2_INT_PIN 35   // Interrupt IMU #2 (input only)

// ============================================
// SPI - ETHERNET W5500
// ============================================

#define SPI_MOSI_PIN 11   // GPIO 11
#define SPI_MISO_PIN 19   // GPIO 19 (ZMIANA z 12!)
#define SPI_SCK_PIN 13    // GPIO 13
#define W5500_CS_PIN 10   // Chip Select
#define W5500_RST_PIN 37  // Reset
#define W5500_INT_PIN 36  // Interrupt

// ============================================
// PWM - SILNIK KIEROWANIA
// ============================================

#define MOTOR_PWM_PIN 2       // PWM1
#define MOTOR_DIR_PIN 3       // Kierunek
#define MOTOR_PWM_FREQ 500    // Hz
#define MOTOR_PWM_CHANNEL 0   // PWM channel

// ============================================
// ANALOG INPUT - CZUJNIK PRĄDU
// ============================================

#define CURRENT_SENSOR_PIN 1  // GPIO 1 - ADC1_CH0
#define CURRENT_SENSOR_RESOLUTION 12  // 12-bit ADC

// ============================================
// DIGITAL INPUT - PRZEŁĄCZNIKI
// ============================================

#define MAIN_SWITCH_PIN 38    // Main ON/OFF
#define SECTION_AUTO_PIN 39   // Auto/Manual (BRAK pullup!)
#define PRESSURE_UP_PIN 20    // Pressure +
#define PRESSURE_DOWN_PIN 21  // Pressure -

// ============================================
// DIGITAL OUTPUT - LED
// ============================================

#define LED_BUILTIN 46    // Wbudowana LED
#define LED_WIFI_PIN 40   // Indykator WiFi (opcjonalnie)
#define LED_ETH_PIN 41    // Indykator Ethernet (opcjonalnie)
#define LED_GPS_PIN 42    // Indykator GPS (opcjonalnie)
#define LED_IMU1_PIN 15   // Indykator IMU #1
#define LED_IMU2_PIN 16   // Indykator IMU #2

// ============================================
// Button WIFI RESCAN
// ============================================

#define BUTTON_WIFI_RESCAN_PIN_CORRECTED 14  // Przycisk WiFi rescan

// ============================================
// PODSUMOWANIE WOLNYCH PINÓW
// ============================================

// Dostępne piny:
// 14, 15, 16, 17, 18
// 33, 34, 35 (input only)

// Zarezerwowane:
// 0, 6-12 (Flash SPI bus)
// 43, 44 (USB UART)
// 45, 46 (PSRAM)

#endif // PINS_CONFIG_H
