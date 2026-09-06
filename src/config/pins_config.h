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

#define SERIAL_RX_PIN 44  // USB RX
#define SERIAL_TX_PIN 43  // USB TX

// ============================================
// UART1 - GPS/GNSS
// ============================================

#define GPS_UART_NUM 1
#define GPS_RX 5      // RX1
#define GPS_TX 4      // TX1

// ============================================
// I2C - CZUJNIKI (DUAL IMU!)
// ============================================

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

#define MOTOR_PWM_FREQ 500    // Hz
#define MOTOR_PWM_CHANNEL 0   // PWM channel

// ============================================
// ANALOG INPUT - CZUJNIK PRĄDU
// ============================================

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

#define BUTTON_WIFI_RESCAN_PIN 14  // Przycisk WiFi rescan

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
