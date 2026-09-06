# AOG_Unified_ESP32

**Ujednolicony system kontroli traktorów do AgOpenGPS na bazie ESP32-S3**

## 🏗️ Funkcjonalność

- ✅ **Sterowanie sekcjami** (16 sekcji) - Section Control
- ✅ **GPS/GNSS RTK** - LG290P z obsługą NTRIP
- ✅ **Automatyczne sterowanie** (Autosteer) - silnik kierowania
- ✅ **Czujnik orientacji** (IMU BNO085 #1) - heading + roll GPS
- ✅ **Czujnik skrętu osi** (IMU BNO085 #2) - roll traktora
- ✅ **Ethernet-first** - fallback na WiFi
- ✅ **Webinterface** - konfiguracja przez przeglądarkę

## 🔧 Sprzęt

### Mózg
- **ESP32-S3-DEV-KIT-N16R8** (Waveshare 28836) - 16MB Flash, dual-core 240MHz

### Komunikacja
- **W5500 Ethernet** - główne połączenie
- **WiFi wbudowany** - backup

### Czujniki
- **LG290P GNSS RTK** - quad-band L1/L2/L5/E6, cm-level positioning
- **BNO085 IMU #1** (Adres I2C: 0x4A) - heading, roll (z GPS)
- **BNO085 IMU #2** (Adres I2C: 0x4B) - roll czujnik skrętu osi

### Sterowniki
- **Cytron MD13S** - sterownik silnika kierowania DC 12V
- **ACS71240 Current Sensor** - pomiar przeciążenia (-30A do +30A, 3.3V)

### Moduły I/O
- **ADS1115** - 4-kanałowy ADC 16-bit
- **MCP23017** - ekspander GPIO I2C (sterowanie sekcjami)
- **Step-Down 5V 5A** - zasilanie

## 📁 Struktura

```
AOG_Unified_ESP32/
├── src/
│   ├── AOG_Unified_Main.ino           # Główny plik
│   ├── config/
│   │   ├── config.h                   # Główna konfiguracja
│   │   ├── pins_config.h              # Przypisanie pinów
│   │   └── settings.h                 # Ustawienia domyślne
│   ├── connectivity/
│   │   ├── ethernet_handler.h/.cpp    # Obsługa Ethernet
│   │   ├── wifi_handler.h/.cpp        # Obsługa WiFi
│   │   └── aog_protocol.h/.cpp        # Protokół AgOpenGPS
│   ├── sensors/
│   │   ├── gps_handler.h/.cpp         # LG290P GNSS
│   │   ├── imu_handler.h/.cpp         # BNO085 IMU (2x)
│   │   └── adc_handler.h/.cpp         # ADS1115
│   ├── control/
│   │   ├── section_control.h/.cpp     # Sterowanie sekcjami
│   │   ├── autosteer.h/.cpp           # Autosteer
│   │   └── motor_control.h/.cpp       # Sterownik silnika
│   ├── webinterface/
│   │   ├── webserver.h/.cpp           # Serwer WWW
│   │   └── web_pages.h                # Strony HTML/JS
│   └── utils/
│       ├── crc.h/.cpp                 # Obliczanie CRC
│       ├── logger.h/.cpp              # Logowanie
│       └── eeprom_manager.h/.cpp      # Zarządzanie EEPROM
├── include/                           # Biblioteki własne
├── docs/
│   ├── WIRING.md                      # Schemat połączeń
│   ├── SETUP.md                       # Instrukcja setup
│   ├── PINOUT_ESP32S3.md              # Przypisanie pinów
│   └── TROUBLESHOOTING.md             # Rozwiązywanie problemów
├── platformio.ini                     # Konfiguracja PlatformIO
└── .gitignore                         # Ignorowanie plików
```

## 🚀 Pierwsze kroki

1. Zainstaluj [PlatformIO](https://platformio.org/)
2. Sklonuj repozytorium
3. Edytuj `src/config/pins_config.h` - przypisz piny do Twojego sprzętu
4. Edytuj `src/config/config.h` - ustaw sieci WiFi i parametry
5. Wgraj kod na ESP32-S3
6. Połącz się z IP x.x.x.10 (domyślnie 192.168.1.10)

## 📖 Dokumentacja

- [Schemat połączeń](docs/WIRING.md)
- [Instrukcja instalacji](docs/SETUP.md)
- [Pinout ESP32-S3](docs/PINOUT_ESP32S3.md)

## 🔌 Połączenia

### Ethernet (priorytet)
- W5500 SPI: MOSI, MISO, CLK, CS
- Fallback na WiFi jeśli brak Ethernet

### GPS LG290P
- UART1: RX, TX (460800 baud)
- I2C opcjonalnie

### IMU BNO085 #1 (Heading + Roll z GPS)
- I2C SDA/SCL (Adres 0x4A)
- Montaż: na dachu/wysoko - do pomiaru orientacji wobec GPS

### IMU BNO085 #2 (Czujnik skrętu osi)
- I2C SDA/SCL (Adres 0x4B)
- Montaż: na piacie - do pomiaru roll traktora
- **WAŻNE**: Oba BNO085 na tej samej szynie I2C, różne adresy!

### MCP23017 Ekspander GPIO (I2C)
- Sterowanie 16 sekcjami przełącznikami
- Adres: 0x20

### ADS1115 ADC (I2C)
- 4 kanały ADC 16-bit
- Adres: 0x48

### Sterownik silnika
- PWM do Cytron MD13S
- DIR pin
- Czujnik prądu ACS71240

### Sekcje
- Przekaźniki przez ekspander GPIO MCP23017
- Przełączniki wejściowe

## 📋 Założenia

✅ **Ethernet jako główne połączenie** - szybsze, stabilniejsze
✅ **WiFi jako fallback** - jeśli Ethernet niedostępny
✅ **Kompatybilność z AgOpenGPS V5+**
✅ **Webinterface do konfiguracji** - bez edycji kodu
✅ **NTRIP client** - wysyłanie RTK corrections
✅ **Monitoring czujników** - temperatura, napięcie, błędy
✅ **Dual IMU BNO085** - niezależne pomiary orientacji

## 🔗 Źródła

Integacja kodów z:
- [AOG_SectionControl_ESP32](https://github.com/mtz8302/AOG_SectionControl_ESP32)
- [AOG_GPS_ESP32](https://github.com/mtz8302/AOG_GPS_ESP32)
- [AOG_Autosteer_ESP32](https://github.com/mtz8302/AOG_Autosteer_ESP32)
- [AOG_IMU_ESP32](https://github.com/mtz8302/AOG_IMU_ESP32)

## 📝 Licencja

MIT

## 👤 Autor

Integacja: kostek415contact-hash
Wersja: 1.0.0
