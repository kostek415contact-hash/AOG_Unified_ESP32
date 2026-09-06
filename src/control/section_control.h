// ============================================
// control/section_control.h
// MCP23017 I2C GPIO Expander - Section Control
// 16 sections (8x 2 GPIO ports)
// ============================================

#ifndef SECTION_CONTROL_H
#define SECTION_CONTROL_H

#include <Wire.h>
#include "../config/config.h"
#include "../config/pins_config.h"
#include "../config/thread_safety.h"

#define MCP23017_IODIRA   0x00
#define MCP23017_IODIRB   0x01
#define MCP23017_GPIOA    0x12
#define MCP23017_GPIOB    0x13

struct SectionState {
    uint16_t desiredState = 0;
    uint16_t currentState = 0;
    bool online = false;
    unsigned long lastUpdate = 0;
    unsigned int errorCount = 0;
};

static SectionState sectionState;

bool mcp23017_write_register(uint8_t reg, uint8_t value) {
    if (!lock_i2c()) return false;
    Wire.beginTransmission(GPIO_EXPANDER_MCP23017_ADDR);
    Wire.write(reg);
    Wire.write(value);
    bool ok = (Wire.endTransmission() == 0);
    unlock_i2c();
    return ok;
}

bool mcp23017_read_register(uint8_t reg, uint8_t &value) {
    if (!lock_i2c()) return false;
    Wire.beginTransmission(GPIO_EXPANDER_MCP23017_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        unlock_i2c();
        return false;
    }
    Wire.requestFrom(GPIO_EXPANDER_MCP23017_ADDR, 1);
    if (Wire.available() < 1) {
        unlock_i2c();
        return false;
    }
    value = Wire.read();
    unlock_i2c();
    return true;
}

bool section_control_init() {
    Serial.println("[SECTIONS] Initializing MCP23017 at 0x20...");
    if (!mcp23017_write_register(MCP23017_IODIRA, 0x00)) return false;
    if (!mcp23017_write_register(MCP23017_IODIRB, 0x00)) return false;
    if (!mcp23017_write_register(MCP23017_GPIOA, 0x00)) return false;
    if (!mcp23017_write_register(MCP23017_GPIOB, 0x00)) return false;
    sectionState.online = true;
    Serial.println("[SECTIONS] ✓ MCP23017 initialized");
    return true;
}

bool section_control_set_state(uint16_t state) {
    if (!sectionState.online) {
        sectionState.errorCount++;
        return false;
    }
    uint8_t portA = state & 0xFF;
    uint8_t portB = (state >> 8) & 0xFF;
    if (!mcp23017_write_register(MCP23017_GPIOA, portA)) return false;
    if (!mcp23017_write_register(MCP23017_GPIOB, portB)) return false;
    sectionState.currentState = state;
    sectionState.lastUpdate = millis();
    return true;
}

bool section_control_is_online() {
    uint8_t dummy;
    if (mcp23017_read_register(MCP23017_IODIRA, dummy)) {
        sectionState.online = true;
        return true;
    }
    sectionState.online = false;
    return false;
}

bool section_is_on(uint8_t num) {
    if (num >= 16) return false;
    return (sectionState.currentState >> num) & 1;
}

void section_set_on(uint8_t num) {
    if (num >= 16) return;
    section_control_set_state(sectionState.currentState | (1 << num));
}

void section_set_off(uint8_t num) {
    if (num >= 16) return;
    section_control_set_state(sectionState.currentState & ~(1 << num));
}

void section_toggle(uint8_t num) {
    if (num >= 16) return;
    section_control_set_state(sectionState.currentState ^ (1 << num));
}

#endif // SECTION_CONTROL_H
