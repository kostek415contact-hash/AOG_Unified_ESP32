// ============================================
// control/autosteer_handler.h
// Autosteer Motor Control with PID
// PWM output, direction control, safety watchdog
// ============================================

#ifndef AUTOSTEER_HANDLER_H
#define AUTOSTEER_HANDLER_H

#include <math.h>
#include "../config/config.h"
#include "../config/pins_config.h"

// ============================================
// AUTOSTEER CONFIGURATION
// ============================================

const float PI_F = 3.14159265359f;

// Motor control
const byte MOTOR_PWM_CHANNEL = 0;      // FreeRTOS PWM channel
const int MOTOR_PWM_FREQ = 5000;       // 5 kHz PWM frequency
const byte MOTOR_PWM_BITS = 8;         // 8-bit (0-255)
const byte MOTOR_MIN_PWM = 0;
const byte MOTOR_MAX_PWM = 255;

// PID tuning (default values - can be adjusted via EEPROM)
const float PID_KP_DEFAULT = 50.0f;    // Proportional gain
const float PID_KI_DEFAULT = 10.0f;    // Integral gain
const float PID_KD_DEFAULT = 5.0f;     // Derivative gain

const float PID_INTEGRAL_MAX = 100.0f; // Anti-windup limit
const float PID_INTEGRAL_MIN = -100.0f;
const float PID_DEADZONE = 0.5f;       // Ignore errors < 0.5°
const byte PID_MIN_PWM = 20;           // Minimum motor power
const byte PID_MAX_PWM = 255;          // Maximum motor power

// Watchdog
const unsigned long AUTOSTEER_TIMEOUT = 2000;  // 2 seconds
const byte WATCHDOG_MAX = 250;         // ~25 seconds at 10 Hz

// ============================================
// AUTOSTEER DATA STRUCTURES
// ============================================

struct PIDController {
    float Kp = PID_KP_DEFAULT;          // Proportional gain
    float Ki = PID_KI_DEFAULT;          // Integral gain
    float Kd = PID_KD_DEFAULT;          // Derivative gain
    
    float setpoint = 0.0f;              // Target heading (degrees)
    float error = 0.0f;                 // Current error
    float prev_error = 0.0f;            // Previous error (for D term)
    float integral = 0.0f;              // Accumulated error (for I term)
    float derivative = 0.0f;            // Error rate (for D term)
    float output = 0.0f;                // PID output (-1.0 to +1.0)
};

struct AutosteerState {
    bool enabled = false;
    bool motorRunning = false;
    
    float current_heading = 0.0f;       // Current heading from IMU
    float current_roll = 0.0f;          // Current roll from IMU
    float target_heading = 0.0f;        // Target heading from AOG
    
    float heading_error = 0.0f;         // Heading error (-180 to +180)
    float roll_error = 0.0f;            // Roll error for compensation
    
    byte pwmValue = 0;                  // Motor PWM (0-255)
    bool motorDirection = 0;            // 0=LEFT, 1=RIGHT
    
    unsigned long lastUpdate = 0;
    unsigned long lastCommand = 0;
    byte watchdogCounter = 0;
    
    // Diagnostics
    unsigned int controlLoops = 0;
    float maxError = 0.0f;
    float avgError = 0.0f;
};

struct AutosteerConfig {
    bool enable_roll_compensation = true;
    float roll_compensation_factor = 0.3f;  // Roll error multiplier
    float steer_sensitivity = 1.0f;        // Overall sensitivity
    byte min_pwm = PID_MIN_PWM;
    byte max_pwm = PID_MAX_PWM;
};

// ============================================
// GLOBAL STATE
// ============================================

PIDController pid;
AutosteerState autosteer;
AutosteerConfig autosteer_config;

// ============================================
// HEADING ERROR CALCULATION
// ============================================

float calculate_heading_error(float current, float target) {
    float error = target - current;
    
    // Normalize to -180 to +180
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    
    return error;
}

// ============================================
// PID CONTROLLER
// ============================================

float pid_update(float error, float dt) {
    // Proportional term
    float p_term = pid.Kp * error;
    
    // Integral term (with anti-windup)
    pid.integral += error * dt;
    if (pid.integral > PID_INTEGRAL_MAX) pid.integral = PID_INTEGRAL_MAX;
    if (pid.integral < PID_INTEGRAL_MIN) pid.integral = PID_INTEGRAL_MIN;
    float i_term = pid.Ki * pid.integral;
    
    // Derivative term
    pid.derivative = (error - pid.prev_error) / dt;
    float d_term = pid.Kd * pid.derivative;
    
    // Combined output (-1.0 to +1.0)
    float output = p_term + i_term + d_term;
    
    // Clamp to -1.0 to +1.0
    if (output > 1.0f) output = 1.0f;
    if (output < -1.0f) output = -1.0f;
    
    pid.prev_error = error;
    pid.output = output;
    
    if (DEBUG_LEVEL >= 3) {
        Serial.print("[PID] E=");
        Serial.print(error, 1);
        Serial.print(" P=");
        Serial.print(p_term, 1);
        Serial.print(" I=");
        Serial.print(i_term, 1);
        Serial.print(" D=");
        Serial.print(d_term, 1);
        Serial.print(" OUT=");
        Serial.println(output, 2);
    }
    
    return output;
}

// ============================================
// AUTOSTEER INITIALIZATION
// ============================================

bool autosteer_init() {
    Serial.println("[AUTOSTEER] Initializing motor control...");
    
    // Configure motor PWM pin
    if (MOTOR_PWM_PIN != 255) {
        pinMode(MOTOR_PWM_PIN, OUTPUT);
        ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
        ledcAttachPin(MOTOR_PWM_PIN, MOTOR_PWM_CHANNEL);
        ledcWrite(MOTOR_PWM_CHANNEL, 0);  // Start at 0
        Serial.print("[AUTOSTEER] PWM pin: ");
        Serial.println(MOTOR_PWM_PIN);
    }
    
    // Configure motor direction pin
    if (MOTOR_DIR_PIN != 255) {
        pinMode(MOTOR_DIR_PIN, OUTPUT);
        digitalWrite(MOTOR_DIR_PIN, LOW);
        Serial.print("[AUTOSTEER] DIR pin: ");
        Serial.println(MOTOR_DIR_PIN);
    }
    
    // Load PID gains from config (in real app, from EEPROM)
    pid.Kp = PID_KP_DEFAULT;
    pid.Ki = PID_KI_DEFAULT;
    pid.Kd = PID_KD_DEFAULT;
    
    Serial.print("[AUTOSTEER] PID: Kp=");
    Serial.print(pid.Kp);
    Serial.print(" Ki=");
    Serial.print(pid.Ki);
    Serial.print(" Kd=");
    Serial.println(pid.Kd);
    
    autosteer.enabled = AUTOSTEER_ENABLED;
    Serial.println("[AUTOSTEER] Initialization complete");
    
    return true;
}

// ============================================
// SET TARGET HEADING
// ============================================

void autosteer_set_target_heading(float heading) {
    autosteer.target_heading = heading;
    autosteer.lastCommand = millis();
}

// ============================================
// UPDATE AUTOSTEER CONTROL
// ============================================

void autosteer_update(float current_heading, float current_roll, float dt) {
    if (!autosteer.enabled) {
        autosteer_set_pwm(0, false);
        return;
    }
    
    // Check timeout
    if (millis() - autosteer.lastCommand > AUTOSTEER_TIMEOUT) {
        Serial.println("[AUTOSTEER] WARNING: Command timeout!");
        autosteer_emergency_stop();
        return;
    }
    
    autosteer.current_heading = current_heading;
    autosteer.current_roll = current_roll;
    
    // Calculate heading error
    autosteer.heading_error = calculate_heading_error(current_heading, autosteer.target_heading);
    
    // Apply deadzone
    if (fabs(autosteer.heading_error) < PID_DEADZONE) {
        autosteer.heading_error = 0.0f;
    }
    
    // Calculate roll error
    autosteer.roll_error = current_roll;
    
    // Update PID controller
    float pid_output = pid_update(autosteer.heading_error, dt);
    
    // Apply roll compensation
    if (autosteer_config.enable_roll_compensation) {
        pid_output += (autosteer.roll_error * autosteer_config.roll_compensation_factor);
    }
    
    // Apply sensitivity
    pid_output *= autosteer_config.steer_sensitivity;
    
    // Clamp output
    if (pid_output > 1.0f) pid_output = 1.0f;
    if (pid_output < -1.0f) pid_output = -1.0f;
    
    // Convert to PWM value
    if (fabs(pid_output) < 0.01f) {
        // No steering needed
        autosteer_set_pwm(0, false);
        autosteer.motorRunning = false;
    } else {
        // Calculate PWM based on output
        byte pwm = (byte)(fabs(pid_output) * (PID_MAX_PWM - autosteer_config.min_pwm)) + autosteer_config.min_pwm;
        bool direction = (pid_output > 0.0f) ? 1 : 0;  // 1=RIGHT, 0=LEFT
        
        autosteer_set_pwm(pwm, direction);
        autosteer.motorRunning = true;
    }
    
    // Update diagnostics
    autosteer.controlLoops++;
    if (fabs(autosteer.heading_error) > autosteer.maxError) {
        autosteer.maxError = fabs(autosteer.heading_error);
    }
    
    if (DEBUG_LEVEL >= 2) {
        Serial.print("[AUTOSTEER] H:");
        Serial.print(current_heading, 1);
        Serial.print("° T:");
        Serial.print(autosteer.target_heading, 1);
        Serial.print("° E:");
        Serial.print(autosteer.heading_error, 1);
        Serial.print("° PWM:");
        Serial.print(autosteer.pwmValue);
        Serial.print(" DIR:");
        Serial.println(autosteer.motorDirection ? "R" : "L");
    }
}

// ============================================
// SET MOTOR PWM
// ============================================

void autosteer_set_pwm(byte value, bool direction) {
    // Clamp value
    if (value > MOTOR_MAX_PWM) value = MOTOR_MAX_PWM;
    if (value < MOTOR_MIN_PWM) value = (value > 0) ? MOTOR_MIN_PWM : 0;
    
    autosteer.pwmValue = value;
    autosteer.motorDirection = direction;
    
    // Set PWM output
    if (MOTOR_PWM_PIN != 255) {
        ledcWrite(MOTOR_PWM_CHANNEL, value);
    }
    
    // Set direction output
    if (MOTOR_DIR_PIN != 255) {
        digitalWrite(MOTOR_DIR_PIN, direction ? HIGH : LOW);
    }
}

// ============================================
// EMERGENCY STOP
// ============================================

void autosteer_emergency_stop() {
    Serial.println("[AUTOSTEER] EMERGENCY STOP!");
    autosteer.enabled = false;
    autosteer.motorRunning = false;
    autosteer_set_pwm(0, false);
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
}

// ============================================
// MANUAL MOTOR CONTROL
// ============================================

void autosteer_manual_control(byte pwm, bool direction) {
    if (!autosteer.enabled) return;
    
    autosteer_set_pwm(pwm, direction);
    autosteer.lastCommand = millis();
}

// ============================================
// PID TUNING
// ============================================

void autosteer_set_pid_gains(float kp, float ki, float kd) {
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    
    Serial.print("[AUTOSTEER] PID updated: Kp=");
    Serial.print(kp);
    Serial.print(" Ki=");
    Serial.print(ki);
    Serial.print(" Kd=");
    Serial.println(kd);
}

void autosteer_get_pid_gains(float &kp, float &ki, float &kd) {
    kp = pid.Kp;
    ki = pid.Ki;
    kd = pid.Kd;
}

// ============================================
// STATUS AND DIAGNOSTICS
// ============================================

float autosteer_get_heading_error() {
    return autosteer.heading_error;
}

float autosteer_get_roll_error() {
    return autosteer.roll_error;
}

byte autosteer_get_pwm() {
    return autosteer.pwmValue;
}

bool autosteer_is_running() {
    return autosteer.motorRunning && autosteer.enabled;
}

void autosteer_print_status() {
    Serial.println("\n[AUTOSTEER] Status Report:");
    Serial.println("======================================");
    
    Serial.print("Status: ");
    Serial.println(autosteer.enabled ? "ENABLED" : "DISABLED");
    
    Serial.print("Motor: ");
    Serial.println(autosteer.motorRunning ? "RUNNING" : "STOPPED");
    
    Serial.print("Current Heading: ");
    Serial.print(autosteer.current_heading, 1);
    Serial.print("° | Target: ");
    Serial.print(autosteer.target_heading, 1);
    Serial.println("°");
    
    Serial.print("Heading Error: ");
    Serial.print(autosteer.heading_error, 1);
    Serial.print("° | Roll Error: ");
    Serial.print(autosteer.roll_error, 1);
    Serial.println("°");
    
    Serial.print("PWM: ");
    Serial.print(autosteer.pwmValue);
    Serial.print(" | Direction: ");
    Serial.println(autosteer.motorDirection ? "RIGHT" : "LEFT");
    
    Serial.print("PID Gains - Kp: ");
    Serial.print(pid.Kp);
    Serial.print(" | Ki: ");
    Serial.print(pid.Ki);
    Serial.print(" | Kd: ");
    Serial.println(pid.Kd);
    
    Serial.print("Control Loops: ");
    Serial.print(autosteer.controlLoops);
    Serial.print(" | Max Error: ");
    Serial.print(autosteer.maxError, 2);
    Serial.println("°");
    
    Serial.print("Time Since Command: ");
    Serial.print((millis() - autosteer.lastCommand) / 1000.0f);
    Serial.println(" s");
    
    Serial.println("======================================\n");
}

#endif // AUTOSTEER_HANDLER_H
