// ============================================
// sensors/imu_handler.h
// Dual BNO085 IMU Handler with Quaternion->Euler conversion
// ============================================

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Adafruit_BNO08x.h>
#include <math.h>
#include "../config/config.h"
#include "../config/pins_config.h"

// ============================================
// IMU CONFIGURATION
// ============================================

const float PI_F = 3.14159265359f;
const int IMU_REPORT_INTERVAL = 100;  // 10 Hz
const int IMU_MAX_CALIBRATION_TIME = 30000;  // 30 seconds

// ============================================
// IMU DATA STRUCTURES
// ============================================

struct IMUCalibration {
    float headingOffset = 0.0f;    // degrees
    float rollOffset = 0.0f;       // degrees
    float pitchOffset = 0.0f;      // degrees
    bool isCalibrated = false;
    byte calibrationLevel = 0;     // 0-3, 3 is fully calibrated
};

struct IMUSensorData {
    // Quaternion (raw from BNO085)
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    
    // Euler angles (processed)
    float heading = 0.0f;          // 0-360 degrees
    float roll = 0.0f;            // -180 to +180 degrees
    float pitch = 0.0f;           // -90 to +90 degrees
    
    // Metadata
    bool dataReady = false;
    bool hasNewData = false;
    unsigned long lastUpdate = 0;
    unsigned int readCount = 0;
    
    // Diagnostics
    float temperature = 0.0f;
    byte systemCalib = 0;
    byte gyroCalib = 0;
    byte accelCalib = 0;
    byte magCalib = 0;
};

// ============================================
// GLOBAL STATE
// ============================================

Adafruit_BNO08x imu1_sensor;
Adafruit_BNO08x imu2_sensor;
sh2_SensorValue_t imu1_sensorValue;
sh2_SensorValue_t imu2_sensorValue;

IMUSensorData imu1_data;
IMUSensorData imu2_data;
IMUCalibration imu1_cal;
IMUCalibration imu2_cal;

bool imu1_initialized = false;
bool imu2_initialized = false;
unsigned long imu_init_time = 0;

// ============================================
// QUATERNION TO EULER CONVERSION
// ============================================

void quaternion_to_euler(float qx, float qy, float qz, float qw, 
                         float &heading, float &roll, float &pitch) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0f) {
        pitch = copysign(PI_F / 2.0f, sinp);  // Use 90 degrees if out of range
    } else {
        pitch = asin(sinp);
    }
    
    // Yaw / Heading (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    heading = atan2(siny_cosp, cosy_cosp);
    
    // Convert from radians to degrees
    heading = heading * 180.0f / PI_F;
    roll = roll * 180.0f / PI_F;
    pitch = pitch * 180.0f / PI_F;
    
    // Normalize heading to 0-360
    if (heading < 0.0f) {
        heading += 360.0f;
    }
}

// ============================================
// IMU INITIALIZATION
// ============================================

bool imu_init() {
    Serial.println("[IMU] Initializing dual BNO085 sensors...");
    imu_init_time = millis();
    
    // Initialize BNO085 #1
    Serial.print("[IMU] Initializing BNO085 #1 (0x");
    Serial.print(IMU1_BNO085_ADDR, HEX);
    Serial.println(")...");
    
    if (!imu1_sensor.begin_I2C(IMU1_BNO085_ADDR)) {
        Serial.println("  ERROR: BNO085 #1 not found!");
        imu1_initialized = false;
    } else {
        Serial.println("  OK: BNO085 #1 detected");
        
        // Enable game rotation vector report (no magnetic field dependency)
        imu1_sensor.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL);
        imu1_sensor.enableReport(SH2_SHAKE_DETECTOR, IMU_REPORT_INTERVAL);
        imu1_initialized = true;
        imu1_data.dataReady = false;
    }
    
    delay(100);
    
    // Initialize BNO085 #2
    Serial.print("[IMU] Initializing BNO085 #2 (0x");
    Serial.print(IMU2_BNO085_ADDR, HEX);
    Serial.println(")...");
    
    if (!imu2_sensor.begin_I2C(IMU2_BNO085_ADDR)) {
        Serial.println("  ERROR: BNO085 #2 not found!");
        imu2_initialized = false;
    } else {
        Serial.println("  OK: BNO085 #2 detected");
        
        // Enable game rotation vector report
        imu2_sensor.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL);
        imu2_sensor.enableReport(SH2_SHAKE_DETECTOR, IMU_REPORT_INTERVAL);
        imu2_initialized = true;
        imu2_data.dataReady = false;
    }
    
    Serial.print("[IMU] Initialization: ");
    if (imu1_initialized || imu2_initialized) {
        Serial.println("✓ At least one BNO085 ready");
        return true;
    } else {
        Serial.println("✗ No BNO085 sensors found!");
        return false;
    }
}

// ============================================
// READ IMU DATA
// ============================================

bool imu_read() {
    bool dataAvailable = false;
    
    // Read IMU #1
    if (imu1_initialized && imu1_sensor.hasNewData()) {
        if (imu1_sensor.getSensorEvent(&imu1_sensorValue)) {
            if (imu1_sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                // Store quaternion
                imu1_data.qx = imu1_sensorValue.un.gameRotationVector.i;
                imu1_data.qy = imu1_sensorValue.un.gameRotationVector.j;
                imu1_data.qz = imu1_sensorValue.un.gameRotationVector.k;
                imu1_data.qw = imu1_sensorValue.un.gameRotationVector.real;
                
                // Convert quaternion to Euler angles
                quaternion_to_euler(imu1_data.qx, imu1_data.qy, imu1_data.qz, imu1_data.qw,
                                   imu1_data.heading, imu1_data.roll, imu1_data.pitch);
                
                // Apply calibration offsets
                imu1_data.heading -= imu1_cal.headingOffset;
                imu1_data.roll -= imu1_cal.rollOffset;
                imu1_data.pitch -= imu1_cal.pitchOffset;
                
                // Normalize heading
                if (imu1_data.heading < 0.0f) {
                    imu1_data.heading += 360.0f;
                }
                if (imu1_data.heading >= 360.0f) {
                    imu1_data.heading -= 360.0f;
                }
                
                // Clamp roll
                if (imu1_data.roll > 180.0f) {
                    imu1_data.roll -= 360.0f;
                }
                if (imu1_data.roll < -180.0f) {
                    imu1_data.roll += 360.0f;
                }
                
                imu1_data.hasNewData = true;
                imu1_data.dataReady = true;
                imu1_data.lastUpdate = millis();
                imu1_data.readCount++;
                dataAvailable = true;
                
                if (DEBUG_LEVEL >= 3) {
                    Serial.print("[IMU1] H:");
                    Serial.print(imu1_data.heading, 1);
                    Serial.print("° R:");
                    Serial.print(imu1_data.roll, 1);
                    Serial.println("°");
                }
            }
        }
    }
    
    // Read IMU #2
    if (imu2_initialized && imu2_sensor.hasNewData()) {
        if (imu2_sensor.getSensorEvent(&imu2_sensorValue)) {
            if (imu2_sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                // Store quaternion
                imu2_data.qx = imu2_sensorValue.un.gameRotationVector.i;
                imu2_data.qy = imu2_sensorValue.un.gameRotationVector.j;
                imu2_data.qz = imu2_sensorValue.un.gameRotationVector.k;
                imu2_data.qw = imu2_sensorValue.un.gameRotationVector.real;
                
                // Convert quaternion to Euler angles
                quaternion_to_euler(imu2_data.qx, imu2_data.qy, imu2_data.qz, imu2_data.qw,
                                   imu2_data.heading, imu2_data.roll, imu2_data.pitch);
                
                // Apply calibration offsets
                imu2_data.heading -= imu2_cal.headingOffset;
                imu2_data.roll -= imu2_cal.rollOffset;
                imu2_data.pitch -= imu2_cal.pitchOffset;
                
                // Normalize heading
                if (imu2_data.heading < 0.0f) {
                    imu2_data.heading += 360.0f;
                }
                if (imu2_data.heading >= 360.0f) {
                    imu2_data.heading -= 360.0f;
                }
                
                // Clamp roll
                if (imu2_data.roll > 180.0f) {
                    imu2_data.roll -= 360.0f;
                }
                if (imu2_data.roll < -180.0f) {
                    imu2_data.roll += 360.0f;
                }
                
                imu2_data.hasNewData = true;
                imu2_data.dataReady = true;
                imu2_data.lastUpdate = millis();
                imu2_data.readCount++;
                dataAvailable = true;
                
                if (DEBUG_LEVEL >= 3) {
                    Serial.print("[IMU2] H:");
                    Serial.print(imu2_data.heading, 1);
                    Serial.print("° R:");
                    Serial.print(imu2_data.roll, 1);
                    Serial.println("°");
                }
            }
        }
    }
    
    return dataAvailable;
}

// ============================================
// GET IMU DATA
// ============================================

IMUSensorData imu_get_data(int sensorId) {
    if (sensorId == 1) {
        imu1_data.hasNewData = false;
        return imu1_data;
    } else {
        imu2_data.hasNewData = false;
        return imu2_data;
    }
}

// ============================================
// GET FUSED DATA (average of both)
// ============================================

IMUSensorData imu_get_fused_data() {
    IMUSensorData fused = imu1_data;  // Start with IMU1
    
    if (imu1_initialized && imu2_initialized) {
        // Average heading (handle wrap-around)
        float h1 = imu1_data.heading;
        float h2 = imu2_data.heading;
        float dh = h2 - h1;
        if (dh > 180.0f) dh -= 360.0f;
        if (dh < -180.0f) dh += 360.0f;
        fused.heading = h1 + dh * 0.5f;
        if (fused.heading < 0.0f) fused.heading += 360.0f;
        if (fused.heading >= 360.0f) fused.heading -= 360.0f;
        
        // Average roll
        fused.roll = (imu1_data.roll + imu2_data.roll) * 0.5f;
        
        // Average pitch
        fused.pitch = (imu1_data.pitch + imu2_data.pitch) * 0.5f;
    }
    
    return fused;
}

// ============================================
// CALIBRATION
// ============================================

bool imu_calibrate() {
    Serial.println("[IMU] Starting calibration routine...");
    Serial.println("[IMU] Keep sensors level and stationary for 30 seconds");
    
    unsigned long calibStartTime = millis();
    float heading1_sum = 0.0f;
    float roll1_sum = 0.0f;
    int count1 = 0;
    
    float heading2_sum = 0.0f;
    float roll2_sum = 0.0f;
    int count2 = 0;
    
    while (millis() - calibStartTime < IMU_MAX_CALIBRATION_TIME) {
        if (imu_read()) {
            if (imu1_data.dataReady) {
                heading1_sum += imu1_data.heading;
                roll1_sum += imu1_data.roll;
                count1++;
            }
            
            if (imu2_data.dataReady) {
                heading2_sum += imu2_data.heading;
                roll2_sum += imu2_data.roll;
                count2++;
            }
        }
        
        // Progress indicator
        if ((millis() - calibStartTime) % 1000 < 50) {
            Serial.print(".");
        }
        
        delay(10);
    }
    
    Serial.println();
    
    // Calculate offsets for IMU #1
    if (count1 > 0) {
        imu1_cal.headingOffset = heading1_sum / count1;
        imu1_cal.rollOffset = roll1_sum / count1;
        imu1_cal.isCalibrated = true;
        imu1_cal.calibrationLevel = 3;
        
        Serial.print("[IMU1] Calibrated - Offset H:");
        Serial.print(imu1_cal.headingOffset, 1);
        Serial.print("° R:");
        Serial.print(imu1_cal.rollOffset, 1);
        Serial.println("°");
    }
    
    // Calculate offsets for IMU #2
    if (count2 > 0) {
        imu2_cal.headingOffset = heading2_sum / count2;
        imu2_cal.rollOffset = roll2_sum / count2;
        imu2_cal.isCalibrated = true;
        imu2_cal.calibrationLevel = 3;
        
        Serial.print("[IMU2] Calibrated - Offset H:");
        Serial.print(imu2_cal.headingOffset, 1);
        Serial.print("° R:");
        Serial.print(imu2_cal.rollOffset, 1);
        Serial.println("°");
    }
    
    return (imu1_cal.isCalibrated || imu2_cal.isCalibrated);
}

// ============================================
// STATUS FUNCTIONS
// ============================================

bool imu_is_calibrated() {
    return (imu1_cal.isCalibrated && imu2_cal.isCalibrated);
}

bool imu_has_valid_data() {
    return (imu1_data.dataReady && imu1_data.readCount > 0) ||
           (imu2_data.dataReady && imu2_data.readCount > 0);
}

void imu_print_status() {
    Serial.println("\n[IMU] Status Report:");
    Serial.println("======================================");
    
    if (imu1_initialized) {
        Serial.print("BNO085 #1 (0x");
        Serial.print(IMU1_BNO085_ADDR, HEX);
        Serial.print("): ");
        Serial.print(imu1_cal.isCalibrated ? "✓ Calibrated" : "✗ Not Calibrated");
        Serial.print(" | Reads: ");
        Serial.print(imu1_data.readCount);
        Serial.print(" | H:");
        Serial.print(imu1_data.heading, 1);
        Serial.print("° R:");
        Serial.print(imu1_data.roll, 1);
        Serial.println("°");
    }
    
    if (imu2_initialized) {
        Serial.print("BNO085 #2 (0x");
        Serial.print(IMU2_BNO085_ADDR, HEX);
        Serial.print("): ");
        Serial.print(imu2_cal.isCalibrated ? "✓ Calibrated" : "✗ Not Calibrated");
        Serial.print(" | Reads: ");
        Serial.print(imu2_data.readCount);
        Serial.print(" | H:");
        Serial.print(imu2_data.heading, 1);
        Serial.print("° R:");
        Serial.print(imu2_data.roll, 1);
        Serial.println("°");
    }
    
    IMUSensorData fused = imu_get_fused_data();
    Serial.print("Fused Data: H:");
    Serial.print(fused.heading, 1);
    Serial.print("° R:");
    Serial.print(fused.roll, 1);
    Serial.println("°");
    Serial.println("======================================\n");
}

#endif // IMU_HANDLER_H
