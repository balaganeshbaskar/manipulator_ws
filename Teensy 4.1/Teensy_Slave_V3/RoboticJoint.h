#ifndef ROBOTIC_JOINT_H
#define ROBOTIC_JOINT_H

#include <Arduino.h>
#include "TeensyTimerTool.h"

using namespace TeensyTimerTool;

// ============================================================
// NATIVE UNIT CONSTANTS (COUNTS)
// ============================================================
#define ENCODER_RESOLUTION 4096.0f
#define GEAR_RATIO 50.0f
#define MICROSTEPS 8
#define MOTOR_STEPS_PER_REV (200.0f * MICROSTEPS)

// 1 Gearbox Rev = 50 Motor Revs = 50 * 1600 Steps = 80,000 Steps
// 1 Gearbox Rev = 4096 Counts
// Steps per Count = 80000 / 4096 = 19.53125
#define STEPS_PER_COUNT ((GEAR_RATIO * MOTOR_STEPS_PER_REV) / ENCODER_RESOLUTION)

// CONVERSION FACTORS
#define COUNTS_PER_DEGREE (ENCODER_RESOLUTION / 360.0f) // ~11.377
#define DEGREES_PER_COUNT (360.0f / ENCODER_RESOLUTION) // ~0.087

// MOTION PARAMETERS (IN COUNTS)
// Acceleration: 100 deg/s^2 * 11.37 = ~1137 counts/s^2
#define ACCEL_COUNTS 1137.0f 

// Deadzone: 0.05 deg * 11.37 = ~0.57 counts (Tight!)
#define DEADZONE_COUNTS 0.8f 

#define MOTOR_DIRECTION_SIGN -1

enum ControlState {
    IDLE, ACCELERATING, CRUISING, DECELERATING, HOLDING
};

// ============================================================
// STEP GENERATOR (Updated for Counts)
// ============================================================
class StepGenerator {
private:
    PeriodicTimer timer;
    uint8_t stepPin;
    volatile bool running;
    volatile uint32_t stepCount;

public:
    StepGenerator(uint8_t pin) : stepPin(pin), running(false), stepCount(0) {
        pinMode(stepPin, OUTPUT);
        digitalWrite(stepPin, LOW);
    }

    // Input: Velocity in COUNTS per Second
    void setSpeedCounts(float counts_per_sec) {
        if (abs(counts_per_sec) < 0.1) { stop(); return; } // Min speed

        float steps_per_sec = abs(counts_per_sec) * STEPS_PER_COUNT;
        
        // Safety Clamps (Teensy Timer Limits)
        float interval_us = 1000000.0f / steps_per_sec;
        if (interval_us < 5.0f) interval_us = 5.0f;           // Max ~200kHz
        if (interval_us > 500000.0f) { stop(); return; }      // Min ~2Hz

        if (!running) {
            running = true;
            timer.begin([this]() { this->pulseISR(); }, interval_us);
        } else {
            timer.setPeriod(interval_us);
        }
    }

    void stop() {
        running = false;
        timer.end();
        digitalWriteFast(stepPin, LOW);
    }

    uint32_t getSteps() { return stepCount; }
    void resetSteps() { stepCount = 0; }

private:
    void pulseISR() {
        if (!running) return;
        digitalWriteFast(stepPin, HIGH);
        delayMicroseconds(2);
        digitalWriteFast(stepPin, LOW);
        stepCount++;
    }
};

// ============================================================
// ROBOTIC JOINT CLASS
// ============================================================
class RoboticJoint {
public:
    RoboticJoint(uint8_t id, uint8_t stepPin, uint8_t dirPin, uint8_t enaPin);
    
    void begin();
    
    // Updates
    void updateSensorData(uint16_t gbCountRaw, uint16_t motCount, int16_t motRot);
    void updateTarget(float new_target_degrees); // Input in degrees (converted immediately)
    void update(); // The main loop
    void stop();

    // Getters (Convert back to degrees for ROS/Display)
    float getCurrentAngle() { return _current_counts * DEGREES_PER_COUNT; }
    float getTargetAngle()  { return _target_counts * DEGREES_PER_COUNT; }
    float getError()        { return (_target_counts - _current_counts) * DEGREES_PER_COUNT; }
    float getVelocity()     { return _current_velocity_counts * DEGREES_PER_COUNT; }
    
    const char* getStateStr();
    
    // State Checks
    bool isDataValid() { return _dataValid; }
    ControlState getState() { return state; }
    
    void setPhysical(bool isPhys) { _isPhysical = isPhys; }
    bool isPhysical() { return _isPhysical; }
    bool isSensorActive(unsigned long timeout_ms) {
        if (!_isPhysical) return true; 
        return (millis() - last_update_time) < timeout_ms;
    }
    
    void forceSyncVirtual() {
        if(!_isPhysical) _current_counts = _target_counts;
    }
    
    // Velocity Limit (Input in Degrees/s -> Stored in Counts/s)
    void setMaxVelocity(float deg_per_sec) {
        _maxVelocityCounts = deg_per_sec * COUNTS_PER_DEGREE;
        // Clamp for safety (e.g. max 100 deg/s)
        if (_maxVelocityCounts > (100.0f * COUNTS_PER_DEGREE)) 
            _maxVelocityCounts = 100.0f * COUNTS_PER_DEGREE;
    }

private:
    uint8_t _id, _stepPin, _dirPin, _enaPin;
    StepGenerator* stepGen;
    
    bool _isPhysical = true;
    bool _dataValid = false;
    unsigned long last_update_time = 0;
    unsigned long last_physics_update = 0;
    
    ControlState state = HOLDING;

    // NATIVE UNITS (COUNTS)
    float _current_counts = 0.0f;    // Float for smooth virtual sim
    float _target_counts = 0.0f;
    float _current_velocity_counts = 0.0f;
    float _maxVelocityCounts = 40.0f * COUNTS_PER_DEGREE; // Default 40 deg/s
    float _lastAppliedVelocity = 0.0f;

    // ADD THESE MISSING DECLARATIONS:
    void setDirection(bool cw);
    float countToAngle(float c);
    float angleToCount(float a);
    float getMotorAngleTotal(uint16_t motCount, int16_t motRot);
    void printStatus(); // If you plan to use it
};

#endif
