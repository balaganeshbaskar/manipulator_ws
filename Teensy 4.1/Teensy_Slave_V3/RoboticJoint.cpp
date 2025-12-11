#include "RoboticJoint.h"

// ============================================================
// CONSTRUCTOR & SETUP
// ============================================================
RoboticJoint::RoboticJoint(uint8_t id, uint8_t stepPin, uint8_t dirPin, uint8_t enaPin)
    : _id(id), _stepPin(stepPin), _dirPin(dirPin), _enaPin(enaPin) 
{
    stepGen = new StepGenerator(_stepPin);
}

void RoboticJoint::begin() {
    pinMode(_dirPin, OUTPUT);
    pinMode(_enaPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);
    digitalWrite(_enaPin, LOW);
    
    last_physics_update = micros();
}

void RoboticJoint::setDirection(bool cw) {
    digitalWriteFast(_dirPin, cw ? HIGH : LOW);
    delayMicroseconds(2);
}

// ============================================================
// CORE: SENSOR UPDATE (RAW)
// ============================================================
void RoboticJoint::updateSensorData(uint16_t gbCountRaw, uint16_t motCount, int16_t motRot) {
    last_update_time = millis();
    _current_counts = (float)gbCountRaw; // Store RAW
    
    if (!_dataValid) {
        _target_counts = _current_counts; // Boot sync
    }
    _dataValid = true;
}

// ============================================================
// CORE: TARGET UPDATE (CONVERT ONCE)
// ============================================================
void RoboticJoint::updateTarget(float new_target_degrees) {
    // Convert and normalize
    float new_target_counts = new_target_degrees * COUNTS_PER_DEGREE;
    while(new_target_counts >= 4096.0f) new_target_counts -= 4096.0f;
    while(new_target_counts < 0.0f)     new_target_counts += 4096.0f;
    
    _target_counts = new_target_counts;
    
    if (state == HOLDING) {
        state = IDLE; // Wake up
        _lastAppliedVelocity = -9999.0f;
    }
}

// ============================================================
// CORE: STOP
// ============================================================
void RoboticJoint::stop() {
    stepGen->stop();
    state = HOLDING;
    _current_velocity_counts = 0.0f;
    _lastAppliedVelocity = 0.0f;
    digitalWrite(_enaPin, LOW);
}

// ============================================================
// V4 PURE SERVO LOOP (No Internal Ramp)
// ============================================================
void RoboticJoint::update() {
    // -----------------------------------------------------
    // 1. TIME STEP
    // -----------------------------------------------------
    unsigned long now = micros();
    float dt = (now - last_physics_update) / 1000000.0f;
    if (dt < 0.0005f) return; // Cap at 2kHz
    last_physics_update = now;

    // -----------------------------------------------------
    // 2. ERROR CALCULATION (IN COUNTS)
    // -----------------------------------------------------
    float error = _target_counts - _current_counts;
    if (error > 2048.0f) error -= 4096.0f;
    if (error < -2048.0f) error += 4096.0f;
    float abs_error = abs(error);

    // -----------------------------------------------------
    // 3. VIRTUAL JOINT PHYSICS (Use commanded velocity)
    // -----------------------------------------------------
    if (!_isPhysical) {
        if (state != HOLDING && state != IDLE) {
            float move_step = _current_velocity_counts * dt;
            if (move_step > abs_error) move_step = abs_error;
            if (error > 0) _current_counts += move_step;
            else           _current_counts -= move_step;
            // Wrap
            if(_current_counts >= 4096.0f) _current_counts -= 4096.0f;
            if(_current_counts < 0.0f)     _current_counts += 4096.0f;
        }
    }

    // -----------------------------------------------------
    // 4. SERVO LOGIC (Trust MoveIt's velocity)
    // -----------------------------------------------------
    
    // ARRIVAL CHECK
    if (abs_error < DEADZONE_COUNTS) {
        if (state != HOLDING) stop();
        return;
    }

    // START / DIRECTION
    if (state == HOLDING || state == IDLE) {
        state = CRUISING; // MoveIt handles ramp, we just execute

        // FIX: Initialize velocity immediately!
        _current_velocity_counts = _maxVelocityCounts; 

        // Set direction once per move
        if (_isPhysical) {
            bool cw = (MOTOR_DIRECTION_SIGN == -1) ? (error > 0) : (error < 0);
            setDirection(cw);
        }
    }
    
    // 5. EXECUTE AT COMMANDED VELOCITY
    if (_isPhysical) {
        // Use MoveIt's commanded velocity directly
        // Dynamic Direction Correction (fixes overshoot)
        bool cw = (MOTOR_DIRECTION_SIGN == -1) ? (error > 0) : (error < 0);
        setDirection(cw);
        
        // Apply speed (filter jitter)
        if (abs(_current_velocity_counts - _lastAppliedVelocity) > 2.0f) {
            stepGen->setSpeedCounts(_current_velocity_counts);
            _lastAppliedVelocity = _current_velocity_counts;
        }
    }
}

// ============================================================
// GETTERS (Convert Counts -> Degrees for ROS/Display)
// ============================================================
// float RoboticJoint::getCurrentAngle() { return _current_counts * DEGREES_PER_COUNT; }
// float RoboticJoint::getTargetAngle()  { return _target_counts * DEGREES_PER_COUNT; }
// float RoboticJoint::getError()        { return (_target_counts - _current_counts) * DEGREES_PER_COUNT; }
// float RoboticJoint::getVelocity()     { return _current_velocity_counts * DEGREES_PER_COUNT; }
const char* RoboticJoint::getStateStr() {
    switch(state) {
        case IDLE: return "IDLE";
        case CRUISING: return "CRUISE";
        case HOLDING: return "HOLD";
        default: return "?";
    }
}

// ============================================================
// PRIVATE HELPER FUNCTIONS (Required for declaration match)
// ============================================================
float RoboticJoint::countToAngle(float c) {
    // Keep this for debug printing if needed
    while(c<0)c+=4096; while(c>=4096)c-=4096; return (c/4096.0)*360.0;
}

float RoboticJoint::angleToCount(float a) {
    // Keep this for debug printing if needed
    while(a<0)a+=360; while(a>=360)a-=360; return (a/360.0)*4096.0;
}

// Note: This function was used for motor encoder calculation in V2.
// In V3/V4, we rely solely on the Gearbox Absolute Encoder for position control.
// We keep it for compatibility and potential diagnostics.
float RoboticJoint::getMotorAngleTotal(uint16_t motCount, int16_t motRot) {
    float raw = (motCount / 4096.0) * 360.0;
    return (motRot * 360.0) + raw;
}

// Legacy Print Function (Optional - for manual debugging)
void RoboticJoint::printStatus() {
    float current_angle = _current_counts * DEGREES_PER_COUNT;
    float target_angle = _target_counts * DEGREES_PER_COUNT;
    float error_deg = (_target_counts - _current_counts) * DEGREES_PER_COUNT;
    
    Serial.println(F("\nSTATUS:"));
    Serial.print(F(" Position:    ")); Serial.print(current_angle, 3); Serial.println(F("°"));
    Serial.print(F(" Target:      ")); Serial.print(target_angle, 2); Serial.println(F("°"));
    Serial.print(F(" Error:       ")); Serial.print(error_deg, 3); Serial.println(F("°"));
    Serial.print(F(" Vel (Cnts):  ")); Serial.print(_current_velocity_counts, 1); 
    Serial.print(F(" State:       ")); Serial.println(getStateStr());
    Serial.println();
}
