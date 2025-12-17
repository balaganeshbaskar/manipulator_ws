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
  // 1. SAVE RAW VALUE FOR FINE TUNING
  _motCountRaw_Global = (int)motCount;
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
  while(new_target_counts < 0.0f) new_target_counts += 4096.0f;
  
  _target_counts = new_target_counts;
  
  if (state == HOLDING && !_inTrajectoryMode) {
    state = IDLE; // Wake up only if not in trajectory mode
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
// ✅ MODIFIED: CONTINUOUS TRAJECTORY SERVO LOOP
// ============================================================
void RoboticJoint::update() {

  if (!_isPhysical) return;
  
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



  // ✅ ADD DEBUG OUTPUT (Joint 1 only)
  if (_id == 1 && millis() % 500 < 10) {  // Print every 500ms
      Serial.print("J1 DEBUG: Tgt=");
      Serial.print(_target_counts * DEGREES_PER_COUNT, 2);
      Serial.print(" Cur=");
      Serial.print(_current_counts * DEGREES_PER_COUNT, 2);
      Serial.print(" Err=");
      Serial.print(error * DEGREES_PER_COUNT, 2);
      Serial.print(" Vel=");
      Serial.print(_current_velocity_counts, 1);
      Serial.print(" TrajMode=");
      Serial.print(_inTrajectoryMode);
      Serial.print(" State=");
      Serial.println(getStateStr());
  }

  // -----------------------------------------------------
  // 3. VIRTUAL JOINT PHYSICS (Use commanded velocity)
  // -----------------------------------------------------
  if (!_isPhysical) {
    if (state != HOLDING && state != IDLE) {
      float move_step = _current_velocity_counts * dt;
      if (move_step > abs_error) move_step = abs_error;
      if (error > 0) _current_counts += move_step;
      else _current_counts -= move_step;
      
      // Wrap
      if(_current_counts >= 4096.0f) _current_counts -= 4096.0f;
      if(_current_counts < 0.0f) _current_counts += 4096.0f;
    }
    
    // Simulation Auto-Stop (only if not in trajectory mode)
    if (abs_error < DEADZONE_COUNTS && state != HOLDING && !_inTrajectoryMode) {
      state = IDLE;
    }
    return; // Skip physical logic in simulation
  }

  // -----------------------------------------------------
  // 4. PHYSICAL MOVEMENT LOGIC
  // -----------------------------------------------------
  
  // ✅ CRITICAL FIX: Skip fine-tuning during trajectory execution
  if (state == FINE_TUNING) {
    // Only do fine tuning if NOT in trajectory mode
    if (_inTrajectoryMode) {
      // Cancel fine tuning, go back to normal operation
      state = CRUISING;
    } else {
      // --- FINE TUNING MODE (MOTOR ENCODER) ---
      int currentMotRaw = getMotorEncoderRaw();
      int delta = currentMotRaw - _lastMotCountRaw;
      
      // Wrap Logic (0-4095)
      if (delta < -2048) delta += 4096;
      else if (delta > 2048) delta -= 4096;
      
      _fineAccumulatedTicks += delta;
      _lastMotCountRaw = currentMotRaw;
      
      long remaining = _fineTargetTicks - _fineAccumulatedTicks;
      
      if (abs(remaining) > 2) {
        bool cw = (MOTOR_DIRECTION_SIGN == -1) ? (remaining > 0) : (remaining < 0);
        setDirection(cw);
        stepGen->setSpeedCounts(FINE_VELOCITY_STEPS);
      } else {
        // FINISHED FINE TUNING
        stop();
      }
      return; // Exit update()
    }
  }

  // --- COARSE MODE (GEARBOX ENCODER) ---
  
  // 1. CHECK IF AT TARGET (DEADZONE)
  float deadzone_threshold = (state == HOLDING) ? HOLDING_DEADZONE_COUNTS : DEADZONE_COUNTS;

  // 1. CHECK IF AT TARGET (DEADZONE)
  if (abs_error < DEADZONE_COUNTS) {
    if (!_inTrajectoryMode) {
      // Traditional point-to-point mode: stop at target
      stepGen->setSpeedCounts(0);
      _lastAppliedVelocity = 0;
      
      if (state != HOLDING) {
        stop(); 
      }
      return;
    } 
    else 
    {
      // ✅ TRAJECTORY MODE: Don't stop! Keep cruising to next waypoint
      // Just ensure we're in motion state
      if (state == HOLDING || state == IDLE) {
        state = CRUISING;
      }
    }
  }


  // 2. ACTIVE POSITION HOLD LOGIC
  // Calculate commanded velocity
  float commandedVelocity;
  
  if (_inTrajectoryMode) {
      // ✅ TRAJECTORY MODE: Use MoveIt's commanded velocity
      commandedVelocity = _maxVelocityCounts;
      
      // Apply creep logic on final waypoint
      if (_isFinalWaypoint && abs(commandedVelocity) < 1.0f && abs_error > DEADZONE_COUNTS) {
          float approachSpeed = abs_error * 5.0f;
          if (approachSpeed > 200.0f) approachSpeed = 200.0f;
          if (approachSpeed < 20.0f) approachSpeed = 20.0f;
          commandedVelocity = approachSpeed;
      }
  }
  else {
      // ✅ HOLDING MODE: Gentle proportional control with large deadband
      const float HOLDING_DEADBAND = 2.0f;  // Ignore errors < 2 counts (~0.18°)
      const float HOLDING_GAIN = 8.0f;       // Velocity = error × 8
      
      if (abs_error < HOLDING_DEADBAND) {
          // Within deadband - don't correct (ignore backlash/noise)
          commandedVelocity = 0.0f;
      }
      else {
          // Outside deadband - gentle proportional correction
          commandedVelocity = abs_error * HOLDING_GAIN;
          
          // Clamp to safe holding speeds
          if (commandedVelocity > 50.0f) commandedVelocity = 50.0f;   // Max ~4.4°/s
          if (commandedVelocity < 10.0f) commandedVelocity = 10.0f;   // Min speed
      }
  }
  
  _current_velocity_counts = commandedVelocity;


  // 3. START/UPDATE LOGIC
  if (state == HOLDING || state == IDLE) {
    state = CRUISING;
  }

  // Dynamic Direction Correction
  bool cw = (MOTOR_DIRECTION_SIGN == -1) ? (error > 0) : (error < 0);
  setDirection(cw);

  // Apply speed to StepGenerator
  if (abs(_current_velocity_counts - _lastAppliedVelocity) > 2.0f) {
    stepGen->setSpeedCounts(_current_velocity_counts);
    _lastAppliedVelocity = _current_velocity_counts;
  }
}

// ============================================================
// GETTERS & HELPERS
// ============================================================
int RoboticJoint::getMotorEncoderRaw() {
  return _motCountRaw_Global;
}

const char* RoboticJoint::getStateStr() {
  switch(state) {
    case IDLE: return "IDLE";
    case CRUISING: return "CRUISE";
    case HOLDING: return "HOLD";
    case FINE_TUNING: return "FINE";
    default: return "?";
  }
}

// ============================================================
// PRIVATE HELPER FUNCTIONS
// ============================================================
float RoboticJoint::countToAngle(float c) {
  while(c<0)c+=4096; while(c>=4096)c-=4096; return (c/4096.0)*360.0;
}

float RoboticJoint::angleToCount(float a) {
  while(a<0)a+=360; while(a>=360)a-=360; return (a/360.0)*4096.0;
}

float RoboticJoint::getMotorAngleTotal(uint16_t motCount, int16_t motRot) {
  float raw = (motCount / 4096.0) * 360.0;
  return (motRot * 360.0) + raw;
}

void RoboticJoint::printStatus() {
  float current_angle = _current_counts * DEGREES_PER_COUNT;
  float target_angle = _target_counts * DEGREES_PER_COUNT;
  float error_deg = (_target_counts - _current_counts) * DEGREES_PER_COUNT;
  
  Serial.println(F("\nSTATUS:"));
  Serial.print(F("  Position: ")); Serial.print(current_angle, 3); Serial.println(F("°"));
  Serial.print(F("  Target: ")); Serial.print(target_angle, 2); Serial.println(F("°"));
  Serial.print(F("  Error: ")); Serial.print(error_deg, 3); Serial.println(F("°"));
  Serial.print(F("  Vel (Cnts): ")); Serial.print(_current_velocity_counts, 1);
  Serial.print(F("  State: ")); Serial.println(getStateStr());
  Serial.println();
}
