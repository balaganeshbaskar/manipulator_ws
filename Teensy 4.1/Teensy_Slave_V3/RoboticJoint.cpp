#include "RoboticJoint.h"

RoboticJoint::RoboticJoint(uint8_t id, uint8_t stepPin, uint8_t dirPin, uint8_t enaPin) 
  : _id(id), _stepPin(stepPin), _dirPin(dirPin), _enaPin(enaPin) {
  stepGen = new StepGenerator(_stepPin);
}

void RoboticJoint::begin() {
  pinMode(_dirPin, OUTPUT);
  pinMode(_enaPin, OUTPUT);
  digitalWrite(_stepPin, LOW);
  digitalWrite(_dirPin, LOW);
  digitalWrite(_enaPin, LOW); // EXACT COPY: LOW at start
}

void RoboticJoint::waitForEncoder() {
  Serial.print(F("Waiting for encoder..."));
  while (!_dataValid) { delay(10); }
  
  // CRITICAL INIT SEQUENCE (EXACT COPY FROM ORIGINAL)
  current_position_count = target_count; // Already set by updateSensorData
  target_angle_gb = countToAngle(target_count);
  current_angle_motor = getMotorAngleTotal(0, 0) * -1.0; // Will be updated next poll
  target_angle_motor = current_angle_motor;
  state = HOLDING;
  move_active = true; // Enable Watchdog immediately
  
  Serial.println(F(" OK"));
  Serial.print(F("Initial: ")); Serial.print(countToAngle(current_position_count), 4); Serial.println(F("°"));
  Serial.print(F("Target Set To: ")); Serial.print(target_angle_gb, 4); Serial.println(F("°"));
}

void RoboticJoint::updateSensorData(uint16_t gbCount, uint16_t motCount, int16_t motRot) {
  // 1. Update Timestamp (Heartbeat)
  last_update_time = millis(); 

  // 2. Process Data
  current_position_count = (float)gbCount;
  current_angle_gb = countToAngle(current_position_count);
  current_angle_motor = getMotorAngleTotal(motCount, motRot) * -1.0;
  
  // 3. First Run Init
  if (!_dataValid) {
    target_count = current_position_count;
    target_angle_gb = current_angle_gb;
    target_angle_motor = current_angle_motor;
  }
  _dataValid = true;
}


void RoboticJoint::moveTo(float target_degrees) {
  digitalWrite(_enaPin, LOW);
  target_count = angleToCount(target_degrees);
  target_angle_gb = target_degrees;
  float current_gb = countToAngle(current_position_count);
  float delta_gb = target_degrees - current_gb;
  if (delta_gb > 180.0) delta_gb -= 360.0;
  if (delta_gb < -180.0) delta_gb += 360.0;
  
  float current_mot = current_angle_motor;
  target_angle_motor = current_mot + (delta_gb * GEAR_RATIO * MOTOR_DIRECTION_SIGN);
  
  state = IDLE; 
  current_velocity = 0.5; 
  stepGen->resetSteps();
  move_start_time = millis(); 
  log_start_time = millis(); 
  move_active = true;
  last_error_degrees = 999.0; 
  correction_attempts = 0;
  Serial.print(F("\nMoving to ")); 
  Serial.print(target_degrees, 4); 
  Serial.println(F("°"));
}

void RoboticJoint::updateTarget(float new_target_degrees) {
  // Only update target, DO NOT reset velocity or state
  target_count = angleToCount(new_target_degrees);
  target_angle_gb = new_target_degrees;
  
  float current_gb = countToAngle(current_position_count);
  float delta_gb = new_target_degrees - current_gb;
  
  // Handle wrap-around
  if (delta_gb > 180.0) delta_gb -= 360.0;
  if (delta_gb < -180.0) delta_gb += 360.0;
  
  float current_mot = current_angle_motor;
  target_angle_motor = current_mot + (delta_gb * GEAR_RATIO * MOTOR_DIRECTION_SIGN);
  
  // DO NOT reset these:
  // state = IDLE;              // Keep current state (CRUISE)
  // current_velocity = 0.0;    // Keep current velocity
  // stepGen->resetSteps();     // Don't reset steps
  
  // Only reset correction tracking
  last_error_degrees = 999.0;
  correction_attempts = 0;
  
  // Mark as active move if not already
  if (!move_active) {
    move_active = true;
    move_start_time = millis();
  }
  
  Serial.print(F("→ Updated target to "));
  Serial.print(new_target_degrees, 2);
  Serial.println(F("° (smooth)"));
}


void RoboticJoint::moveRelative(float delta) {
  moveTo(countToAngle(current_position_count) + delta);
}

void RoboticJoint::stop() {
  stepGen->stop();
  state = HOLDING;
  move_active = true;
  digitalWrite(_enaPin, LOW);
  Serial.println(F("STOPPED (HOLDING)"));
}

void RoboticJoint::relax() {
  stepGen->stop();
  state = HOLDING;
  move_active = false;
  digitalWrite(_enaPin, HIGH);
  Serial.println(F("RELAXED (OFF)"));
}

// ============================================================
// UPDATE CONTROL LOOP (EXACT COPY FROM ORIGINAL)
// ============================================================
#include "RoboticJoint.h"

// ... (Constructors remain same)

void RoboticJoint::update() {
    // -----------------------------------------------------
    // 1. CALCULATE ERROR
    // -----------------------------------------------------
    float error = target_angle_gb - current_angle_gb;
    
    // Wraparound fix (-180 to 180)
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    
    float abs_error = abs(error);

    // -----------------------------------------------------
    // 2. VIRTUAL JOINT SIMULATION
    // -----------------------------------------------------
    if (!_isPhysical) {
        unsigned long now = micros();
        float dt = (now - last_sim_time) / 1000000.0;
        last_sim_time = now;
        if(dt > 0.1) dt = 0.0; // Prevent huge jumps on first loop

        // Simulate motion: Move 'current_angle_gb' towards target by current_velocity * dt
        if (state != HOLDING && state != IDLE) {
            float move_step = current_velocity * dt;
            if (move_step > abs_error) move_step = abs_error; // Don't overshoot
            
            if (error > 0) current_angle_gb += move_step;
            else           current_angle_gb -= move_step;
            
            // Normalize virtual angle
            if(current_angle_gb >= 360.0) current_angle_gb -= 360.0;
            if(current_angle_gb < 0.0)    current_angle_gb += 360.0;
            
            // Sync counts for consistency
            current_position_count = angleToCount(current_angle_gb);
        }
    }

    // -----------------------------------------------------
    // 3. STATE MACHINE (Shared by Real & Virtual)
    // -----------------------------------------------------
    
    // Check Arrival
    if (abs_error < DEADZONE) {
        if (state != HOLDING) {
            stop(); 
        }
        return;
    }

    // State Transitions
    if (state == HOLDING || state == IDLE) {
        state = ACCELERATING;
        if (_isPhysical) {
            // Only toggle pins if real
            if (MOTOR_DIRECTION_SIGN == -1) setDirection(error > 0);
            else setDirection(error < 0);
        }
    }
    
    // Velocity Profiling (Ramp)
    if (state == ACCELERATING) {
        current_velocity += ACCELERATION * 0.005; 
        if (current_velocity >= _maxVelocity) {
            current_velocity = _maxVelocity;
            state = CRUISING;
        }
    }
    else if (state == CRUISING) {
        float decel_dist = (current_velocity * current_velocity) / (2 * ACCELERATION);
        if (abs_error <= decel_dist) {
            state = DECELERATING;
        }
    }
    else if (state == DECELERATING) {
        current_velocity -= ACCELERATION * 0.01;
        if (current_velocity < 1.0) current_velocity = 1.0; 
    }

    // -----------------------------------------------------
    // 4. HARDWARE EXECUTION (Real Only)
    // -----------------------------------------------------
    if (_isPhysical) {
        // FILTER: Only update timer if velocity changed by > 1% 
        // OR if we are stopping (velocity is 0).
        if (abs(current_velocity - _lastAppliedVelocity) > 0.1 || current_velocity == 0.0) {
            stepGen->setSpeed(current_velocity);
            _lastAppliedVelocity = current_velocity;
        }
    }
}



// Helpers
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
void RoboticJoint::setDirection(bool cw) {
  digitalWriteFast(_dirPin, cw ? HIGH : LOW); delayMicroseconds(5);
}

const char* RoboticJoint::getStateStr() {
  switch(state) {
    case IDLE: return "IDLE";
    case ACCELERATING: return "ACCEL";
    case CRUISING: return "CRUISE";
    case DECELERATING: return "DECEL";
    case SETTLING: return "SETTLING";
    case CORRECTING: return "CORRECT";
    case HOLDING: return "HOLD";
    default: return "?";
  }
}

void RoboticJoint::printMoveSummary() {
  unsigned long duration_ms = millis() - move_start_time;
  float gb_coarse = countToAngle(current_position_count);
  float final_err = (position_error_counts / 4096.0) * 360.0;
  
  Serial.println(F("\n╔═══════════════════════════════════════╗"));
  Serial.println(F("║         MOVE COMPLETE                 ║"));
  Serial.println(F("╠═══════════════════════════════════════╣"));
  Serial.print(F("║ Target:      ")); Serial.print(countToAngle(target_count), 4); Serial.println(F("°"));
  Serial.print(F("║ Final GB:    ")); Serial.print(gb_coarse, 4); Serial.println(F("° (Coarse)"));
  Serial.print(F("║ Error:       ")); Serial.print(final_err, 4); Serial.println(F("° (Fine)"));
  Serial.print(F("║ Corrections: ")); Serial.println(correction_attempts);
  Serial.print(F("║ Duration:    ")); Serial.print(duration_ms / 1000.0, 2); Serial.println(F(" s"));
  Serial.print(F("║ Steps:       ")); Serial.println(stepGen->getSteps());
  Serial.print(F("║ Success:     "));
  Serial.println(abs(final_err) < MAX_ACCEPTABLE_ERROR ? F("YES ✓") : F("NO ✗"));
  Serial.println(F("╚═══════════════════════════════════════╝\n"));
}


void RoboticJoint::printStatus() {
  float current_angle = countToAngle(current_position_count);
  float target_angle = countToAngle(target_count);
  float error_deg = (position_error_counts / 4096.0) * 360.0;
  
  Serial.println(F("\nSTATUS:"));
  Serial.print(F("  Position:    ")); Serial.print(current_angle, 3); Serial.println(F("°"));
  Serial.print(F("  Target:      ")); Serial.print(target_angle, 2); Serial.println(F("°"));
  Serial.print(F("  Error:       ")); Serial.print(error_deg, 3); Serial.println(F("°"));
  Serial.print(F("  Velocity:    ")); Serial.print(current_velocity, 2); Serial.println(F(" deg/s"));
  Serial.print(F("  Corrections: ")); Serial.println(correction_attempts);
  Serial.print(F("  State:       ")); Serial.println(getStateStr());
  Serial.println();
}
