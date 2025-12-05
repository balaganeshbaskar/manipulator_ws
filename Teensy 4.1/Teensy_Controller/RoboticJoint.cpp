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
  current_position_count = (float)gbCount;
  current_angle_gb = countToAngle(current_position_count);
  current_angle_motor = getMotorAngleTotal(motCount, motRot) * -1.0;
  
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
  
  state = IDLE; current_velocity = 0.0; stepGen->resetSteps();
  move_start_time = millis(); log_start_time = millis(); move_active = true;
  last_error_degrees = 999.0; correction_attempts = 0;
  Serial.print(F("\nMoving to ")); Serial.print(target_degrees, 4); Serial.println(F("°"));
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
void RoboticJoint::update() {
  // STATIC VARIABLES (EXACT COPY)
  static unsigned long zero_error_start_time = 0;
  static float last_stable_position = 0.0;
  static unsigned long position_hold_timer = 0;
  static unsigned long correction_move_start_time = 0;

  // READ SENSORS
  float error_gb = target_angle_gb - current_angle_gb;
  if (error_gb > 180.0) error_gb -= 360.0;
  if (error_gb < -180.0) error_gb += 360.0;
  
  float effective_error;
  bool use_fine_mode = false;
  
  if (abs(error_gb) > HYBRID_SWITCH_THRESHOLD) {
    effective_error = error_gb;
    } else {
    float ideal_motor_target = current_angle_motor + (error_gb * GEAR_RATIO * MOTOR_DIRECTION_SIGN);
    target_angle_motor = ideal_motor_target; // <--- THIS IS THE BUG
    float error_mot = target_angle_motor - current_angle_motor;
    effective_error = error_mot / (GEAR_RATIO * MOTOR_DIRECTION_SIGN);
    use_fine_mode = true;
  }

  
  position_error_counts = (effective_error / 360.0) * 4096.0;
  float distance_remaining = abs(effective_error);

  // 1. WATCHDOG & HOLDING MAINTENANCE
  if (state == HOLDING) {
      if (distance_remaining > 0.50) {  // WAS distance_remaining > 0.05
          Serial.print(F("⚠ DRIFT DETECTED: ")); Serial.println(effective_error, 4);
          state = CORRECTING;
          move_active = true;
          correction_attempts = 0; 
          correction_move_start_time = millis();
          return;
      }
      stepGen->stop(); 
      return; 
  }


  // 2. TARGET LOCK (EXIT)
  if (distance_remaining < 0.005) { 
      if (zero_error_start_time == 0) zero_error_start_time = millis();
      if (millis() - zero_error_start_time > 50) {
          if (state != HOLDING) {
              stepGen->stop(); 
              current_velocity = 0.0; 
              state = HOLDING; 
              Serial.println(F("→ Target Lock. Holding."));
              printMoveSummary();
          }
          return;
      }
  } else {
      zero_error_start_time = 0;
  }

  // 3. STATE MACHINE
  if (distance_remaining < DEADZONE && state != SETTLING && state != HOLDING) {
      stepGen->stop(); current_velocity = 0.0; state = SETTLING; settling_start_time = millis();
      if (correction_attempts == 0) Serial.println(F("→ Entering settling phase..."));
      return;
  }
  
  if (state == SETTLING) {
    stepGen->stop(); current_velocity = 0.0;
    if (millis() - settling_start_time >= SETTLING_DURATION) {
      if (distance_remaining <= PRECISION_THRESHOLD) { 
        state = HOLDING; move_active = false; printMoveSummary(); return; 
      }
      else if (correction_attempts < MAX_CORRECTION_ATTEMPTS) {
        correction_attempts++;
        Serial.print(F("→ Correction #")); Serial.print(correction_attempts);
        Serial.print(F(": Err:")); Serial.println(effective_error, 4);
        state = CORRECTING;
        correction_move_start_time = millis();
      } else {
        state = HOLDING; move_active = false;
        Serial.print(F("⚠ Max corrections. Error: ")); Serial.print(effective_error, 4); Serial.println(F("°"));
        printMoveSummary(); return;
      }
    }
    return;
  }
  
  // OVERSHOOT PROTECTION
  if (abs(last_error_degrees) > DEADZONE && abs(last_error_degrees) < 999.0) {
    if ((last_error_degrees > 0 && effective_error < 0) || (last_error_degrees < 0 && effective_error > 0)) {
      if (state != CORRECTING || (millis() - correction_move_start_time > 250)) {
          if (abs(effective_error) > PRECISION_THRESHOLD) {
              Serial.println(F("⚠ Overshoot. Stopping."));
              stepGen->stop(); state = SETTLING; settling_start_time = millis(); return;
          }
      }
    }
  }
  last_error_degrees = effective_error;
  
  float max_decel_dist = (_maxVelocity * _maxVelocity) / (2.0 * ACCELERATION);
  float decel_start_distance = max_decel_dist * DECEL_SAFETY_FACTOR;

  // DIRECTION LOGIC
  bool need_positive = (effective_error > 0.0);
  if (MOTOR_DIRECTION_SIGN == -1) direction_cw = need_positive;
  else direction_cw = !need_positive;

  // MOTION EXECUTION
  if (state == CORRECTING) {

    if (distance_remaining > 50.0) {
      // Target jumped far away - switch back to acceleration
      Serial.println(F("→ Large target change detected, re-accelerating..."));
      state = ACCELERATING;
    } else
    {
      current_velocity = CORRECTION_SPEED;
      if (distance_remaining < 0.02) current_velocity = 0.2;
      setDirection(direction_cw);
    }
    
  } else if (state == IDLE) {
    setDirection(direction_cw); 
    current_velocity = 0.0; state = ACCELERATING;
  } else if (state == ACCELERATING) {
    current_velocity += ACCELERATION * 0.02;
    if (current_velocity >= _maxVelocity) { current_velocity = _maxVelocity; state = CRUISING; }
    if (distance_remaining <= decel_start_distance) state = DECELERATING;
  } else if (state == CRUISING) {
    current_velocity = _maxVelocity;
    if (distance_remaining <= decel_start_distance) state = DECELERATING;
  } else if (state == DECELERATING) {

    // NEW: Check if target changed dramatically while decelerating
    if (distance_remaining > 50.0) {
      // Target jumped far away - switch back to acceleration
      Serial.println(F("→ Large target change detected, re-accelerating..."));
      state = ACCELERATING;
    } else {
      // Normal deceleration
      current_velocity -= ACCELERATION * 0.02;
      float min_speed = MIN_SPEED_FAR;
      if (use_fine_mode) min_speed = MIN_SPEED_CLOSE; 
      else if (distance_remaining < 5.0) min_speed = MIN_SPEED_NEAR;
      if (current_velocity < min_speed) current_velocity = min_speed;
    }

  } else if (state == HOLDING) { stepGen->stop(); return; }
  
  stepGen->setSpeed(current_velocity);
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

