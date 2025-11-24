// ============================================================
// PRECISION JOINT CONTROLLER - ULTRA PRECISION (0.01 deg)
// ============================================================
// - Resolution: 0.0017 deg (Sensor) / 0.0045 deg (Step)
// - Reporting: 4 Decimal Places
// - Logic: Sensor Fusion (Gearbox Coarse + Motor Fine)
// ============================================================

#include <TeensyTimerTool.h>
#include <CRC16.h>

using namespace TeensyTimerTool;

// ============================================================
// CONSTANTS
// ============================================================
const float GEAR_RATIO = 50.0;
const int MOTOR_DIRECTION_SIGN = -1; // -1 because Motor CW = Gearbox CCW
const uint16_t MICROSTEPS = 8;
const float MOTOR_STEPS_PER_REV = 200.0 * MICROSTEPS;
const float GEARBOX_STEPS_PER_DEGREE = (MOTOR_STEPS_PER_REV / 360.0) * GEAR_RATIO;

// ============================================================
// MOTION PARAMETERS
// ============================================================
const float MAX_VELOCITY = 40.0;
const float ACCELERATION = 100.0; // was 80
const float DECEL_SAFETY_FACTOR = 2.0;

// >>> ULTRA PRECISION SETTINGS <<<
const float DEADZONE = 0.050;              // 5 thousandths of a degree
const float PRECISION_THRESHOLD = 0.01;    // Target: +/- 0.01 degree
const float HYBRID_SWITCH_THRESHOLD = 2.0; 

const float MAX_ACCEPTABLE_ERROR = 0.5;
const float CORRECTION_SPEED = 1.5;        // 1- Slower correction for micro-steps

const float MIN_SPEED_FAR = 8.0;
const float MIN_SPEED_NEAR = 4.0;
const float MIN_SPEED_CLOSE = 1;         // 1 Crawl speed for final micron-landing

const unsigned long SETTLING_DURATION = 150; // Slightly longer to settle vibration
const uint8_t MAX_CORRECTION_ATTEMPTS = 5;   // More attempts allowed for high precision

// ============================================================
// HARDWARE PINS
// ============================================================
struct JointPins {
  uint8_t step, dir, ena;
};
const JointPins pins[5] = {
  {3, 4, 5}, {6, 7, 8}, {9, 10, 11}, {12, 13, 14}, {15, 16, 17}
};
#define ACTIVE_JOINT 0

// RS485
#define RS485_SERIAL Serial1
const int RS485_DE_PIN = 2;
const uint32_t RS485_BAUD = 57600;
const uint8_t JOINT_ID = 1;
CRC16 crc;

// ============================================================
// STATES
// ============================================================
enum ControlState {
  IDLE, ACCELERATING, CRUISING, DECELERATING, SETTLING, CORRECTING, HOLDING
};
ControlState state = HOLDING;

// ============================================================
// STEP GENERATOR CLASS
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
  
  void setSpeed(float deg_per_sec) {
    if (abs(deg_per_sec) < 0.001) { stop(); return; } // Higher sensitivity stop
    float steps_per_sec = abs(deg_per_sec) * GEARBOX_STEPS_PER_DEGREE;
    float interval_us = 1000000.0 / steps_per_sec;
    if (interval_us < 5.0) interval_us = 5.0;
    if (interval_us > 500000.0) { stop(); return; } // Allow slower pulses
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
// VARIABLES
// ============================================================
float target_angle_gb = 0.0;
float target_angle_motor = 0.0;
float current_angle_gb = 0.0;
float current_angle_motor = 0.0;
float target_count = 0.0;
float current_position_count = 0.0; 
float position_error_counts = 0.0;
float current_velocity = 0.0;
float last_error_degrees = 0.0;
bool direction_cw = true;
bool move_active = false;
unsigned long settling_start_time = 0;
uint8_t correction_attempts = 0;
StepGenerator stepGen(pins[ACTIVE_JOINT].step);
unsigned long move_start_time = 0;
unsigned long log_start_time = 0;

struct JointData {
  uint16_t motorCount;
  int16_t motorRotations;
  uint16_t gearboxCount;
  bool limitSwitch1;
  bool limitSwitch2;
  uint8_t statusByte;
  bool dataValid;
  unsigned long lastUpdate;
} joint1;

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  pinMode(pins[ACTIVE_JOINT].step, OUTPUT);
  pinMode(pins[ACTIVE_JOINT].dir,  OUTPUT);
  pinMode(pins[ACTIVE_JOINT].ena,  OUTPUT);
  digitalWrite(pins[ACTIVE_JOINT].step, LOW);
  digitalWrite(pins[ACTIVE_JOINT].dir,  LOW);
  digitalWrite(pins[ACTIVE_JOINT].ena,  LOW);
  RS485_SERIAL.begin(RS485_BAUD);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  
  Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
  Serial.println(F("║  PRECISION JOINT CONTROLLER - ULTRA HIGH RES      ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════╝\n"));
  
  Serial.print(F("Waiting for encoder..."));
  while (!joint1.dataValid) { pollJoint(); delay(10); }
  current_position_count = (float)joint1.gearboxCount;
  Serial.println(F(" OK"));
  Serial.print(F("Initial: ")); Serial.print(countToAngle(current_position_count), 4); Serial.println(F("°"));
  Serial.println(F("Ready! Try: M 150"));
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  static unsigned long lastPoll = 0;
  static unsigned long lastLog = 0;

  updateTestManager();

  processCommands();
  if (millis() - lastPoll >= 20) {
    pollJoint();
    if (joint1.dataValid && move_active && state != HOLDING) updateControlLoop();
    else if (state == HOLDING) stepGen.stop();
    lastPoll = millis();
  }
  if (millis() - lastLog >= 100) {
    if (state != HOLDING && move_active) logData();
    lastLog = millis();
  }
}

// Helper: Get High Resolution Output Angle (Derived from Motor)
float getHighResOutputAngle() {
  float motor_angle = (joint1.motorCount / 4096.0) * 360.0;
  float total_motor = (joint1.motorRotations * 360.0) + motor_angle;
  
  // Convert to output shaft degrees, accounting for sign
  // Note: This value is relative to startup or calibration. 
  // For absolute readout we combine it with Gearbox coarse data.
  // But for logging movements, this raw high-res value / ratio is fine.
  return total_motor / (GEAR_RATIO * MOTOR_DIRECTION_SIGN); 
}

// Standard Motor Angle
float getMotorAngleTotal() {
  float raw = (joint1.motorCount / 4096.0) * 360.0;
  return (joint1.motorRotations * 360.0) + raw;
}

void processCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim(); if (cmd.length() == 0) return;
  char t = cmd.charAt(0);
  if (t == 'M') moveToAngle(cmd.substring(2).toFloat());
  else if (t == 'R') moveRelative(cmd.substring(2).toFloat());
  else if (t == 'S') { stepGen.stop(); state = HOLDING; move_active = false; digitalWrite(pins[ACTIVE_JOINT].ena, LOW); Serial.println(F("STOPPED")); }
  else if (t == 'X') { stepGen.stop(); state = HOLDING; move_active = false; digitalWrite(pins[ACTIVE_JOINT].ena, HIGH); Serial.println(F("RELAXED")); }
  else if (t == 'Q') { pollJoint(); printStatus(); }
  else if (t == 'T') printStatus();
  else if (t == 'Z') {
    // Format: Z <cycles> <side>
    // Example: "Z 10 1" (10 cycles, Positive side)
    // Example: "Z 5 -1" (5 cycles, Negative side)
    
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    
    int cycles = 5;
    int side = 1; // Default to Positive if not specified
    
    if (firstSpace > 0) {
      if (secondSpace > 0) {
        cycles = cmd.substring(firstSpace + 1, secondSpace).toInt();
        side = cmd.substring(secondSpace + 1).toInt();
      } else {
        cycles = cmd.substring(firstSpace + 1).toInt();
      }
    }
    startAccuracyTest(cycles, side);
  }
}

void moveToAngle(float target_degrees) {
  digitalWrite(pins[ACTIVE_JOINT].ena, LOW);
  target_count = angleToCount(target_degrees);
  target_angle_gb = target_degrees;
  float current_gb = countToAngle(joint1.gearboxCount);
  float delta_gb = target_degrees - current_gb;
  if (delta_gb > 180.0) delta_gb -= 360.0;
  if (delta_gb < -180.0) delta_gb += 360.0;
  
  float current_mot = getMotorAngleTotal();
  target_angle_motor = current_mot + (delta_gb * GEAR_RATIO * MOTOR_DIRECTION_SIGN);
  
  state = IDLE; current_velocity = 0.0; stepGen.resetSteps();
  move_start_time = millis(); log_start_time = millis(); move_active = true;
  last_error_degrees = 999.0; correction_attempts = 0;
  Serial.print(F("\nMoving to ")); Serial.print(target_degrees, 4); Serial.println(F("°"));
}

void moveRelative(float delta) { moveToAngle(countToAngle(joint1.gearboxCount) + delta); }
float angleToCount(float a) { while(a<0)a+=360; while(a>=360)a-=360; return (a/360.0)*4096.0; }
float countToAngle(float c) { while(c<0)c+=4096; while(c>=4096)c-=4096; return (c/4096.0)*360.0; }
void setDirection(bool cw) { digitalWriteFast(pins[ACTIVE_JOINT].dir, cw ? HIGH : LOW); delayMicroseconds(5); }

void updateControlLoop() {
  // --- STATIC VARIABLES ---
  static unsigned long zero_error_start_time = 0;
  static float last_stable_position = 0.0;
  static unsigned long position_hold_timer = 0;
  static unsigned long correction_move_start_time = 0; // NEW: Track when correction started

  // --- READ SENSORS ---
  current_position_count = (float)joint1.gearboxCount;
  current_angle_gb = countToAngle(current_position_count);
  current_angle_motor = getMotorAngleTotal() * -1.0; 
  
  // --- CALCULATE ERRORS ---
  float error_gb = target_angle_gb - current_angle_gb;
  if (error_gb > 180.0) error_gb -= 360.0;
  if (error_gb < -180.0) error_gb += 360.0;
  
  float effective_error;
  bool use_fine_mode = false;
  
  if (abs(error_gb) > HYBRID_SWITCH_THRESHOLD) {
    effective_error = error_gb;
  } else {
    float ideal_motor_target = current_angle_motor + (error_gb * GEAR_RATIO * MOTOR_DIRECTION_SIGN);
    target_angle_motor = ideal_motor_target;
    float error_mot = target_angle_motor - current_angle_motor;
    effective_error = error_mot / (GEAR_RATIO * MOTOR_DIRECTION_SIGN);
    use_fine_mode = true;
  }
  
  position_error_counts = (effective_error / 360.0) * 4096.0;
  float distance_remaining = abs(effective_error);

  // ============================================================
  // 1. INSTANT "TARGET LOCK"
  // ============================================================
  if (distance_remaining < 0.005) { 
      if (zero_error_start_time == 0) zero_error_start_time = millis();
      if (millis() - zero_error_start_time > 50) {
          if (state != HOLDING) {
              stepGen.stop(); 
              current_velocity = 0.0; 
              state = HOLDING; 
              move_active = false; 
              Serial.println(F("→ Target Lock (Zero Error). Stop."));
              printMoveSummary();
          }
          return;
      }
  } else {
      zero_error_start_time = 0;
  }

  // ============================================================
  // 2. STALL DETECTION
  // ============================================================
  if (state == CORRECTING) {
      if (abs(current_angle_gb - last_stable_position) < 0.002) {
          if (millis() - position_hold_timer > 1000) { 
             Serial.println(F("⚠ STALL DETECTED: Forcing Finish."));
             stepGen.stop(); state = HOLDING; move_active = false; printMoveSummary(); return;
          }
      } else {
          last_stable_position = current_angle_gb;
          position_hold_timer = millis();
      }
  } else {
      position_hold_timer = millis();
      last_stable_position = current_angle_gb;
  }

  // ============================================================
  // 3. NORMAL STATE MACHINE
  // ============================================================
  
  // Entering Settling
  if (distance_remaining < DEADZONE && state != SETTLING && state != HOLDING) {
      stepGen.stop(); current_velocity = 0.0; state = SETTLING; settling_start_time = millis();
      if (correction_attempts == 0) Serial.println(F("→ Entering settling phase..."));
      return;
  }
  
  if (state == SETTLING) {
    stepGen.stop(); current_velocity = 0.0;
    if (millis() - settling_start_time >= SETTLING_DURATION) {
      if (distance_remaining <= PRECISION_THRESHOLD) { 
        state = HOLDING; move_active = false; printMoveSummary(); return; 
      }
      else if (correction_attempts < MAX_CORRECTION_ATTEMPTS) {
        correction_attempts++;
        Serial.print(F("→ Correction #")); Serial.print(correction_attempts);
        Serial.print(F(": Err:")); Serial.println(effective_error, 4);
        state = CORRECTING;
        correction_move_start_time = millis(); // >>> NEW: Timestamp start of correction move
        
        bool need_positive = (effective_error > 0.0);
        if (MOTOR_DIRECTION_SIGN == -1) direction_cw = need_positive;
        else direction_cw = !need_positive;
        setDirection(direction_cw); 
      } else {
        state = HOLDING; move_active = false;
        Serial.print(F("⚠ Max corrections. Error: ")); Serial.print(effective_error, 4); Serial.println(F("°"));
        printMoveSummary(); return;
      }
    }
    return;
  }
  
  // ============================================================
  // OVERSHOOT PROTECTION (WITH GRACE PERIOD)
  // ============================================================
  if (abs(last_error_degrees) > DEADZONE && abs(last_error_degrees) < 999.0) {
    if ((last_error_degrees > 0 && effective_error < 0) || (last_error_degrees < 0 && effective_error > 0)) {
      
      // >>> FIX: GRACE PERIOD <<<
      // Only trigger overshoot stop if we have been moving for > 100ms.
      // This allows the motor to cross the backlash gap without false triggering.
      if (state != CORRECTING || (millis() - correction_move_start_time > 250)) {
          if (abs(effective_error) > PRECISION_THRESHOLD) {
              Serial.println(F("⚠ Overshoot. Stopping."));
              stepGen.stop(); state = SETTLING; settling_start_time = millis(); return;
          }
      }
    }
  }
  last_error_degrees = effective_error;
  
  // Velocity Profiles
  float max_decel_dist = (MAX_VELOCITY * MAX_VELOCITY) / (2.0 * ACCELERATION);
  float decel_start_distance = max_decel_dist * DECEL_SAFETY_FACTOR;

  // Direction Logic
  bool need_positive = (effective_error > 0.0);
  if (MOTOR_DIRECTION_SIGN == -1) direction_cw = need_positive;
  else direction_cw = !need_positive;

  if (state == CORRECTING) {
    current_velocity = CORRECTION_SPEED;
    if (distance_remaining < 0.02) current_velocity = 0.2; 
  } else if (state == IDLE) {
    setDirection(direction_cw); 
    current_velocity = 0.0; state = ACCELERATING;
  } else if (state == ACCELERATING) {
    current_velocity += ACCELERATION * 0.02;
    if (current_velocity >= MAX_VELOCITY) { current_velocity = MAX_VELOCITY; state = CRUISING; }
    if (distance_remaining <= decel_start_distance) state = DECELERATING;
  } else if (state == CRUISING) {
    current_velocity = MAX_VELOCITY;
    if (distance_remaining <= decel_start_distance) state = DECELERATING;
  } else if (state == DECELERATING) {
    current_velocity -= ACCELERATION * 0.02;
    float min_speed = MIN_SPEED_FAR;
    if (use_fine_mode) min_speed = MIN_SPEED_CLOSE; else if (distance_remaining < 5.0) min_speed = MIN_SPEED_NEAR;
    if (current_velocity < min_speed) current_velocity = min_speed;
  } else if (state == HOLDING) { stepGen.stop(); return; }
  
  stepGen.setSpeed(current_velocity);
}







void pollJoint() {
  digitalWrite(RS485_DE_PIN, LOW); delayMicroseconds(100);
  uint8_t msg[6] = {0x02, JOINT_ID, 0x06, 0, 0, 0x03};
  crc.restart(); crc.add(msg[0]); crc.add(msg[1]); crc.add(msg[2]);
  uint16_t c = crc.calc(); msg[3]=c>>8; msg[4]=c&0xFF;
  while(RS485_SERIAL.available()) RS485_SERIAL.read();
  digitalWrite(RS485_DE_PIN, HIGH); delayMicroseconds(100);
  RS485_SERIAL.write(msg, 6); RS485_SERIAL.flush();
  delayMicroseconds(100); digitalWrite(RS485_DE_PIN, LOW);
  uint8_t r[13]; if(receiveResponse(r, 13, 200)) parseResponse(r);
}
bool receiveResponse(uint8_t *r, size_t len, unsigned long to) {
  unsigned long s=millis(); size_t i=0; bool stx=false;
  while(millis()-s<to && i<len) {
    if(RS485_SERIAL.available()) {
      uint8_t b=RS485_SERIAL.read();
      if(!stx && b==0x02) { stx=true; r[i++]=b; } else if(stx) r[i++]=b;
    }
  } return (i==len && r[len-1]==0x03);
}
void parseResponse(uint8_t *r) {
  crc.restart(); for(int i=0; i<=9; i++) crc.add(r[i]);
  if(crc.calc()!=(((uint16_t)r[10]<<8)|r[11])) return;
  joint1.motorCount=((uint16_t)r[2]<<8)|r[3];
  joint1.motorRotations=((int16_t)r[4]<<8)|r[5];
  joint1.gearboxCount=((uint16_t)r[6]<<8)|r[7];
  joint1.statusByte=r[9]; joint1.dataValid=true; joint1.lastUpdate=millis();
}

// ============================================================
// HIGH PRECISION LOGGING
// ============================================================
void logData() {
  if (!joint1.dataValid) return;
  
  // Logic to display High Res position in logs
  float gb_coarse = countToAngle(current_position_count);
  // float mot_fine_relative = getHighResOutputAngle(); // Raw High Res angle
  
  // We print the Gearbox (Absolute) but calculated error is High Res
  float effective_error = (position_error_counts / 4096.0) * 360.0;
  
  const char* sStr;
  switch(state) {
    case IDLE: sStr="IDLE"; break;
    case ACCELERATING: sStr="ACCEL"; break;
    case CRUISING: sStr="CRUISE"; break;
    case DECELERATING: sStr="DECEL"; break;
    case SETTLING: sStr="SETTLING"; break;
    case CORRECTING: sStr="CORRECT"; break;
    case HOLDING: sStr="HOLD"; break;
    default: sStr="?"; break;
  }
  
  Serial.print(F("[")); Serial.print((millis() - log_start_time) / 1000.0, 1);
  Serial.print(F("s] T:")); Serial.print(countToAngle(target_count), 2);
  Serial.print(F("° Pos:")); Serial.print(gb_coarse, 3); // Still show coarse pos for context
  Serial.print(F("° Err:")); Serial.print(effective_error, 4); // Show HIGH RES error
  Serial.print(F("° V:")); Serial.print(current_velocity, 1);
  Serial.print(F(" ")); Serial.println(sStr);
}

void printMoveSummary() 
{
  unsigned long duration_ms = millis() - move_start_time;
  float gb_coarse = countToAngle(joint1.gearboxCount);
  float final_err = (position_error_counts / 4096.0) * 360.0; // High Res Error
  
  Serial.println(F("\n╔═══════════════════════════════════════╗"));
  Serial.println(F("║         MOVE COMPLETE                 ║"));
  Serial.println(F("╠═══════════════════════════════════════╣"));
  Serial.print(F("║ Target:      ")); Serial.print(countToAngle(target_count), 4); Serial.println(F("°"));
  Serial.print(F("║ Final GB:    ")); Serial.print(gb_coarse, 4); Serial.println(F("° (Coarse)"));
  Serial.print(F("║ Error:       ")); Serial.print(final_err, 4); Serial.println(F("° (Fine)"));
  Serial.print(F("║ Corrections: ")); Serial.println(correction_attempts);
  Serial.print(F("║ Duration:    ")); Serial.print(duration_ms / 1000.0, 2); Serial.println(F(" s"));
  Serial.print(F("║ Steps:       ")); Serial.println(stepGen.getSteps());
  Serial.print(F("║ Success:     "));
  Serial.println(abs(final_err) < MAX_ACCEPTABLE_ERROR ? F("YES ✓") : F("NO ✗"));
  Serial.println(F("╚═══════════════════════════════════════╝\n"));
}


void printStatus() {
  pollJoint();
  if (joint1.dataValid) {
    current_position_count = (float)joint1.gearboxCount;
  }
  
  float current_angle = countToAngle(current_position_count);
  float target_angle = countToAngle(target_count);
  float error_deg = (position_error_counts / 4096.0) * 360.0;
  
  Serial.println(F("\nSTATUS:"));
  Serial.print(F("  Position:    ")); Serial.print(current_angle, 3); Serial.println(F("°"));
  Serial.print(F("  Target:      ")); Serial.print(target_angle, 2); Serial.println(F("°"));
  Serial.print(F("  Error:       ")); Serial.print(error_deg, 3); Serial.println(F("°"));
  Serial.print(F("  Velocity:    ")); Serial.print(current_velocity, 2); Serial.println(F(" deg/s"));
  Serial.print(F("  Corrections: ")); Serial.println(correction_attempts);
  Serial.print(F("  State:       "));
  switch(state) {
    case IDLE: Serial.println(F("IDLE")); break;
    case ACCELERATING: Serial.println(F("ACCELERATING")); break;
    case CRUISING: Serial.println(F("CRUISING")); break;
    case DECELERATING: Serial.println(F("DECELERATING")); break;
    case SETTLING: Serial.println(F("SETTLING")); break;
    case CORRECTING: Serial.println(F("CORRECTING")); break;
    case HOLDING: Serial.println(F("HOLDING")); break;
  }
  Serial.println();
}






// ============================================================
// ADVANCED METROLOGY TEST MANAGER
// ============================================================

struct TestMetric {
  uint16_t cycle_num;
  int8_t   direction;        // 1 = Returning from Positive, -1 = Returning from Negative
  float    start_angle;
  uint16_t start_raw_count;
  float    end_angle;
  uint16_t end_raw_count;
  float    target_angle;
  float    final_error_deg;
  float    final_error_counts;
  unsigned long duration;
  uint8_t  corrections;
};

// Configuration
const int MAX_TEST_CYCLES = 50; 
TestMetric test_data[MAX_TEST_CYCLES];
int test_total_cycles = 0;
int test_current_cycle = 0;
bool test_active = false;
unsigned long test_dwell_timer = 0;
const unsigned long DWELL_TIME_MS = 1500; // Slightly faster dwell to speed up testing

// 1=Pos Only, -1=Neg Only, 0=Mixed
int test_side_constraint = 1; 

enum TestState { T_IDLE, T_MOVING_OUT, T_MOVING_HOME, T_MEASURING };
TestState tState = T_IDLE;

void startAccuracyTest(int cycles, int side) {
  if (cycles > MAX_TEST_CYCLES) cycles = MAX_TEST_CYCLES;
  if (cycles < 1) cycles = 1;
  
  test_total_cycles = cycles;
  test_current_cycle = 0;
  test_active = true;
  test_side_constraint = side; 
  tState = T_MOVING_OUT;
  
  Serial.println(F("\n══════════════════════════════════════════════════════════════════════"));
  Serial.print(F("⚠ INITIALIZING METROLOGY SUITE: ")); Serial.print(cycles); Serial.println(F(" CYCLES"));
  Serial.print(F("  Constraint: ")); 
  if (side == 1) Serial.println(F("POSITIVE (+)"));
  else if (side == -1) Serial.println(F("NEGATIVE (-)"));
  else Serial.println(F("MIXED"));
  Serial.println(F("══════════════════════════════════════════════════════════════════════"));
  
  triggerRandomMove();
}

void triggerRandomMove() {
  long randAngle = random(15, 170); // Good range to test mechanics
  
  if (test_side_constraint == 0) { if (random(0, 2) == 1) randAngle *= -1; } 
  else if (test_side_constraint == -1) { randAngle = -1 * abs(randAngle); }
  else { randAngle = abs(randAngle); }
  
  Serial.print(F("\n[CYCLE ")); Serial.print(test_current_cycle + 1); 
  Serial.print(F("/")); Serial.print(test_total_cycles);
  Serial.print(F("] Moving to Start Position: ")); Serial.println(randAngle);
  
  tState = T_MOVING_OUT;
  moveToAngle((float)randAngle);
}

void updateTestManager() {
  if (!test_active) return;
  if (state != HOLDING) return; // Wait for motion to stop

  switch (tState) {
    case T_MOVING_OUT:
      delay(100); // Mechanical settling
      
      // --- CAPTURE START CONDITIONS ---
      test_data[test_current_cycle].cycle_num = test_current_cycle + 1;
      test_data[test_current_cycle].start_angle = countToAngle(joint1.gearboxCount);
      test_data[test_current_cycle].start_raw_count = joint1.gearboxCount;
      
      // Determine return direction
      if (test_data[test_current_cycle].start_angle > 0) test_data[test_current_cycle].direction = -1;
      else test_data[test_current_cycle].direction = 1;
      
      Serial.print(F(">> Returning to 0.000° from "));
      Serial.print(test_data[test_current_cycle].start_angle, 2);
      Serial.println(F("°..."));
      
      tState = T_MOVING_HOME;
      test_data[test_current_cycle].target_angle = 0.0000;
      moveToAngle(0.000);
      break;

    case T_MOVING_HOME: { // <--- ADDED BRACE HERE
      // --- CAPTURE END CONDITIONS ---
      test_data[test_current_cycle].end_angle = countToAngle(joint1.gearboxCount);
      test_data[test_current_cycle].end_raw_count = joint1.gearboxCount;
      
      // Calculate Exact Errors
      float err_deg = (position_error_counts / 4096.0) * 360.0;
      test_data[test_current_cycle].final_error_deg = err_deg;
      test_data[test_current_cycle].final_error_counts = position_error_counts;
      
      test_data[test_current_cycle].duration = millis() - move_start_time;
      test_data[test_current_cycle].corrections = correction_attempts;
      
      Serial.print(F(">> ARRIVED. Err: ")); 
      Serial.print(err_deg, 4);
      Serial.println(F("°"));
      
      test_dwell_timer = millis();
      tState = T_MEASURING;
      break; 
    } // <--- ADDED CLOSING BRACE HERE

    case T_MEASURING:
      if (millis() - test_dwell_timer >= DWELL_TIME_MS) {
        test_current_cycle++;
        if (test_current_cycle >= test_total_cycles) { finishTest(); } 
        else { triggerRandomMove(); }
      }
      break;
      
    case T_IDLE: break;
  }
}


void finishTest() {
  test_active = false;
  tState = T_IDLE;
  
  Serial.println(F("\n\n══════════════════════════════════════════════════════════════════════════════════"));
  Serial.println(F("                               METROLOGY REPORT"));
  Serial.println(F("══════════════════════════════════════════════════════════════════════════════════"));
  
  // CSV Friendly Header
  Serial.println(F("Iter,Dir,Start_Deg,Start_Raw,End_Deg,End_Raw,Target,Err_Deg,Err_Cnt,Time_ms,Corr"));
  
  float sum_abs_err = 0; 
  float max_abs_err = 0; 
  float sum_time = 0;
  int perfect_runs = 0;
  
  for (int i = 0; i < test_total_cycles; i++) {
    TestMetric *d = &test_data[i];
    
    Serial.print(d->cycle_num); Serial.print(F(","));
    Serial.print(d->direction > 0 ? "+" : "-"); Serial.print(F(","));
    Serial.print(d->start_angle, 2); Serial.print(F(","));
    Serial.print(d->start_raw_count); Serial.print(F(","));
    Serial.print(d->end_angle, 4); Serial.print(F(","));
    Serial.print(d->end_raw_count); Serial.print(F(","));
    Serial.print(d->target_angle, 1); Serial.print(F(","));
    Serial.print(d->final_error_deg, 5); Serial.print(F(","));
    Serial.print(d->final_error_counts, 2); Serial.print(F(","));
    Serial.print(d->duration); Serial.print(F(","));
    Serial.println(d->corrections);
    
    float abs_e = abs(d->final_error_deg);
    sum_abs_err += abs_e;
    sum_time += d->duration;
    if (abs_e > max_abs_err) max_abs_err = abs_e;
    if (abs_e < 0.001) perfect_runs++;
  }
  
  Serial.println(F("══════════════════════════════════════════════════════════════════════════════════"));
  Serial.print(F("TOTAL CYCLES:     ")); Serial.println(test_total_cycles);
  Serial.print(F("PERFECT (0.00°):  ")); Serial.print(perfect_runs); 
  Serial.print(F(" (")); Serial.print((float)perfect_runs/test_total_cycles*100.0, 1); Serial.println(F("%)"));
  Serial.println(F("----------------------------------------------------------------------------------"));
  Serial.print(F("AVG ERROR (ABS):  ")); Serial.print(sum_abs_err / test_total_cycles, 5); Serial.println(F("°"));
  Serial.print(F("MAX ERROR:        ")); Serial.print(max_abs_err, 5); Serial.println(F("°"));
  Serial.print(F("AVG TIME:         ")); Serial.print(sum_time / test_total_cycles, 0); Serial.println(F(" ms"));
  Serial.println(F("══════════════════════════════════════════════════════════════════════════════════"));
}

