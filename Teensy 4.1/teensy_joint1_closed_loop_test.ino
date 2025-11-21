// ============================================================
// PRECISION JOINT CONTROLLER - OPTIMIZED TRAPEZOIDAL
// ============================================================
// Back to proven trapezoidal with best tuning
// ============================================================

#include <CRC16.h>

// ============================================================
// CONSTANTS
// ============================================================
const float GEAR_RATIO = 50.0;
const uint16_t MICROSTEPS = 8;
const float MOTOR_STEPS_PER_REV = 200.0 * MICROSTEPS;
const float GEARBOX_STEPS_PER_DEGREE = (MOTOR_STEPS_PER_REV / 360.0) * GEAR_RATIO;

// ============================================================
// MOTION PARAMETERS - OPTIMIZED
// ============================================================
const float MAX_VELOCITY = 40.0;           // deg/s (increased!)
const float ACCELERATION = 80.0;           // deg/s² (increased!)
const float DECEL_SAFETY_FACTOR = 2.0;     // Balanced safety

// Thresholds
const float DEADZONE = 0.3;
const float PRECISION_THRESHOLD = 0.2;
const float MAX_ACCEPTABLE_ERROR = 0.5;
const float CORRECTION_SPEED = 1.0;

// Speed limits
const float MIN_SPEED_FAR = 8.0; //3.0;
const float MIN_SPEED_NEAR = 4.0; //1.5;
const float MIN_SPEED_CLOSE = 2.0; //0.8;

const unsigned long SETTLING_DURATION = 100;
const uint8_t MAX_CORRECTION_ATTEMPTS = 2;

// ============================================================
// HARDWARE PINS
// ============================================================
const uint8_t STEP_PIN = 3;
const uint8_t DIR_PIN = 4;
const uint8_t ENA_PIN = 5;

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
  IDLE,
  ACCELERATING,
  CRUISING,
  DECELERATING,
  SETTLING,
  CORRECTING,
  HOLDING
};
ControlState state = HOLDING;

// ============================================================
// VARIABLES
// ============================================================
float target_count = 0.0;
float current_position_count = 0.0;
float current_velocity = 0.0;
float position_error_counts = 0.0;
float last_error_degrees = 0.0;
bool direction_cw = true;
bool move_active = false;

unsigned long settling_start_time = 0;
uint8_t correction_attempts = 0;

volatile uint32_t step_count = 0;
volatile bool stepping_active = false;
IntervalTimer stepTimer;

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
  
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENA_PIN, LOW);
  
  RS485_SERIAL.begin(RS485_BAUD);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  
  Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
  Serial.println(F("║  PRECISION CONTROLLER - OPTIMIZED                 ║"));
  Serial.println(F("╠═══════════════════════════════════════════════════╣"));
  Serial.println(F("║  • Trapezoidal profile with tuned parameters      ║"));
  Serial.println(F("║  • Fast & reliable (40 deg/s)                     ║"));
  Serial.println(F("║  • Settling & correction phases                   ║"));
  Serial.println(F("║  • Target accuracy: ±0.2°                         ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════╝\n"));
  
  Serial.println(F("Configuration:"));
  Serial.print(F("  Max velocity:     ")); Serial.print(MAX_VELOCITY); Serial.println(F(" deg/s"));
  Serial.print(F("  Acceleration:     ")); Serial.print(ACCELERATION); Serial.println(F(" deg/s²"));
  Serial.print(F("  Precision:        ±")); Serial.print(PRECISION_THRESHOLD); Serial.println(F("°"));
  Serial.println();
  
  Serial.println(F("Commands: M <angle> | R <delta> | S | Q | T\n"));
  
  joint1.dataValid = false;
  Serial.print(F("Waiting for encoder..."));
  while (!joint1.dataValid) {
    pollJoint();
    delay(10);
  }
  
  current_position_count = (float)joint1.gearboxCount;
  Serial.println(F(" OK"));
  Serial.print(F("Initial: "));
  Serial.print(countToAngle(current_position_count), 2);
  Serial.println(F("°\n"));
  
  Serial.println(F("Ready! Try: M 180\n"));
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  static unsigned long lastPoll = 0;
  static unsigned long lastLog = 0;
  
  processCommands();
  
  if (millis() - lastPoll >= 50) {
    pollJoint();
    if (joint1.dataValid && move_active) {
      updateControlLoop();
    }
    lastPoll = millis();
  }
  
  if (millis() - lastLog >= 100) {
    if (state != HOLDING && move_active) {
      logData();
    }
    lastLog = millis();
  }
}

// ============================================================
// COMMANDS
// ============================================================
void processCommands() {
  if (!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  
  char cmd_type = cmd.charAt(0);
  
  switch(cmd_type) {
    case 'M': {
      float angle = cmd.substring(2).toFloat();
      moveToAngle(angle);
      break;
    }
    case 'R': {
      float delta = cmd.substring(2).toFloat();
      moveRelative(delta);
      break;
    }
    case 'S': {
      stopMotor();
      state = HOLDING;
      move_active = false;
      Serial.println(F("STOPPED"));
      break;
    }
    case 'Q': {
      pollJoint();
      if (joint1.dataValid) {
        current_position_count = (float)joint1.gearboxCount;
        Serial.print(F("Position: "));
        Serial.print(countToAngle(current_position_count), 3);
        Serial.println(F("°"));
      }
      break;
    }
    case 'T': {
      printStatus();
      break;
    }
  }
}

void moveToAngle(float target_degrees) {
  target_count = angleToCount(target_degrees);
  
  state = IDLE;
  current_velocity = 0.0;
  step_count = 0;
  move_start_time = millis();
  log_start_time = millis();
  move_active = true;
  last_error_degrees = 999.0;
  correction_attempts = 0;
  
  Serial.print(F("\nMoving to "));
  Serial.print(target_degrees, 2);
  Serial.println(F("°\n"));
}

void moveRelative(float delta_degrees) {
  float current_angle = countToAngle(current_position_count);
  moveToAngle(current_angle + delta_degrees);
}

float angleToCount(float angle_degrees) {
  while (angle_degrees < 0.0) angle_degrees += 360.0;
  while (angle_degrees >= 360.0) angle_degrees -= 360.0;
  return (angle_degrees / 360.0) * 4096.0;
}

float countToAngle(float count) {
  while (count < 0.0) count += 4096.0;
  while (count >= 4096.0) count -= 4096.0;
  return (count / 4096.0) * 360.0;
}

// ============================================================
// CONTROL LOOP - PROVEN & OPTIMIZED
// ============================================================
void updateControlLoop() {
  current_position_count = (float)joint1.gearboxCount;
  
  position_error_counts = target_count - current_position_count;
  if (position_error_counts > 2048.0) position_error_counts -= 4096.0;
  if (position_error_counts < -2048.0) position_error_counts += 4096.0;
  
  float error_degrees = (position_error_counts / 4096.0) * 360.0;
  float distance_remaining = abs(error_degrees);
  
  // SETTLING
  if (state == SETTLING) {
    if (millis() - settling_start_time >= SETTLING_DURATION) {
      float final_error = abs(error_degrees);
      
      if (final_error <= PRECISION_THRESHOLD) {
        state = HOLDING;
        move_active = false;
        printMoveSummary();
        return;
      }
      else if (final_error > MAX_ACCEPTABLE_ERROR && correction_attempts < MAX_CORRECTION_ATTEMPTS) {
        correction_attempts++;
        Serial.print(F("→ Correction #"));
        Serial.print(correction_attempts);
        Serial.print(F(": Error = "));
        Serial.print(error_degrees, 2);
        Serial.println(F("°"));
        
        state = CORRECTING;
        current_velocity = CORRECTION_SPEED;
        direction_cw = (position_error_counts > 0.0);
        setDirection(direction_cw);  // CRITICAL: Set direction for correction!
      }
      else {
        state = HOLDING;
        move_active = false;
        if (correction_attempts >= MAX_CORRECTION_ATTEMPTS) {
          Serial.print(F("⚠ Max corrections. Error: "));
          Serial.print(error_degrees, 2);
          Serial.println(F("°"));
        }
        printMoveSummary();
        return;
      }
    }
    return;
  }
  
  // DEADZONE CHECK
  if (abs(error_degrees) < DEADZONE && 
      (state == DECELERATING || state == CORRECTING || state == ACCELERATING || state == CRUISING)) {
    stopMotor();
    state = SETTLING;
    settling_start_time = millis();
    if (correction_attempts == 0) {
      Serial.println(F("→ Entering settling phase..."));
    }
    return;
  }
  
  // OVERSHOOT DETECTION
  if (abs(last_error_degrees) > DEADZONE && abs(last_error_degrees) < 999.0) {
    if ((last_error_degrees > 0 && error_degrees < 0) || 
        (last_error_degrees < 0 && error_degrees > 0)) {
      Serial.print(F("⚠ Overshoot! Error: "));
      Serial.print(error_degrees, 2);
      Serial.println(F("°"));
      stopMotor();
      state = SETTLING;
      settling_start_time = millis();
      return;
    }
  }
  
  last_error_degrees = error_degrees;
  
  // DECEL DISTANCE
  float max_decel_distance = (MAX_VELOCITY * MAX_VELOCITY) / (2.0 * ACCELERATION);
  float decel_start_distance = max_decel_distance * DECEL_SAFETY_FACTOR;
  
  // CORRECTING
  if (state == CORRECTING) {
    current_velocity = CORRECTION_SPEED;
    if (distance_remaining < 1.0) {
      current_velocity = 0.5;
    }
  }
  
  // IDLE
  else if (state == IDLE) {
    direction_cw = (position_error_counts > 0.0);
    setDirection(direction_cw);
    state = ACCELERATING;
  }
  
  // ACCELERATING
  else if (state == ACCELERATING) {
    current_velocity += ACCELERATION * 0.05;  // 4.0 deg/s per cycle
    
    if (current_velocity >= MAX_VELOCITY) {
      current_velocity = MAX_VELOCITY;
      state = CRUISING;
    }
    
    if (distance_remaining <= decel_start_distance) {
      state = DECELERATING;
    }
  }
  
  // CRUISING
  else if (state == CRUISING) {
    current_velocity = MAX_VELOCITY;
    
    if (distance_remaining <= decel_start_distance) {
      state = DECELERATING;
    }
  }
  
  // DECELERATING
  else if (state == DECELERATING) {
    current_velocity -= ACCELERATION * 0.05;  // 4.0 deg/s per cycle
    
    float min_speed;
    if (distance_remaining > 10.0) {
      min_speed = MIN_SPEED_FAR;
    } else if (distance_remaining > 5.0) {
      min_speed = MIN_SPEED_NEAR;
    } else {
      min_speed = MIN_SPEED_CLOSE;
    }
    
    if (current_velocity < min_speed) {
      current_velocity = min_speed;
    }
  }
  
  // Generate steps
  float direction_sign = (position_error_counts > 0.0) ? 1.0 : -1.0;
  generateSteps(current_velocity * direction_sign);
}

// ============================================================
// STEP GENERATION
// ============================================================
void generateSteps(float velocity_deg_per_sec) {
  if (abs(velocity_deg_per_sec) < 0.1) {
    stopMotor();
    return;
  }
  
  float steps_per_sec = abs(velocity_deg_per_sec) * GEARBOX_STEPS_PER_DEGREE;
  float step_interval_us = 1000000.0 / steps_per_sec;
  
  if (!stepping_active) {
    stepping_active = true;
    stepTimer.begin(stepISR, step_interval_us);
  } else {
    stepTimer.update(step_interval_us);
  }
}

void stopMotor() {
  if (stepping_active) {
    stepTimer.end();
    stepping_active = false;
  }
}

void stepISR() {
  digitalWriteFast(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWriteFast(STEP_PIN, LOW);
  step_count++;
}

void setDirection(bool clockwise) {
  digitalWriteFast(DIR_PIN, clockwise ? HIGH : LOW);
  delayMicroseconds(5);
}

// RS485 functions (same as before)
void pollJoint() {
  digitalWrite(RS485_DE_PIN, LOW);
  delayMicroseconds(100);
  
  uint8_t pollMsg[6];
  pollMsg[0] = 0x02;
  pollMsg[1] = JOINT_ID;
  pollMsg[2] = 0x06;
  
  crc.restart();
  crc.add(pollMsg[0]);
  crc.add(pollMsg[1]);
  crc.add(pollMsg[2]);
  uint16_t calcCRC = crc.calc();
  pollMsg[3] = (calcCRC >> 8) & 0xFF;
  pollMsg[4] = calcCRC & 0xFF;
  pollMsg[5] = 0x03;
  
  while (RS485_SERIAL.available()) RS485_SERIAL.read();
  
  digitalWrite(RS485_DE_PIN, HIGH);
  delayMicroseconds(100);
  RS485_SERIAL.write(pollMsg, 6);
  RS485_SERIAL.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_DE_PIN, LOW);
  
  uint8_t response[13];
  if (receiveResponse(response, 13, 200)) {
    parseResponse(response);
  }
}

bool receiveResponse(uint8_t *response, size_t length, unsigned long timeoutMs) {
  unsigned long startTime = millis();
  size_t index = 0;
  bool gotSTX = false;
  
  while (millis() - startTime < timeoutMs && index < length) {
    if (RS485_SERIAL.available()) {
      uint8_t byte = RS485_SERIAL.read();
      if (!gotSTX) {
        if (byte == 0x02) {
          gotSTX = true;
          response[index++] = byte;
        }
      } else {
        response[index++] = byte;
      }
    }
  }
  
  return (index == length && gotSTX && response[length-1] == 0x03);
}

void parseResponse(uint8_t *r) {
  if (r[0] != 0x02 || r[12] != 0x03) return;
  
  crc.restart();
  for (int i = 0; i <= 9; i++) crc.add(r[i]);
  uint16_t calcCRC = crc.calc();
  uint16_t recvCRC = ((uint16_t)r[10] << 8) | r[11];
  
  if (calcCRC != recvCRC) return;
  
  joint1.motorCount = ((uint16_t)r[2] << 8) | r[3];
  joint1.motorRotations = ((int16_t)r[4] << 8) | r[5];
  joint1.gearboxCount = ((uint16_t)r[6] << 8) | r[7];
  joint1.limitSwitch1 = (r[8] & 0x01) != 0;
  joint1.limitSwitch2 = (r[8] & 0x02) != 0;
  joint1.statusByte = r[9];
  joint1.dataValid = true;
  joint1.lastUpdate = millis();
}

// Logging
void logData() {
  if (!joint1.dataValid) return;
  
  float target_angle = countToAngle(target_count);
  float current_angle = countToAngle(current_position_count);
  float error_deg = (position_error_counts / 4096.0) * 360.0;
  
  const char* stateStr;
  switch(state) {
    case IDLE: stateStr = "IDLE"; break;
    case ACCELERATING: stateStr = "ACCEL"; break;
    case CRUISING: stateStr = "CRUISE"; break;
    case DECELERATING: stateStr = "DECEL"; break;
    case SETTLING: stateStr = "SETTLING"; break;
    case CORRECTING: stateStr = "CORRECT"; break;
    case HOLDING: stateStr = "HOLD"; break;
    default: stateStr = "?"; break;
  }
  
  Serial.print(F("["));
  Serial.print((millis() - log_start_time) / 1000.0, 1);
  Serial.print(F("s] T:"));
  Serial.print(target_angle, 2);
  Serial.print(F("° Pos:"));
  Serial.print(current_angle, 3);
  Serial.print(F("° Err:"));
  Serial.print(error_deg, 3);
  Serial.print(F("° V:"));
  Serial.print(current_velocity, 1);
  Serial.print(F(" "));
  Serial.println(stateStr);
}

void printMoveSummary() {
  unsigned long duration = millis() - move_start_time;
  float target_angle = countToAngle(target_count);
  float final_angle = countToAngle(current_position_count);
  float final_error_deg = (position_error_counts / 4096.0) * 360.0;
  
  Serial.println(F("\n╔═══════════════════════════════════════╗"));
  Serial.println(F("║         MOVE COMPLETE                 ║"));
  Serial.println(F("╠═══════════════════════════════════════╣"));
  Serial.print(F("║ Target:      ")); Serial.print(target_angle, 2); Serial.println(F("°"));
  Serial.print(F("║ Final:       ")); Serial.print(final_angle, 3); Serial.println(F("°"));
  Serial.print(F("║ GB Raw:      ")); Serial.println(joint1.gearboxCount);
  Serial.print(F("║ Error:       ")); Serial.print(final_error_deg, 3); Serial.println(F("°"));
  Serial.print(F("║ Corrections: ")); Serial.println(correction_attempts);
  Serial.print(F("║ Duration:    ")); Serial.print(duration / 1000.0, 2); Serial.println(F(" s"));
  Serial.print(F("║ Steps:       ")); Serial.println(step_count);
  Serial.print(F("║ Success:     "));
  Serial.println(abs(final_error_deg) < MAX_ACCEPTABLE_ERROR ? F("YES ✓") : F("NO ✗"));
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
