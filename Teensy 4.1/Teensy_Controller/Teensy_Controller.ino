#include "RoboticJoint.h"
#include <CRC16.h>

// ============ TEST FRAMEWORK (OOP) ============
#include "TestManager.h"
#include "Test_Accuracy.h"
#include "Test_Backlash.h"

using namespace TeensyTimerTool;

// ============================================================
// HYBRID MODE CONFIGURATION
// ============================================================
const bool JOINT_PHYSICAL[5] = {
    true,   // Joint 1: REAL (has encoder + motor)
    false,  // Joint 2: SIMULATED
    false,  // Joint 3: SIMULATED
    false,  // Joint 4: SIMULATED
    false   // Joint 5: SIMULATED
};

float simulated_positions[5] = {0.0, 0.0, 0.0, 0.0, 0.0};  // degrees
float simulated_targets[5] = {0.0, 0.0, 0.0, 0.0, 0.0};     // degrees
bool simulated_initialized[5] = {false, false, false, false, false};

// ============================================================
// SYSTEM CONFIGURATION
// ============================================================
RoboticJoint joints[5] = {
  RoboticJoint(1, 3, 4, 5),     // ID, Step, Dir, Ena
  RoboticJoint(2, 6, 7, 8),
  RoboticJoint(3, 9, 10, 11),
  RoboticJoint(4, 24, 25, 26),
  RoboticJoint(5, 28, 29, 30)
};
const int NUM_JOINTS = 5;
#define ACTIVE_JOINT 0

// Test Manager Instance
TestManager testManager;

// RS485
#define RS485_SERIAL Serial1
const int RS485_DE_PIN = 2;
const uint32_t RS485_BAUD = 57600;
CRC16 crc;

// Timing
unsigned long lastPoll = 0;
unsigned long lastLog = 0;

// ============ ROS2 INTEGRATION ============
bool ros2Mode = false;
unsigned long lastROS2Feedback = 0;

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  RS485_SERIAL.begin(RS485_BAUD);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  randomSeed(analogRead(14));

  Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
  Serial.println(F("║  PRECISION JOINT CONTROLLER - ULTRA HIGH RES      ║"));
  Serial.println(F("║  HYBRID MODE: 1 REAL + 4 SIMULATED                ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════╝\n"));

  // Initialize all joints
  for(int i=0; i<NUM_JOINTS; i++) {
    if (JOINT_PHYSICAL[i]) {
      joints[i].begin();
    }
  }

  // Wait for REAL encoders only
  for(int i=0; i<NUM_JOINTS; i++) {
    Serial.print(F("Joint "));
    Serial.print(i+1);
    Serial.print(F(": "));
    
    if (JOINT_PHYSICAL[i]) {
      Serial.print(F("REAL - waiting for encoder..."));
      while (!joints[i].isDataValid()) {
        pollJoint(i+1);
        delay(10);
      }
      Serial.print(F(" OK - Initial: "));
      Serial.print(joints[i].getCurrentAngle(), 4);
      Serial.println(F("°"));
    } else {
      Serial.println(F("SIMULATED - Ready"));
      simulated_initialized[i] = true;
      simulated_positions[i] = 0.0;
      simulated_targets[i] = 0.0;
    }
  }

  Serial.println(F("\n✓ System Ready"));
  Serial.println(F("Manual: M 150 | ROS2: Auto-detect\n"));
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  processCommands();

  // Update test manager
  testManager.update();

  // RS485 Polling & Motion Control
  if (millis() - lastPoll >= 20) {
    pollAllJoints();

    // Update REAL joints
    for(int i=0; i<NUM_JOINTS; i++) {
      if (JOINT_PHYSICAL[i]) {
        if (joints[i].isDataValid()) {
          if (joints[i].isMoveActive() || joints[i].getState() == HOLDING) {
            joints[i].update();
          }
        }
      }
    }
    
    // Update SIMULATED joints
    updateSimulatedJoints();
    
    lastPoll = millis();
  }

  // ROS2 Feedback (50Hz)
  if (ros2Mode && (millis() - lastROS2Feedback >= 20)) {
    sendROS2Feedback();
    lastROS2Feedback = millis();
  }

  // Telemetry Logging (only in manual mode)
  if (!ros2Mode && (millis() - lastLog >= 100)) {
    if (joints[ACTIVE_JOINT].getState() != HOLDING || 
       (joints[ACTIVE_JOINT].getState() == HOLDING && joints[ACTIVE_JOINT].isMoveActive())) { 
        logData();
    }
    lastLog = millis();
  }
}

// ============================================================
// SIMULATED JOINT UPDATE
// ============================================================
void updateSimulatedJoints() {
  const float MAX_STEP = 2.0;  // Max degrees per update (20ms = 100 deg/s)
  
  for(int i=0; i<NUM_JOINTS; i++) {
    if (!JOINT_PHYSICAL[i] && simulated_initialized[i]) {
      float error = simulated_targets[i] - simulated_positions[i];
      
      // Handle wrap-around
      if (error > 180.0) error -= 360.0;
      if (error < -180.0) error += 360.0;
      
      if (abs(error) > 0.01) {
        float step = constrain(error, -MAX_STEP, MAX_STEP);
        simulated_positions[i] += step;
        
        // Keep in 0-360 range
        if (simulated_positions[i] < 0) simulated_positions[i] += 360.0;
        if (simulated_positions[i] >= 360.0) simulated_positions[i] -= 360.0;
      } else {
        simulated_positions[i] = simulated_targets[i];
      }
    }
  }
}

// ============================================================
// COMMAND PARSER
// ============================================================
void processCommands() {
  if (!Serial.available()) return;
  
  // Check for ROS2 mode
  char firstChar = Serial.peek();
  if (firstChar == '<') {
    ros2Mode = true;
    String cmd = Serial.readStringUntil('\n'); 
    cmd.trim();
    if (cmd.startsWith("<") && cmd.endsWith(">")) {
      processROS2Command(cmd);
    }
    return;
  }
  
  // Manual commands
  String cmd = Serial.readStringUntil('\n'); 
  cmd.trim();
  if (cmd.length() == 0) return;

  char t = cmd.charAt(0);

  if (t == 'M') joints[ACTIVE_JOINT].moveTo(cmd.substring(2).toFloat());
  else if (t == 'R') joints[ACTIVE_JOINT].moveRelative(cmd.substring(2).toFloat());
  else if (t == 'S') joints[ACTIVE_JOINT].stop();
  else if (t == 'X') joints[ACTIVE_JOINT].relax();
  else if (t == 'Q') { pollAllJoints(); joints[ACTIVE_JOINT].printStatus(); }
  else if (t == 'T') joints[ACTIVE_JOINT].printStatus();

  // Test Commands
  else if (t == 'Z') {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    int cycles = 5;
    int side = 1;

    if (firstSpace > 0) {
      if (secondSpace > 0) {
        cycles = cmd.substring(firstSpace + 1, secondSpace).toInt();
        side = cmd.substring(secondSpace + 1).toInt();
      } else {
        cycles = cmd.substring(firstSpace + 1).toInt();
      }
    }
    testManager.runTest(new AccuracyTest(cycles, side), &joints[ACTIVE_JOINT]);
  }

  else if (t == 'B') {
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    int sp3 = cmd.indexOf(' ', sp2 + 1);
    float startAngle = 45.0;
    float moveAngle = 30.0;
    int cycles = 10;

    if (sp1 > 0 && sp2 > 0 && sp3 > 0) {
      startAngle = cmd.substring(sp1 + 1, sp2).toFloat();
      moveAngle = cmd.substring(sp2 + 1, sp3).toFloat();
      cycles = cmd.substring(sp3 + 1).toInt();
    }
    testManager.runTest(new BacklashTest(startAngle, moveAngle, cycles), &joints[ACTIVE_JOINT]);
  }
  else if (t == 'V') {
      float newSpeed = cmd.substring(2).toFloat();
      joints[ACTIVE_JOINT].setMaxVelocity(newSpeed);
      Serial.print("Max Speed set to: ");
      Serial.println(newSpeed);
  }
}

// ============================================================
// ROS2 FUNCTIONS
// ============================================================
void processROS2Command(String cmd) {
  String content = cmd.substring(1, cmd.length() - 1);
  
  Serial.print(F("[Teensy] Command: "));
  Serial.println(cmd);
  
  if (content == "STATUS") {
    sendROS2Feedback();
    return;
  }
  
  float targetRad[NUM_JOINTS];
  int idx = 0;
  int start = 0;
  
  for (int i = 0; i <= content.length(); i++) {
    if (i == content.length() || content.charAt(i) == ',') {
      if (idx < NUM_JOINTS) {
        targetRad[idx] = content.substring(start, i).toFloat();
        idx++;
      }
      start = i + 1;
    }
  }
  
  if (idx == NUM_JOINTS) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      float targetDeg = targetRad[i] * 180.0 / PI;
      
      if (JOINT_PHYSICAL[i]) {
        // REAL joint - send to hardware
        if (joints[i].isDataValid()) {
          joints[i].moveTo(targetDeg);
        }
      } else {
        // SIMULATED joint - update target
        simulated_targets[i] = targetDeg;
      }
    }
    
    Serial.print(F("[Teensy] ✓ Parsed: ["));
    for (int i = 0; i < NUM_JOINTS; i++) {
      Serial.print(targetRad[i], 3);
      if (i < NUM_JOINTS - 1) Serial.print(F(", "));
    }
    Serial.println(F("]"));
  }
}

void sendROS2Feedback() {
  Serial.print("<");
  for (int i = 0; i < NUM_JOINTS; i++) {
    float angleDeg;
    
    if (JOINT_PHYSICAL[i]) {
      // REAL joint - read from encoder
      if (joints[i].isDataValid()) {
        angleDeg = joints[i].getCurrentAngle();
      } else {
        angleDeg = 0.0;
      }
    } else {
      // SIMULATED joint - use simulated position
      angleDeg = simulated_positions[i];
    }
    
    float angleRad = angleDeg * PI / 180.0;
    Serial.print(angleRad, 4);
    
    if (i < NUM_JOINTS - 1) Serial.print(",");
  }
  Serial.println(">");
}

// ============================================================
// RS485 COMMUNICATION
// ============================================================
void pollAllJoints() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (JOINT_PHYSICAL[i]) {
      pollJoint(i + 1);
    }
  }
}

void pollJoint(uint8_t jointID) {
  digitalWrite(RS485_DE_PIN, LOW); delayMicroseconds(100);
  uint8_t msg[6] = {0x02, jointID, 0x06, 0, 0, 0x03};
  crc.restart(); crc.add(msg[0]); crc.add(msg[1]); crc.add(msg[2]);
  uint16_t c = crc.calc(); msg[3]=c>>8; msg[4]=c&0xFF;
  while(RS485_SERIAL.available()) RS485_SERIAL.read();
  digitalWrite(RS485_DE_PIN, HIGH); delayMicroseconds(100);
  RS485_SERIAL.write(msg, 6); RS485_SERIAL.flush();
  delayMicroseconds(100); digitalWrite(RS485_DE_PIN, LOW);
  uint8_t r[13]; if(receiveResponse(r, 13, 200)) parseResponse(r, jointID);
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

void parseResponse(uint8_t *r, uint8_t jointID) {
  crc.restart(); for(int i=0; i<=9; i++) crc.add(r[i]);
  if(crc.calc()!=(((uint16_t)r[10]<<8)|r[11])) return;

  uint16_t motCount=((uint16_t)r[2]<<8)|r[3];
  int16_t motRotations=((int16_t)r[4]<<8)|r[5];
  uint16_t gearboxCount=((uint16_t)r[6]<<8)|r[7];

  if (jointID >= 1 && jointID <= NUM_JOINTS) {
    joints[jointID - 1].updateSensorData(gearboxCount, motCount, motRotations);
  }
}
// ============================================================
// LOGGING
// ============================================================
void logData() {
  const char* sStr = joints[ACTIVE_JOINT].getStateStr();

  if(joints[ACTIVE_JOINT].getCurrentAngle() != 0.0000)
  {
    Serial.print(F("[")); 
    Serial.print((millis() - joints[ACTIVE_JOINT].getMoveStartTime()) / 1000.0, 1);
    Serial.print(F("s] T:")); 
    Serial.print(joints[ACTIVE_JOINT].getTargetAngle(), 2);
    Serial.print(F("° Pos:")); 
    Serial.print(joints[ACTIVE_JOINT].getCurrentAngle(), 3);
    Serial.print(F("° Err:")); 
    Serial.print(joints[ACTIVE_JOINT].getError(), 4);
    Serial.print(F("° V:")); 
    Serial.print(joints[ACTIVE_JOINT].getVelocity(), 1);
    Serial.print(F(" ")); 
    Serial.println(sStr);
  }
}
