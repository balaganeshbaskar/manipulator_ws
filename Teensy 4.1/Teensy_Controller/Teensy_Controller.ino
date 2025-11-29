#include "RoboticJoint.h"
#include <CRC16.h>

// ============ TEST FRAMEWORK (OOP) ============
#include "TestManager.h"
#include "Test_Accuracy.h"
#include "Test_Backlash.h"

using namespace TeensyTimerTool;

// ============================================================
// SYSTEM CONFIGURATION
// ============================================================
RoboticJoint joints[1] = {
  RoboticJoint(1, 3, 4, 5)  // ID, Step, Dir, Ena
};
const int NUM_JOINTS = 1;
#define ACTIVE_JOINT 0

// Test Manager Instance
TestManager testManager;

// RS485
#define RS485_SERIAL Serial1
const int RS485_DE_PIN = 2;
const uint32_t RS485_BAUD = 57600;
const uint8_t JOINT_ID = 1;
CRC16 crc;

// Timing
unsigned long lastPoll = 0;
unsigned long lastLog = 0;

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // ... [Pins & Random Seed Setup] ...
  RS485_SERIAL.begin(RS485_BAUD);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  randomSeed(analogRead(14));

  Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
  Serial.println(F("║  PRECISION JOINT CONTROLLER - ULTRA HIGH RES      ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════╝\n"));

  // Initialize joints
  for(int i=0; i<NUM_JOINTS; i++) {
    joints[i].begin();
  }

  Serial.print(F("Waiting for encoder..."));
  while (!joints[ACTIVE_JOINT].isDataValid()) {
    pollJoint();
    delay(10);
  }

  // Note: The RoboticJoint class handles the critical init sequence internally
  // when waitForEncoder() or updateSensorData() logic runs

  Serial.println(F(" OK"));
  Serial.print(F("Initial: ")); Serial.print(joints[ACTIVE_JOINT].getCurrentAngle(), 4); Serial.println(F("°"));
  Serial.print(F("Target Set To: ")); Serial.print(joints[ACTIVE_JOINT].getTargetAngle(), 4); Serial.println(F("°"));
  Serial.println(F("Ready! Try: M 150"));
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  processCommands();

  // Update test manager (handles all OOP tests)
  testManager.update();

  // RS485 Polling & Motion Control
  if (millis() - lastPoll >= 20) {
    pollJoint();

    // >>> FIX: MATCH EXACT ORIGINAL LOGIC <<<
    if (joints[ACTIVE_JOINT].isDataValid()) {
        // Original logic: if (move_active || state == HOLDING)
        // In OOP: we use the public getter methods
        if (joints[ACTIVE_JOINT].isMoveActive() || joints[ACTIVE_JOINT].getState() == HOLDING) {
             joints[ACTIVE_JOINT].update();  // Watchdog always runs
        }
    }
    lastPoll = millis();
  }

  // Telemetry Logging
  if (millis() - lastLog >= 100) {
    // Match original: log if moving, OR if we are in holding but drift correcting
    if (joints[ACTIVE_JOINT].getState() != HOLDING || 
       (joints[ACTIVE_JOINT].getState() == HOLDING && joints[ACTIVE_JOINT].isMoveActive())) { 
        logData();
    }
    lastLog = millis();
  }
}

// ============================================================
// COMMAND PARSER
// ============================================================
void processCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); 
  cmd.trim();
  if (cmd.length() == 0) return;

  char t = cmd.charAt(0);

  if (t == 'M') joints[ACTIVE_JOINT].moveTo(cmd.substring(2).toFloat());
  else if (t == 'R') joints[ACTIVE_JOINT].moveRelative(cmd.substring(2).toFloat());
  else if (t == 'S') joints[ACTIVE_JOINT].stop();
  else if (t == 'X') joints[ACTIVE_JOINT].relax();
  else if (t == 'Q') { pollJoint(); joints[ACTIVE_JOINT].printStatus(); }
  else if (t == 'T') joints[ACTIVE_JOINT].printStatus();

  // ============ TEST COMMANDS (OOP Framework) ============

  else if (t == 'Z') {
    // Accuracy Test
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
    // Backlash Test
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
}

// ============================================================
// RS485 COMMUNICATION
// ============================================================
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
  // USE ORIGINAL LOGIC EXACTLY
  crc.restart(); for(int i=0; i<=9; i++) crc.add(r[i]);
  if(crc.calc()!=(((uint16_t)r[10]<<8)|r[11])) return;

  uint16_t motCount=((uint16_t)r[2]<<8)|r[3];
  int16_t motRotations=((int16_t)r[4]<<8)|r[5];
  uint16_t gearboxCount=((uint16_t)r[6]<<8)|r[7];

  joints[ACTIVE_JOINT].updateSensorData(gearboxCount, motCount, motRotations);
}

// ============================================================
// LOGGING
// ============================================================
void logData() {
  // Logic to display High Res position in logs
  // float gb_coarse = countToAngle(current_position_count);
  // float effective_error = (position_error_counts / 4096.0) * 360.0;

  const char* sStr = joints[ACTIVE_JOINT].getStateStr();

  Serial.print(F("[")); 
  Serial.print((millis() - joints[ACTIVE_JOINT].getMoveStartTime()) / 1000.0, 1); // Using getMoveStartTime() to approximate log_start_time
  Serial.print(F("s] T:")); 
  Serial.print(joints[ACTIVE_JOINT].getTargetAngle(), 2);
  Serial.print(F("° Pos:")); 
  Serial.print(joints[ACTIVE_JOINT].getCurrentAngle(), 3); // Still show coarse pos for context
  Serial.print(F("° Err:")); 
  Serial.print(joints[ACTIVE_JOINT].getError(), 4); // Show HIGH RES error
  Serial.print(F("° V:")); 
  Serial.print(joints[ACTIVE_JOINT].getVelocity(), 1);
  Serial.print(F(" ")); 
  Serial.println(sStr);
}
