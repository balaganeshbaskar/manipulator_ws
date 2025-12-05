#include "RoboticJoint.h"
#include <CRC16.h>

#include "SystemConfig.h"
#include "RS485Communication.h"
#include "ROS2Bridge.h"

// ============ TEST FRAMEWORK (OOP) ============
#include "TestManager.h"
#include "Test_Accuracy.h"
#include "Test_Backlash.h"

using namespace TeensyTimerTool;

/// ============================================================
// WAYPOINT BUFFER SYSTEM WITH LIFECYCLE MANAGEMENT
// ============================================================

enum MotionType {
    MOTION_START = 0,  // First waypoint - clears buffer
    MOTION_VIA = 1,    // Intermediate waypoint
    MOTION_STOP = 2,   // Final waypoint
    MOTION_ABORT = 3   // Emergency stop - clears buffer
};

struct Waypoint {
  float positions[NUM_JOINTS];
  MotionType motionType;
  bool valid;
};

class WaypointBuffer {
private:
  Waypoint buffer[WAYPOINT_BUFFER_SIZE];
  int head = 0;
  int tail = 0;
  int count = 0;
  bool executing = false;
  unsigned long waypointStartTime = 0;

public:
  WaypointBuffer() {
    for (int i = 0; i < WAYPOINT_BUFFER_SIZE; i++) {
      buffer[i].valid = false;
    }
  }

  bool push(float positions[NUM_JOINTS], int motionType) {
    // Handle ABORT separately
    if (motionType == MOTION_ABORT) {
        clear();
        Serial.println(F("ABORTED"));
        Serial.println(F("BUFFER_CLEARED"));
        return true;
    }
    
    // Handle START - clear old buffer
    if (motionType == MOTION_START) {
        clear();
        Serial.println(F("BUFFER_CLEARED"));
    }
    
    // Check if buffer is full
    if (count >= WAYPOINT_BUFFER_SIZE) {
        return false;
    }
    
    // Add waypoint to buffer
    for (int i = 0; i < NUM_JOINTS; i++) {
        buffer[tail].positions[i] = positions[i];
    }
    
    buffer[tail].motionType = (MotionType)motionType;
    buffer[tail].valid = true;
    tail = (tail + 1) % WAYPOINT_BUFFER_SIZE;
    count++;
    
    return true;
  }


  bool hasWaypoints() { return count > 0; }

  Waypoint* getCurrentWaypoint() {
    if (count == 0) return nullptr;
    return &buffer[head];
  }

  void startExecution() {
    if (count > 0 && !executing) {
        executing = true;
        waypointStartTime = millis();
        Serial.println(F("EXECUTING"));
    }
  }

  bool checkCompletion(RoboticJoint joints[], const bool jointPhysical[], ROS2Bridge* bridge) {
    if (!executing || count == 0) return false;
    
    Waypoint* current = getCurrentWaypoint();
    if (!current || !current->valid) return false;
    
    // Timeout check
    if (millis() - waypointStartTime > WAYPOINT_TIMEOUT) {
        Serial.println(F("ERROR:TIMEOUT"));
        pop();
        executing = false;
        return true;
    }
    
    // Check if all physical joints reached target
    bool allReached = true;
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (!jointPhysical[i]) continue;
        if (!joints[i].isDataValid()) {
            allReached = false;
            break;
        }
        
        float currentPos = joints[i].getCurrentAngle();
        float targetPos = current->positions[i];
        float error = abs(targetPos - currentPos);
        if (error > 180.0) error = 360.0 - error;
        
        // Dynamic tolerance - much tighter for final STOP
        float tolerance;
        if (current->motionType == MOTION_STOP) {
            tolerance = 0.08;  // 0.08° = spot on! (was 0.50°)
        } else if (current->motionType == MOTION_START) {
            float decelDist = joints[i].getDecelStartDistance();
            tolerance = decelDist + 5.0;
            tolerance = constrain(tolerance, 3.0, 25.0);
        } else {  // VIA
            float decelDist = joints[i].getDecelStartDistance();
            tolerance = decelDist + 1.0;
            tolerance = constrain(tolerance, 3.0, 15.0);  // Tighter VIA too
        }

        
        if (error > tolerance) {
            allReached = false;
            break;
        }
    }
    
    if (allReached) {
        // Notify ROS2 with waypoint number
        bridge->notifyWaypointReached();
        
        pop();
        
        // Check if this was the last waypoint
        if (count == 0) {
            Serial.println(F("COMPLETE"));
            bridge->resetWaypointCounter();
        }
        
        executing = false;
        return true;
    }
    
    return false;
  }


  void pop() {
    if (count == 0) return;
    buffer[head].valid = false;
    head = (head + 1) % WAYPOINT_BUFFER_SIZE;
    count--;
  }

  int getCount() { return count; }
  bool isExecuting() { return executing; }

  void clear() {
    head = tail = count = 0;
    executing = false;
    for (int i = 0; i < WAYPOINT_BUFFER_SIZE; i++) {
      buffer[i].valid = false;
    }
  }
};

// Instantiate buffer
WaypointBuffer waypointBuffer;

// Control mode
enum ControlMode { MODE_MANUAL, MODE_ROS2_WAYPOINT };
ControlMode currentMode = MODE_MANUAL;

// ============================================================
// HYBRID MODE CONFIGURATION
// (JOINT_PHYSICAL is now in SystemConfig.cpp)
// ============================================================

float simulated_positions[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0};  // degrees
float simulated_targets[NUM_JOINTS]   = {0.0, 0.0, 0.0, 0.0, 0.0};  // degrees
bool  simulated_initialized[NUM_JOINTS] = {false, false, false, false, false};

// ============================================================
// SYSTEM CONFIGURATION
// ============================================================

RoboticJoint joints[NUM_JOINTS] = {
  RoboticJoint(1, 3, 4, 5),     // ID, Step, Dir, Ena
  RoboticJoint(2, 6, 7, 8),
  RoboticJoint(3, 9, 10, 11),
  RoboticJoint(4, 24, 25, 26),
  RoboticJoint(5, 28, 29, 30)
};

// Test Manager Instance
TestManager testManager;

// RS485 module (protocol is in RS485Communication)
RS485Communication rs485(RS485_DE_PIN, RS485_SERIAL, RS485_BAUD);

// ROS2 bridge
ROS2Bridge ros2Bridge(joints, JOINT_PHYSICAL, simulated_positions,
                      simulated_targets, NUM_JOINTS);

// Timing
unsigned long lastPoll         = 0;
unsigned long lastLog          = 0;
unsigned long lastROS2Feedback = 0;

// ============ ROS2 MODE FLAG ============
bool ros2Mode = false;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void processCommands();
void updateSimulatedJoints();
void processROS2Command(String cmd);
void sendROS2Feedback();
void pollAllJoints();
void pollJoint(uint8_t jointID);
void logData();
void updateWaypointExecution();
void parseWaypointCommand(String data);

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(DEBUG_SERIAL_BAUD);
  while (!Serial && millis() < SERIAL_TIMEOUT_MS);

  rs485.begin();          // RS485 init moved into module
  randomSeed(analogRead(14));

  Serial.println(F("\n╔═══════════════════════════════════════════════════╗"));
  Serial.println(F("║  PRECISION JOINT CONTROLLER - ULTRA HIGH RES      ║"));
  Serial.println(F("║  HYBRID MODE: 1 REAL + 4 SIMULATED                ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════╝\n"));

  // Initialize all joints
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (JOINT_PHYSICAL[i]) {
      joints[i].begin();
    }
  }

  // Wait for REAL encoders only
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.print(F("Joint "));
    Serial.print(i + 1);
    Serial.print(F(": "));

    if (JOINT_PHYSICAL[i]) {
      Serial.print(F("REAL - waiting for encoder..."));
      while (!joints[i].isDataValid()) {
        pollJoint(i + 1);
        delay(10);
      }
      Serial.print(F(" OK - Initial: "));
      Serial.print(joints[i].getCurrentAngle(), 4);
      Serial.println(F("°"));
    } else {
      Serial.println(F("SIMULATED - Ready"));
      simulated_initialized[i] = true;
      simulated_positions[i]   = 0.0;
      simulated_targets[i]     = 0.0;
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
  testManager.update();

  // RS485 Polling & Motion Control
  if (millis() - lastPoll >= POLL_INTERVAL_MS) {
    pollAllJoints();

    // Update REAL joints
    for (int i = 0; i < NUM_JOINTS; i++) {
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

  // Waypoint execution in ROS2 waypoint mode
  if (currentMode == MODE_ROS2_WAYPOINT) {
    updateWaypointExecution();

    if (millis() - lastROS2Feedback >= ROS2_FEEDBACK_INTERVAL_MS) {
      sendROS2Feedback();
      lastROS2Feedback = millis();
    }
  }

  // Telemetry Logging (only in manual mode)
  if (!ros2Mode && (millis() - lastLog >= LOG_INTERVAL_MS)) {
    if (joints[ACTIVE_JOINT].getState() != HOLDING ||
        (joints[ACTIVE_JOINT].getState() == HOLDING &&
         joints[ACTIVE_JOINT].isMoveActive())) {
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

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (!JOINT_PHYSICAL[i] && simulated_initialized[i]) {
      float error = simulated_targets[i] - simulated_positions[i];

      // Handle wrap-around
      if (error > 180.0)  error -= 360.0;
      if (error < -180.0) error += 360.0;

      if (abs(error) > 0.01) {
        float step = constrain(error, -MAX_STEP, MAX_STEP);
        simulated_positions[i] += step;

        // Keep in 0-360 range
        if (simulated_positions[i] < 0)          simulated_positions[i] += 360.0;
        if (simulated_positions[i] >= 360.0)     simulated_positions[i] -= 360.0;
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

  // Mode switching
  if (t == '@') {
    if (cmd.substring(1).equals("ROS2")) {
      currentMode = MODE_ROS2_WAYPOINT;
      ros2Mode = true;  // ADD THIS LINE
      waypointBuffer.clear();
      Serial.println(F("=== ROS2 WAYPOINT MODE ==="));
    } else if (cmd.substring(1).equals("MANUAL")) {
      currentMode = MODE_MANUAL;
      ros2Mode = false;  // ADD THIS LINE
      waypointBuffer.clear();
      Serial.println(F("=== MANUAL MODE ==="));
    }
    return;
  }

  // Waypoint command (ROS2 mode only)
  if (t == 'W' && currentMode == MODE_ROS2_WAYPOINT) {
    parseWaypointCommand(cmd.substring(2));
    return;
  }

  // In processCommands(), after mode switching:
  if (t == 'S' && currentMode == MODE_ROS2_WAYPOINT) {
      // Emergency stop in ROS2 mode - send ABORT
      float currentPos[NUM_JOINTS];
      for (int i = 0; i < NUM_JOINTS; i++) {
          if (JOINT_PHYSICAL[i]) {
              currentPos[i] = joints[i].getCurrentAngle();
          } else {
              currentPos[i] = simulated_positions[i];
          }
      }
      waypointBuffer.push(currentPos, MOTION_ABORT);
      return;
  }


  if (t == 'M')       joints[ACTIVE_JOINT].moveTo(cmd.substring(2).toFloat());
  else if (t == 'R') joints[ACTIVE_JOINT].moveRelative(cmd.substring(2).toFloat());
  else if (t == 'S') joints[ACTIVE_JOINT].stop();
  else if (t == 'X') joints[ACTIVE_JOINT].relax();
  else if (t == 'Q') {
    pollAllJoints();
    joints[ACTIVE_JOINT].printStatus();

    // Add joint states in ROS2 format
    Serial.print(F("\nJoint States: "));
    sendROS2Feedback();  // Outputs <0.123,0.456,...>

    // Add buffer status
    Serial.print(F("Buffer: "));
    Serial.print(waypointBuffer.getCount());
    Serial.print(F(" waypoints"));

    if (currentMode == MODE_ROS2_WAYPOINT) {
      Serial.print(F(" | Mode: ROS2"));
    } else {
      Serial.print(F(" | Mode: MANUAL"));
    }
    Serial.println();
  }
  else if (t == 'T') {
    joints[ACTIVE_JOINT].printStatus();
  }
  // Test Commands
  else if (t == 'Z') {
    int firstSpace  = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    int cycles = 5;
    int side   = 1;

    if (firstSpace > 0) {
      if (secondSpace > 0) {
        cycles = cmd.substring(firstSpace + 1, secondSpace).toInt();
        side   = cmd.substring(secondSpace + 1).toInt();
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
    float moveAngle  = 30.0;
    int cycles       = 10;

    if (sp1 > 0 && sp2 > 0 && sp3 > 0) {
      startAngle = cmd.substring(sp1 + 1, sp2).toFloat();
      moveAngle  = cmd.substring(sp2 + 1, sp3).toFloat();
      cycles     = cmd.substring(sp3 + 1).toInt();
    }
    testManager.runTest(new BacklashTest(startAngle, moveAngle, cycles), &joints[ACTIVE_JOINT]);
  }
  else if (t == 'V') {
    float newSpeed = cmd.substring(2).toFloat();
    joints[ACTIVE_JOINT].setMaxVelocity(newSpeed);
    Serial.print("Max Speed set to: ");
    Serial.println(newSpeed);
  }
  else if (t == 'U') {
    // Smooth update (doesn't stop)
    joints[ACTIVE_JOINT].updateTarget(cmd.substring(2).toFloat());
  }
}

// ============================================================
// ROS2 FUNCTIONS
// ============================================================
void processROS2Command(String cmd) {
  // Delegate to bridge (logic identical)
  ros2Bridge.processCommand(cmd);
}

void sendROS2Feedback() {
  // Delegate to bridge (logic identical)
  ros2Bridge.sendFeedback();
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
  int32_t gearboxCount = 0;
  int32_t motorCount   = 0;
  int32_t motorRots    = 0;

  // Use the RS485 module; check if communication was successful
  bool success = rs485.pollJoint(jointID, gearboxCount, motorCount, motorRots);

  if (success && jointID >= 1 && jointID <= NUM_JOINTS) {
    joints[jointID - 1].updateSensorData(gearboxCount, motorCount, motorRots);
  }
  // If communication failed, don't update sensor data - joint will remain invalid
}


// ============================================================
// LOGGING
// ============================================================
void logData() {
  const char* sStr = joints[ACTIVE_JOINT].getStateStr();

  if (joints[ACTIVE_JOINT].getCurrentAngle() != 0.0000) {
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

// ============================================================
// WAYPOINT EXECUTION
// ============================================================
void updateWaypointExecution() {
    static unsigned long lastDebug = 0;
    
    // Debug output every 500ms
    if (millis() - lastDebug > 500) {
        Serial.print(F("[Buffer] Count: "));
        Serial.print(waypointBuffer.getCount());
        Serial.print(F(" | Executing: "));
        Serial.println(waypointBuffer.isExecuting() ? F("YES") : F("NO"));
        lastDebug = millis();
    }
    
    // Check if current waypoint completed
    if (waypointBuffer.isExecuting()) {
        bool completed = waypointBuffer.checkCompletion(joints, JOINT_PHYSICAL, &ros2Bridge);
        if (completed) {
            Serial.println(F("[Buffer] Waypoint completed, checking for next..."));
        } else {
            return;
        }
    }
    
    // Start next waypoint
    if (waypointBuffer.hasWaypoints() && !waypointBuffer.isExecuting()) {
        Waypoint* wp = waypointBuffer.getCurrentWaypoint();
        if (wp && wp->valid) {
            Serial.print(F("[Buffer] Starting waypoint - Target: "));
            Serial.print(wp->positions[0], 1);
            Serial.print(F("° Type: "));
            Serial.println(
                wp->motionType == MOTION_START ? F("START") :
                wp->motionType == MOTION_VIA ? F("VIA") : 
                wp->motionType == MOTION_STOP ? F("STOP") : F("ABORT")
            );
            
            // ✅ FIXED: Smart target update
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (JOINT_PHYSICAL[i]) {
                    if (wp->motionType == MOTION_VIA && joints[i].isMoveActive()) {
                        Serial.print(F(" → updateTarget(")); Serial.print(wp->positions[i],1); Serial.println(F("°)"));
                        joints[i].updateTarget(wp->positions[i]);  // No accel reset!
                    } else {
                        Serial.print(F(" → moveTo(")); Serial.print(wp->positions[i],1); Serial.println(F("°)"));
                        joints[i].moveTo(wp->positions[i]);        // Full reset
                    }
                } else {
                    simulated_targets[i] = wp->positions[i];
                }
            }
            
            waypointBuffer.startExecution();
        }
    }
}



void parseWaypointCommand(String data) {
    float positions[NUM_JOINTS];
    int motionType = 1;  // Default to VIA
    int idx = 0;
    int start = 0;
    int dataLen = data.length();
    
    for (int i = 0; i <= dataLen; i++) {
        if (i == dataLen || data.charAt(i) == ',') {
            if (idx < NUM_JOINTS) {
                positions[idx] = data.substring(start, i).toFloat();
            } else if (idx == NUM_JOINTS) {
                motionType = data.substring(start, i).toInt();
            }
            idx++;
            start = i + 1;
        }
    }
    
    if (idx >= NUM_JOINTS) {
        // Validate motion type
        if (motionType < 0 || motionType > 3) {
            Serial.println(F("ERROR:INVALID_MOTION_TYPE"));
            return;
        }
        
        if (waypointBuffer.push(positions, motionType)) {
            Serial.println(F("BUFFERED"));
            
            // Auto-start execution if enough waypoints buffered
            if (waypointBuffer.getCount() >= MIN_BUFFER_TO_START && 
                !waypointBuffer.isExecuting()) {
                waypointBuffer.startExecution();
            }
        } else {
            Serial.println(F("BUFFER_FULL"));
        }
    } else {
        Serial.println(F("ERROR:PARSE"));
    }
}

