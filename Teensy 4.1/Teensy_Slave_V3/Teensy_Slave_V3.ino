/**
 * ROS2 Industrial Slave Controller V3.1
 * Features: Hybrid Real/Virtual Joints, Status Reporting, Sync Trajectories
 */

#include "SystemConfig.h"
#include "RoboticJoint.h"
#include "RS485Communication.h"
#include "ROS2Bridge.h"
#include "TrajectoryExecutor.h"

// ================= GLOBAL OBJECTS =================
RoboticJoint joints[NUM_JOINTS] = {
    RoboticJoint(1, 3, 4, 5),   // ID, Step, Dir, Ena
    RoboticJoint(2, 6, 7, 8),
    RoboticJoint(3, 9, 10, 11),
    RoboticJoint(4, 24, 25, 26),
    RoboticJoint(5, 28, 29, 30)
};

// ============================================================
// WATCHDOG TIMER CONSTANTS
// ============================================================
#define ROS2_WATCHDOG_TIMEOUT_MS 500   // Emergency stop if no ROS2 command for 500ms
#define WATCHDOG_CHECK_INTERVAL_MS 50  // Check watchdog every 50ms

unsigned long lastRos2CommandTime = 0;
bool watchdogEnabled = false;  // Enable after first ROS2 command received


// Virtual arrays for ROS2Bridge
float simulated_positions[NUM_JOINTS] = {0};
float simulated_targets[NUM_JOINTS] = {0};

RS485Communication rs485(RS485_DE_PIN, RS485_SERIAL, RS485_BAUD);
ROS2Bridge ros2Bridge(joints, JOINT_PHYSICAL, simulated_positions, simulated_targets, NUM_JOINTS);
TrajectoryExecutor executor(joints, NUM_JOINTS);

void setup() {
    Serial.begin(115200);
    
    // 1. Initialize Joints & Apply Physical/Virtual Config
    for(int i=0; i<NUM_JOINTS; i++) {
        joints[i].begin();
        joints[i].setPhysical(JOINT_PHYSICAL[i]);
    }

    // 2. Initialize RS485 (Only if at least one joint is physical)
    rs485.begin();
    delay(100);

    // 3. Initial Sensor Poll
    Serial.println("BOOT: Syncing Sensors...");
    pollAllSensors();
    
    Serial.println("BOOT: READY [V3 HYBRID]");
}

void loop() {

    // ============================================================
    // 1. WATCHDOG TIMER - DETECT ROS2 DISCONNECT
    // ============================================================
    static unsigned long lastWatchdogCheck = 0;
    if (millis() - lastWatchdogCheck >= WATCHDOG_CHECK_INTERVAL_MS) 
    {
        lastWatchdogCheck = millis();
        
        // ✅ Check watchdog ALWAYS when in ROS2 mode (not just during execution)
        if (watchdogEnabled && currentSystemMode == MODE_ROS) 
        {
            unsigned long timeSinceLastCommand = millis() - lastRos2CommandTime;
            
            if (timeSinceLastCommand > ROS2_WATCHDOG_TIMEOUT_MS) 
            {
                // ROS2 connection lost - EMERGENCY STOP
                Serial.println("EMERGENCY: ROS2 CONNECTION LOST");
                Serial.print("Last heartbeat: ");
                Serial.print(timeSinceLastCommand);
                Serial.println("ms ago");
                
                // Abort any active trajectory
                executor.abort();
                
                // Stop all joints
                for (int i = 0; i < NUM_JOINTS; i++) {
                    joints[i].stop();
                }
                
                // Disable watchdog until connection restored
                watchdogEnabled = false;
                
                Serial.println("SYSTEM_HALTED - Reconnect ROS2 to resume");
            }
        }
    }
    

    // ============================================================
    // 1. SAFETY WATCHDOG (DETECT DISCONNECTS)
    // ============================================================
    static unsigned long lastSafetyCheck = 0;
    if (millis() - lastSafetyCheck > 100) 
    { // Check every 100ms
        lastSafetyCheck = millis();
        
        for (int i = 0; i < NUM_JOINTS; i++) 
        {
            // Allow 500ms timeout (missed ~10 polls) before killing system
            if (!joints[i].isSensorActive(500)) 
            { 
                // ✅ ALWAYS trigger emergency, not just during execution
                Serial.print(F("EMERGENCY: Sensor Timeout on J"));
                Serial.println(i + 1);
                Serial.print(F("Last sensor data: "));
                Serial.print(millis() - joints[i].getLastUpdateTime());
                Serial.println(F("ms ago"));
                
                // Abort any active trajectory
                executor.abort();
                
                // Stop ALL motors immediately
                for (int j = 0; j < NUM_JOINTS; j++) 
                {
                    joints[j].stop();
                }
                
                // Reset ROS2 waypoint counter
                ros2Bridge.resetWaypointCounter();
                
                Serial.println(F("SYSTEM_HALTED - Check sensor connections"));
                Serial.println(F("Send RESET to attempt recovery"));
                
                // ✅ Break out of loop (don't check other sensors until reset)
                break;
            }
        }
    }

    // ============================================================
    // 4. POLL PHYSICAL SENSORS (50Hz)
    // ============================================================
    static unsigned long lastPoll = 0;
    if (millis() - lastPoll > POLL_INTERVAL_MS) {
        pollNextSensor();
        lastPoll = millis();
    }

    // ============================================================
    // 2. READ USB COMMANDS
    // ============================================================
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        processCommand(cmd);
    }

    // ============================================================
    // 3. EXECUTE TRAJECTORY (High Freq)
    // ============================================================
    // Only update if no emergency stop is active
    executor.update();

    // ✅ NEW: Also update individual joints (for @HOME and manual moves)
    // if (!executor.isRunning()) {
    //     for (int i = 0; i < NUM_JOINTS; i++) {
    //     joints[i].update();
    //     }
    // }

    // 5. SEND FEEDBACK (Dual Mode)
    if (currentSystemMode == MODE_ROS) {
        static unsigned long lastRosReport = 0;
        if (millis() - lastRosReport > ROS2_FEEDBACK_INTERVAL_MS) {
            ros2Bridge.sendFeedback();
            lastRosReport = millis();
        }
    } 
    else {
        static unsigned long lastManualReport = 0;
        if (millis() - lastManualReport > 200) { 
            printDebugStatus();
            lastManualReport = millis();
        }
    }

}


// NEW FUNCTION: Pretty Print
void printDebugStatus() {
    Serial.print("["); Serial.print(millis()/1000.0, 1); Serial.print("s] ");
    
    // Just print Joint 1 for clarity (or loop if you want all)
    int i = 0; // J1
    Serial.print("Tgt:"); Serial.print(joints[i].getTargetAngle(), 2);
    Serial.print(" Cur:"); Serial.print(joints[i].getCurrentAngle(), 2);
    Serial.print(" Err:"); Serial.print(joints[i].getError(), 2);
    Serial.print(" Vel:"); Serial.print(joints[i].getVelocity(), 1);
    
    // You can access raw counts if you make them public or add a getter
    // Serial.print(" Raw:"); Serial.print(joints[i].getRawGBCount()); 
    
    Serial.println();
}


// ================= COMMAND PARSER =================

void processCommand(String cmd) 
{

    // ✅ NEW: Update watchdog timer on ANY command
    lastRos2CommandTime = millis();
    
    if (!watchdogEnabled && currentSystemMode == MODE_ROS) 
    {
        watchdogEnabled = true;  // Enable watchdog after first ROS2 command
        Serial.println("WATCHDOG_ENABLED");
    }

    // ✅ NEW: Heartbeat command (does nothing except reset watchdog)
    if (cmd == "HEARTBEAT" || cmd == "PING") 
    {
        Serial.println("HEARTBEAT");
        watchdogEnabled = true;
        return;  // Don't process further
    }

    if (cmd.startsWith("W ")) {
        executor.addWaypoint(cmd.substring(2));
    }

    // ✅ SIMPLE HOME COMMAND
    else if (cmd.startsWith("@HOME")) {
        // Format: @HOME or @HOME J1 0.0 10.0
        // Default: Home J1 to 0° at 10°/s
        
        int joint = 0; // Default J1
        float target = 0.0; // Default 0°
        float velocity = 3.0; // Default speed
        
        // Parse optional parameters
        cmd.replace("@HOME", "");
        cmd.trim();
        
        if (cmd.length() > 0) {
        int spaceIdx1 = cmd.indexOf(' ');
        if (spaceIdx1 > 0) {
            String jointStr = cmd.substring(0, spaceIdx1);
            if (jointStr.startsWith("J")) {
            joint = jointStr.substring(1).toInt() - 1;
            }
            
            int spaceIdx2 = cmd.indexOf(' ', spaceIdx1 + 1);
            if (spaceIdx2 > 0) {
            target = cmd.substring(spaceIdx1 + 1, spaceIdx2).toFloat();
            velocity = cmd.substring(spaceIdx2 + 1).toFloat();
            } else {
            target = cmd.substring(spaceIdx1 + 1).toFloat();
            }
        }
        }
        
        // Validate
        if (joint < 0 || joint >= NUM_JOINTS) {
        Serial.println("ERROR: Invalid joint");
        return;
        }
        
        // Stop trajectory if running
        if (executor.isRunning()) {
        executor.abort();
        }
        
        // ✅ Just set target and velocity directly!
        joints[joint].setTrajectoryMode(false);  // Disable trajectory mode
        joints[joint].setMaxVelocity(velocity);
        joints[joint].updateTarget(target);


        Serial.print("HOMING: J");
        Serial.print(joint + 1);
        Serial.print(" -> ");
        Serial.print(target, 1);
        Serial.print("° @ ");
        Serial.print(velocity, 1);
        Serial.println("°/s");
    }
  

    else if (cmd == "@RESET") {
        executor.abort();
        ros2Bridge.resetWaypointCounter();
        Serial.println("BUFFER_CLEARED");
    }
    else if (cmd == "@ABORT") {
        executor.abort();
        Serial.println("ABORTED");
    }
    else if (cmd == "@STATUS") {
        printSystemStatus();
    }
    else if (cmd == "@ROS") {
        currentSystemMode = MODE_ROS;
        Serial.println("MODE: ROS2 (Machine Output)");
    }
    else if (cmd == "@MANUAL") {
        currentSystemMode = MODE_MANUAL;
        Serial.println("MODE: MANUAL (Human Output)");
    }
}

// ================= SENSOR LOGIC =================

void pollAllSensors() {
    Serial.println(F("BOOT: strict sync started..."));
    
    // We need to track which physical joints have successfully reported in
    bool jointSyncStatus[NUM_JOINTS]; 
    for(int i=0; i<NUM_JOINTS; i++) {
        // If it's virtual, mark it as "synced" immediately
        // If it's physical, mark it false (needs data)
        jointSyncStatus[i] = !joints[i].isPhysical();
        
        // Also force virtual joints to sync their "sensor" position to target 0
        if (!joints[i].isPhysical()) {
            joints[i].forceSyncVirtual();
        }
    }

    bool allSynced = false;
    
    // STRICT BLOCKING LOOP
    while (!allSynced) {
        allSynced = true; // Assume success, prove otherwise
        
        for(int i=0; i<NUM_JOINTS; i++) {
            if (jointSyncStatus[i]) continue; // Already synced this one
            
            // Try to poll this specific physical joint
            Serial.print(F("WAIT: Polling J")); Serial.print(i+1); Serial.print("... ");
            
            int32_t gb, mot, rot;
            if(rs485.pollJoint(i+1, gb, mot, rot)) {
                // Success! Update data and mark as synced
                joints[i].updateSensorData((uint16_t)gb, (uint16_t)mot, (int16_t)rot);
                jointSyncStatus[i] = true;
                Serial.print(F("OK! (Pos=")); 
                Serial.print(joints[i].getCurrentAngle()); 
                Serial.println(F(")"));
            } else {
                // Failed, so we are not done yet
                allSynced = false;
                Serial.println(F("NO_REPLY"));
                delay(100); // Wait a bit before retrying to avoid spamming
            }
        }
        
        if (!allSynced) {
            Serial.println(F("BOOT: Waiting for sensors... (Check Power/Cables)"));
            delay(500); // Retry loop delay
        }
    }
    
    Serial.println(F("BOOT: All physical sensors synced!"));
}

void pollNextSensor() {
    static int currentIdx = 0;
    int attempts = 0;
    
    // Find next PHYSICAL joint
    while(!joints[currentIdx].isPhysical() && attempts < NUM_JOINTS) {
        currentIdx = (currentIdx + 1) % NUM_JOINTS;
        attempts++;
    }
    
    // Poll it
    if(joints[currentIdx].isPhysical()) {
        int32_t gb, mot, rot;
        if(rs485.pollJoint(currentIdx+1, gb, mot, rot)) {
            joints[currentIdx].updateSensorData((uint16_t)gb, (uint16_t)mot, (int16_t)rot);
        }
    }
    currentIdx = (currentIdx + 1) % NUM_JOINTS;
}

// ================= STATUS REPORT =================

void printSystemStatus() {
    Serial.println(F("\n=== SYSTEM STATUS ==="));
    Serial.print(F("Executor: ")); 
    Serial.println(executor.isBufferEmpty() ? "IDLE" : "RUNNING");
    
    for(int i=0; i<NUM_JOINTS; i++) {
        Serial.print(F("J")); Serial.print(i+1);
        Serial.print(joints[i].isPhysical() ? F(" [REAL]: ") : F(" [VIRT]: "));
        
        Serial.print(F("Pos=")); Serial.print(joints[i].getCurrentAngle(), 2);
        Serial.print(F("° Tgt=")); Serial.print(joints[i].getTargetAngle(), 2);
        Serial.print(F("° Err=")); Serial.print(joints[i].getError(), 2);
        Serial.print(F("° State=")); Serial.print(joints[i].getStateStr());
        Serial.println();
    }
    Serial.println(F("=====================\n"));
}
// HOMING COMMAND:
// @HOME J1 0.0 1.0

// SAMPLE TEST COMMAND:
// W 45.0,0,0,0,0,5.0,5.0,5.0,5.0,5.0,1

// Tiny move:
// W 1.0,0,0,0,0,5.0,5.0,5.0,5.0,5.0,1

