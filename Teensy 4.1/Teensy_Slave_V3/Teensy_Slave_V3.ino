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
    // 1. SAFETY WATCHDOG (DETECT DISCONNECTS)
    // ============================================================
    static unsigned long lastSafetyCheck = 0;
    if (millis() - lastSafetyCheck > 100) { // Check every 100ms
        lastSafetyCheck = millis();
        
        for (int i = 0; i < NUM_JOINTS; i++) {
            // Allow 500ms timeout (missed ~10 polls) before killing system
            if (!joints[i].isSensorActive(500)) { 
                if (executor.isRunning()) {
                    Serial.print(F("EMERGENCY: Sensor Timeout on J"));
                    Serial.println(i + 1);
                    executor.abort(); // STOP ALL MOTORS
                    ros2Bridge.resetWaypointCounter();
                }
            }
        }
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

    // ============================================================
    // 4. POLL PHYSICAL SENSORS (50Hz)
    // ============================================================
    static unsigned long lastPoll = 0;
    if (millis() - lastPoll > POLL_INTERVAL_MS) {
        pollNextSensor();
        lastPoll = millis();
    }

    // ============================================================
    // 5. SEND FEEDBACK TO ROS (20Hz)
    // ============================================================
    static unsigned long lastReport = 0;
    if (millis() - lastReport > ROS2_FEEDBACK_INTERVAL_MS) {
        ros2Bridge.sendFeedback();
        lastReport = millis();
    }
}


// ================= COMMAND PARSER =================

void processCommand(String cmd) {
    if (cmd.startsWith("W ")) {
        executor.addWaypoint(cmd.substring(2));
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
