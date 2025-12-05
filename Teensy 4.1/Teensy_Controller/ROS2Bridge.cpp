#include "ROS2Bridge.h"

ROS2Bridge::ROS2Bridge(RoboticJoint* jointsArray, const bool* physicalArray,
                       float* simPos, float* simTargets, int numJnts)
    : joints(jointsArray), jointPhysical(physicalArray),
      simulatedPositions(simPos), simulatedTargets(simTargets), 
      numJoints(numJnts), waypointsCompleted(0) {
}

void ROS2Bridge::processCommand(String cmd) {
    String content = cmd.substring(1, cmd.length() - 1);
    
    if (content == "STATUS") {
        sendFeedback();
        return;
    }
    
    // Parse joint target angles in radians: <j1,j2,j3,j4,j5>
    float targetRad[NUM_JOINTS];
    int idx = 0;
    int start = 0;
    int contentLen = content.length();
    
    for (int i = 0; i <= contentLen; i++) {
        if (i == contentLen || content.charAt(i) == ',') {
            if (idx < NUM_JOINTS) {
                targetRad[idx] = content.substring(start, i).toFloat();
                idx++;
            }
            start = i + 1;
        }
    }
    
    if (idx == NUM_JOINTS) {
        // Apply targets to all joints
        for (int i = 0; i < NUM_JOINTS; i++) {
            float targetDeg = targetRad[i] * 180.0 / PI;
            
            // Normalize to -180 to 180
            while (targetDeg > 180.0) targetDeg -= 360.0;
            while (targetDeg < -180.0) targetDeg += 360.0;
            
            if (jointPhysical[i]) {
                if (joints[i].isDataValid()) {
                    joints[i].moveTo(targetDeg);
                }
            } else {
                // Simulated joint - instant response
                simulatedTargets[i] = targetDeg;
                simulatedPositions[i] = targetDeg;  // Instant for tracking
            }
        }
    }
}

void ROS2Bridge::sendFeedback() {
    Serial.print(F("<"));
    for (int i = 0; i < numJoints; i++) {
        float angleDeg;
        
        if (jointPhysical[i]) {
            // REAL joint - read from encoder
            if (joints[i].isDataValid()) {
                angleDeg = joints[i].getCurrentAngle();
            } else {
                angleDeg = 0.0;
            }
        } else {
            // SIMULATED joint - use simulated position
            angleDeg = simulatedPositions[i];
        }
        
        // Normalize to -180 to 180 for ROS2
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        
        float angleRad = angleDeg * PI / 180.0;
        Serial.print(angleRad, 4);
        if (i < numJoints - 1) Serial.print(F(","));
    }
    Serial.println(F(">"));
}

void ROS2Bridge::notifyWaypointReached() {
    waypointsCompleted++;
    Serial.print(F("REACHED_WP"));
    Serial.println(waypointsCompleted);
}

void ROS2Bridge::resetWaypointCounter() {
    waypointsCompleted = 0;
}
