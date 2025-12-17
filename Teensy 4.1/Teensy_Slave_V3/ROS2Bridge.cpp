#include "ROS2Bridge.h"

// ✅ Constructor matching the Header
ROS2Bridge::ROS2Bridge(RoboticJoint* jointsArray, const bool* physicalArray, 
                       float* simPos, float* simTargets, int numJnts) 
    : numJoints(numJnts) 
{
    joints = jointsArray;
    jointPhysical = physicalArray;
    simulatedPositions = simPos;
    simulatedTargets = simTargets;
    waypointsCompleted = 0;
}

void ROS2Bridge::sendFeedback() {
    // Format: @JOINT_STATE pos1,pos2,pos3,pos4,pos5,vel1,vel2,vel3,vel4,vel5
    // This allows ROS2 to know exactly where the Real AND Virtual joints are.
    
    String msg = "@JOINT_STATE ";

    // 1. POSITIONS (Degrees)
    for (int i = 0; i < numJoints; i++) {
        // In V3, getCurrentAngle() returns sensor data for Real joints
        // and simulated calculated data for Virtual joints.
        float pos = joints[i].getCurrentAngle();
        pos = normalizeAngle(pos);  // ✅ Convert to ROS2 convention
        msg += String(pos, 3);
        if (i < numJoints - 1) msg += ",";
    }

    msg += ","; // Splitter

    // 2. VELOCITIES (Deg/s)
    for (int i = 0; i < numJoints; i++) {
        float vel = joints[i].getVelocity();
        msg += String(vel, 2);
        if (i < numJoints - 1) msg += ",";
    }

    Serial.println(msg);
}

void ROS2Bridge::notifyWaypointReached() {
    waypointsCompleted++;
    // Optional: Send event to ROS if needed, usually Feedback is enough
    // Serial.print("REACHED_WP"); 
    // Serial.println(waypointsCompleted);
}

void ROS2Bridge::resetWaypointCounter() {
    waypointsCompleted = 0;
}

void ROS2Bridge::processCommand(String cmd) {
    if(cmd == "@PING") Serial.println("@PONG");
}
