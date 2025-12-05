#ifndef ROS2_BRIDGE_H
#define ROS2_BRIDGE_H

#include <Arduino.h>
#include "SystemConfig.h"
#include "RoboticJoint.h"

// ============================================================
// ROS2 COMMUNICATION BRIDGE
// Handles ROS2 protocol with waypoint lifecycle management
// ============================================================

class ROS2Bridge {
private:
    RoboticJoint* joints;
    const bool* jointPhysical;
    float* simulatedPositions;
    float* simulatedTargets;
    const int numJoints;
    
    // Waypoint tracking for REACHED_WPx feedback
    int waypointsCompleted;

public:
    ROS2Bridge(RoboticJoint* jointsArray, const bool* physicalArray,
               float* simPos, float* simTargets, int numJnts);
    
    void processCommand(String cmd);
    void sendFeedback();
    
    // NEW: Waypoint completion notification
    void notifyWaypointReached();
    
    // NEW: Reset waypoint counter
    void resetWaypointCounter();
};

#endif // ROS2_BRIDGE_H
