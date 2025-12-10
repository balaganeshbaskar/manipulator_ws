#ifndef ROS2_BRIDGE_H
#define ROS2_BRIDGE_H

#include <Arduino.h>
#include "SystemConfig.h"
#include "RoboticJoint.h" // ✅ Crucial Include

class ROS2Bridge {
private:
    RoboticJoint* joints;        // ✅ Fixed type
    const bool* jointPhysical;
    float* simulatedPositions;
    float* simulatedTargets;
    const int numJoints;

    int waypointsCompleted;

public:
    // Constructor
    ROS2Bridge(RoboticJoint* jointsArray, const bool* physicalArray, 
               float* simPos, float* simTargets, int numJnts);

    void processCommand(String cmd);
    void sendFeedback();
    void notifyWaypointReached();
    void resetWaypointCounter();
};

#endif // ROS2_BRIDGE_H
