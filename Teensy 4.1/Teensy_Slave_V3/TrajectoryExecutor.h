#ifndef TRAJECTORY_EXECUTOR_H
#define TRAJECTORY_EXECUTOR_H

#include <Arduino.h>
#include "SystemConfig.h"
#include "RoboticJoint.h"

struct WaypointV3 {
  float positions[NUM_JOINTS];
  float velocities[NUM_JOINTS];
  int motionType; // 0=START, 1=VIA, 2=STOP
};

class TrajectoryExecutor {
private:
  RoboticJoint* joints;
  int numJoints;

  // Circular Buffer
  WaypointV3 buffer[WAYPOINT_BUFFER_SIZE];
  int head = 0;
  int tail = 0;
  int count = 0;

  bool isExecuting;
  WaypointV3 currentTarget;
  bool motionStarted = false;
  
  // ✅ CORRECTED: Per-joint dynamic thresholds
  float dynamicThresholds[NUM_JOINTS];
  
  // ✅ Calculate average spacing per joint
  void calculateAverageSpacing();

public:
  TrajectoryExecutor(RoboticJoint* jointArray, int nJoints);
  void addWaypoint(String csvData);
  void update();
  void abort();
  
  bool isBufferFull();
  bool isBufferEmpty();
  bool isRunning() { return isExecuting; }
  int getBufferCount() { return count; }
};

#endif
