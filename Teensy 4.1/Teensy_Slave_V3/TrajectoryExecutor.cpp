#include "TrajectoryExecutor.h"

TrajectoryExecutor::TrajectoryExecutor(RoboticJoint* jointArray, int nJoints) {
  joints = jointArray;
  numJoints = nJoints;
  isExecuting = false;
  motionStarted = false;
  
  // Initialize all thresholds to minimum
  for(int i=0; i<NUM_JOINTS; i++) {
    dynamicThresholds[i] = MIN_SWITCH_THRESHOLD_DEG;
  }
}

void TrajectoryExecutor::abort() {
  head = tail = count = 0;
  isExecuting = false;
  motionStarted = false;
  
  for(int i=0; i<numJoints; i++) {
    joints[i].setTrajectoryMode(false);
    joints[i].setFinalWaypoint(false);  // ✅ NEW: Reset flag
    joints[i].stop();
  }
}

bool TrajectoryExecutor::isBufferFull() { return count >= WAYPOINT_BUFFER_SIZE; }
bool TrajectoryExecutor::isBufferEmpty() { return count == 0; }

// ============================================================
// ✅ CORRECTED: Calculate per-joint average spacing
// ============================================================
void TrajectoryExecutor::calculateAverageSpacing() {
  if (count < 2) {
    // Not enough waypoints - use minimum threshold for all joints
    for(int i=0; i<NUM_JOINTS; i++) {
      dynamicThresholds[i] = MIN_SWITCH_THRESHOLD_DEG;
    }
    return;
  }
  
  // Track total distance per joint
  float jointTotalDistance[NUM_JOINTS] = {0};
  int transitionCount = count - 1; // Number of waypoint-to-waypoint transitions
  
  // Loop through all buffered waypoints and calculate distances
  for(int wpIdx = 0; wpIdx < transitionCount; wpIdx++) {
    int bufferIdx1 = (head + wpIdx) % WAYPOINT_BUFFER_SIZE;
    int bufferIdx2 = (head + wpIdx + 1) % WAYPOINT_BUFFER_SIZE;
    
    WaypointV3 wp1 = buffer[bufferIdx1];
    WaypointV3 wp2 = buffer[bufferIdx2];
    
    // Calculate distance for each joint
    for(int j=0; j<numJoints; j++) {
      float diff = abs(wp2.positions[j] - wp1.positions[j]);
      
      // Handle wrap-around (e.g., 359° to 1°)
      if (diff > 180.0f) diff = 360.0f - diff;
      
      jointTotalDistance[j] += diff;
    }
  }
  
  // Calculate per-joint average and threshold
  Serial.println("Per-joint dynamic thresholds:");
  for(int j=0; j<numJoints; j++) {
    float avgSpacing = jointTotalDistance[j] / transitionCount;
    
    // Apply ratio to get switching threshold
    dynamicThresholds[j] = avgSpacing * WAYPOINT_SWITCH_RATIO;
    
    // Clamp between min and max
    if (dynamicThresholds[j] < MIN_SWITCH_THRESHOLD_DEG) {
      dynamicThresholds[j] = MIN_SWITCH_THRESHOLD_DEG;
    }
    if (dynamicThresholds[j] > MAX_SWITCH_THRESHOLD_DEG) {
      dynamicThresholds[j] = MAX_SWITCH_THRESHOLD_DEG;
    }
    
    // Debug output
    Serial.print("  J");
    Serial.print(j + 1);
    Serial.print(": avg=");
    Serial.print(avgSpacing, 3);
    Serial.print("° → threshold=");
    Serial.print(dynamicThresholds[j], 3);
    Serial.println("°");
  }
}

void TrajectoryExecutor::addWaypoint(String csvData) {
  if(isBufferFull()) {
    Serial.println("BUFFER_FULL");
    return;
  }

  if(isBufferEmpty())
  {
    for(int i=0; i<numJoints; i++) 
    {
      joints[i].setFinalWaypoint(false);
    }
  }

  // Parse CSV: p1...p5, v1...v5, type
  WaypointV3 wp;
  int strIdx = 0;
  
  // Parse Positions
  for(int i=0; i<numJoints; i++) {
    int nextComma = csvData.indexOf(',', strIdx);
    if(nextComma == -1) break;
    wp.positions[i] = csvData.substring(strIdx, nextComma).toFloat();
    strIdx = nextComma + 1;
  }
  
  // Parse Velocities
  for(int i=0; i<numJoints; i++) {
    int nextComma = csvData.indexOf(',', strIdx);
    if(nextComma == -1) break;
    wp.velocities[i] = csvData.substring(strIdx, nextComma).toFloat();
    strIdx = nextComma + 1;
  }
  
  // Parse Type
  wp.motionType = csvData.substring(strIdx).toInt();

  // Add to buffer
  buffer[tail] = wp;
  tail = (tail + 1) % WAYPOINT_BUFFER_SIZE;
  count++;
  
  Serial.println("BUFFERED");
}

// ============================================================
// ✅ CORRECTED: Use per-joint thresholds
// ============================================================
void TrajectoryExecutor::update() {
  
  // ============================================================
  // PHASE 1: START TRAJECTORY (Wait for minimum buffer)
  // ============================================================
  if (!isExecuting && !motionStarted) {
    if (count >= MIN_BUFFER_TO_START) {
      Serial.print("TRAJECTORY_START: ");
      Serial.print(count);
      Serial.println(" waypoints buffered");
      
      // ✅ Calculate per-joint thresholds from buffered waypoints
      calculateAverageSpacing();
      
      // Pop first waypoint
      currentTarget = buffer[head];
      head = (head + 1) % WAYPOINT_BUFFER_SIZE;
      count--;

      //FOR DEBUGGING
      for(int i=0; i<numJoints; i++) 
      {
        Serial.print("J: ");
        Serial.print(i+1);
        Serial.print(", Pos: ");
        Serial.print(currentTarget.positions[i]);
        Serial.print(", Vel: ");
        Serial.println(currentTarget.velocities[i]);
      }
      
      // Enable trajectory mode on all joints
      for(int i=0; i<numJoints; i++) 
      {
        joints[i].setMaxVelocity(currentTarget.velocities[i]);
        joints[i].updateTarget(currentTarget.positions[i]);
        if (!joints[i].isPhysical()) 
        {
          joints[i].forceSyncVirtual();   // make virtual joint instantly “reach” target
        }
        else
        {
          joints[i].setTrajectoryMode(true);
        }
      }
      
      isExecuting = true;
      motionStarted = true;
      // Serial.println("EXECUTING");
    }
    return;
  }

  // ============================================================
  // PHASE 2: CONTINUOUS TRAJECTORY EXECUTION
  // ============================================================
  if (isExecuting) {
    
    // ✅ NEW: Periodic status reporting during execution (every 500ms)
    static unsigned long lastStatusReport = 0;
    if (millis() - lastStatusReport > 500) {
        Serial.println("EXECUTING: ");

        //FOR DEBUGGING
        for(int i=0; i<numJoints; i++) 
        {
          Serial.print("J: ");
          Serial.print(i+1);
          Serial.print(", Pos: ");
          Serial.print(currentTarget.positions[i]);
          Serial.print(", Vel: ");
          Serial.println(currentTarget.velocities[i]);
        }

        Serial.print(count);
        Serial.println(" waypoints remaining");
      lastStatusReport = millis();
    }
    
    // -----------------------------------------------------
    // STEP 1: CHECK IF NEAR TARGET (per-joint thresholds)
    // -----------------------------------------------------
    bool nearTarget = true;
    
    for(int i=0; i<numJoints; i++) 
    {
      float error = abs(joints[i].getError());
      
      // ✅ Each joint uses its own threshold
      if (error > dynamicThresholds[i]) 
      {
        nearTarget = false;
      }
    }
    
    // -----------------------------------------------------
    // STEP 2: PREEMPTIVE WAYPOINT SWITCHING
    // -----------------------------------------------------
    if (nearTarget && !isBufferEmpty()) {
      // Load next waypoint BEFORE joints reach current target
      currentTarget = buffer[head];
      head = (head + 1) % WAYPOINT_BUFFER_SIZE;
      count--;
      
      // ✅ NEW: Report waypoint consumption
      Serial.print("SWITCHED_WP: ");
      Serial.print(count);
      Serial.println(" remaining");

      for(int i=0; i<numJoints; i++) 
      {
        joints[i].setMaxVelocity(currentTarget.velocities[i]);
        joints[i].updateTarget(currentTarget.positions[i]);

        if (!joints[i].isPhysical()) 
        {
          joints[i].forceSyncVirtual();   // make virtual joint instantly “reach” target
        }
        else
        {
          joints[i].setFinalWaypoint(false);  // ✅ NEW: Not final yet
        }
      }

    }
    
    // -----------------------------------------------------
    // STEP 3: UPDATE JOINT PHYSICS (AFTER target check!)
    // -----------------------------------------------------
    // for(int i=0; i<numJoints; i++) {
    //   joints[i].update();
    // }

    for (int i = 0; i < numJoints; i++) 
    {
      if (joints[i].isPhysical()) joints[i].update();
    }
    
    // -----------------------------------------------------
    // STEP 4: TRAJECTORY COMPLETION CHECK
    // -----------------------------------------------------
    if (isBufferEmpty() && nearTarget) {

      // ✅ NEW: Mark all joints as on final waypoint
      for(int i=0; i<numJoints; i++)
      {
        joints[i].setFinalWaypoint(true);
      }

      bool allSettled = true;
      
      for(int i=0; i<numJoints; i++)
      {
        // Use minimum threshold for final settling
        if (abs(joints[i].getError()) > FINAL_POSITION_THRESHOLD_DEG)
        {
          allSettled = false;
          break;
        }

      }
      
      if (allSettled) 
      { 
        // Trajectory complete - disable trajectory mode
        for(int i=0; i<numJoints; i++) 
        {
          joints[i].setTrajectoryMode(false);
          joints[i].stop();
          joints[i].setFinalWaypoint(false);

          // ✅ Set low holding velocity (gentle correction)
          joints[i].setMaxVelocity(5.0f);  // Slow holding speed
        }
        
        isExecuting = false;
        motionStarted = false;
        
        if (currentTarget.motionType == 2) 
        {
          Serial.println("COMPLETE");
        }
      }
    }
  }

}
