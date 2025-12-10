#include "TrajectoryExecutor.h"

TrajectoryExecutor::TrajectoryExecutor(RoboticJoint* jointArray, int nJoints) {
    joints = jointArray;
    numJoints = nJoints;
    isExecuting = false;
}

void TrajectoryExecutor::abort() {
    head = tail = count = 0;
    isExecuting = false;
    for(int i=0; i<numJoints; i++) joints[i].stop();
}

bool TrajectoryExecutor::isBufferFull() { return count >= WAYPOINT_BUFFER_SIZE; }
bool TrajectoryExecutor::isBufferEmpty() { return count == 0; }

void TrajectoryExecutor::addWaypoint(String csvData) {
    if(isBufferFull()) {
        Serial.println("BUFFER_FULL");
        return;
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

void TrajectoryExecutor::update() {
    // 1. If idle, try to fetch next waypoint
    if(!isExecuting && !isBufferEmpty()) {
        // Start executing new waypoint
        currentTarget = buffer[head];
        head = (head + 1) % WAYPOINT_BUFFER_SIZE;
        count--;
        
        // Push targets to joints
        bool moveStarted = false;
        for(int i=0; i<numJoints; i++) {
            // Set speed limit for this segment
            joints[i].setMaxVelocity(currentTarget.velocities[i]);
            // Set target (Non-blocking)
            joints[i].updateTarget(currentTarget.positions[i]);
            
            if(abs(joints[i].getError()) > 0.1) moveStarted = true;
        }
        
        isExecuting = true;
        if(moveStarted) Serial.println("EXECUTING");
    }

    // 2. Monitor Progress
    if(isExecuting) {
        bool allDone = true;
        for(int i=0; i<numJoints; i++) {
            // Call the joint's update logic (Stepper pulsing is handled by interrupt, this handles state)
            joints[i].update(); 
            
            // Check if joint is "holding" or "settled" close to target
            if(joints[i].getState() != HOLDING) {
                allDone = false;
            }
        }
        
        if(allDone) {
            isExecuting = false; // Ready for next waypoint
            if(currentTarget.motionType == 2) Serial.println("COMPLETE"); // STOP point
        }
    }
}
