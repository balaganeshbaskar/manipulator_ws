#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// ============================================================
// SYSTEM-WIDE CONFIGURATION
// ============================================================

// Number of joints
#define NUM_JOINTS 5

// Active joint for manual testing
#define ACTIVE_JOINT 0  // Joint 1

// Physical joint configuration (true = real hardware, false = simulated)
extern const bool JOINT_PHYSICAL[NUM_JOINTS];

// Serial configuration
#define DEBUG_SERIAL_BAUD 115200
#define SERIAL_TIMEOUT_MS 3000

// RS485 configuration
#define RS485_DE_PIN 2
#define RS485_SERIAL Serial1
#define RS485_BAUD 57600

// Timing intervals
#define POLL_INTERVAL_MS 20         // 50Hz joint polling
#define LOG_INTERVAL_MS 100         // 10Hz telemetry logging
#define ROS2_FEEDBACK_INTERVAL_MS 20  // 50Hz ROS2 feedback

// Waypoint buffer configuration
#define WAYPOINT_BUFFER_SIZE 5      // Maximum 5 waypoints in buffer
#define MIN_BUFFER_TO_START 3       // Start motion when 3 waypoints buffered
#define WAYPOINT_TIMEOUT 20000      // 20 seconds timeout per waypoint

enum SystemMode {
    MODE_ROS,    // Outputs @JOINT_STATE (Machine readable)
    MODE_MANUAL  // Outputs Human-readable logs
};

extern SystemMode currentSystemMode; // Global declaration


#endif // SYSTEM_CONFIG_H