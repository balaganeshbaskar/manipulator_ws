#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// =====================================================
// 🔧 GLOBAL DEBUG CONTROL
// =====================================================
// Set to true to enable verbose debugging across ALL files
// Set to false for production (much faster execution)
extern bool DEBUG_ENABLED;

// ============================================================
// SYSTEM-WIDE CONFIGURATION
// ============================================================

// Number of joints
#define NUM_JOINTS 5

// Active joint for manual testing
#define ACTIVE_JOINT 0 // Joint 1

// Physical joint configuration (true = real hardware, false = simulated)
extern const bool JOINT_PHYSICAL[NUM_JOINTS];

// ARM POSITION TOLERANCE
#define FINAL_POSITION_THRESHOLD_DEG 0.1    // Maximum 3.0° (safety cap)

// Serial configuration
#define DEBUG_SERIAL_BAUD 115200
#define SERIAL_TIMEOUT_MS 3000

// RS485 configuration
#define RS485_DE_PIN 2
#define RS485_SERIAL Serial1
#define RS485_BAUD 57600

// Timing intervals
#define POLL_INTERVAL_MS 20 // 50Hz joint polling
#define LOG_INTERVAL_MS 100 // 10Hz telemetry logging
#define ROS2_FEEDBACK_INTERVAL_MS 20 // 50Hz ROS2 feedback

// ============================================================
// ✅ SIMPLIFIED: Dynamic threshold based on trajectory spacing
// ============================================================
#define WAYPOINT_BUFFER_SIZE 50      // Maximum 20 waypoints in buffer
#define MIN_BUFFER_TO_START 10      // was 10 - Start motion when 10 waypoints buffered
#define WAYPOINT_TIMEOUT 20000       // 20 seconds timeout per waypoint

// ✅ Simple average-based threshold
#define WAYPOINT_SWITCH_RATIO 0.4       // Switch at 40% of average waypoint spacing
#define MIN_SWITCH_THRESHOLD_DEG 0.1   // Minimum 0.05° (safety floor)
#define MAX_SWITCH_THRESHOLD_DEG 3.0    // Maximum 3.0° (safety cap)

enum SystemMode {
  MODE_ROS,    // Outputs @JOINT_STATE (Machine readable)
  MODE_MANUAL  // Outputs Human-readable logs
};

extern SystemMode currentSystemMode; // Global declaration

#endif // SYSTEM_CONFIG_H
