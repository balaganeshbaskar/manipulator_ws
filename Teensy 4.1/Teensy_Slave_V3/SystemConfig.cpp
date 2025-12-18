#include "SystemConfig.h"

// ✅ Define the global debug flag here
// Change this to false to disable ALL debug prints
bool DEBUG_ENABLED = false;  // Set to true when debugging needed

// ============================================================
// HYBRID MODE CONFIGURATION
// ============================================================
const bool JOINT_PHYSICAL[5] = {
    true,   // Joint 1: REAL (has encoder + motor)
    false,  // Joint 2: SIMULATED
    false,  // Joint 3: SIMULATED
    false,  // Joint 4: SIMULATED
    false   // Joint 5: SIMULATED
};

SystemMode currentSystemMode = MODE_ROS; // Default to MANUAL for safety/debugging