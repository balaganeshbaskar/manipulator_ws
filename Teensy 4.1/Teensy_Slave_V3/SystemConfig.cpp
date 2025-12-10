#include "SystemConfig.h"

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