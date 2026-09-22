#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config/parameter_group.h"

typedef enum {
    CONTROL_MODE_PILOT = 0,     // A pilot controls the aircraft through RC channels
    CONTROL_MODE_AUTOPILOT,     // No channel input: telemetry commands and NAV modes only
} controlMode_e;

typedef struct controlModeConfig_s {
    uint8_t controlMode;        // controlMode_e
} controlModeConfig_t;

PG_DECLARE(controlModeConfig_t, controlModeConfig);

bool isAutopilotControlMode(void);

// Manual (pilot-flown) flight modes are available only in Pilot mode
bool controlAllowsManualModes(void);
