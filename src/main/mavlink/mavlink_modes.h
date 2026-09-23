#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

typedef struct mavlinkModeSelection_s {
    flightModeForTelemetry_e flightMode;
    uint8_t customMode;
} mavlinkModeSelection_t;

typedef enum {
    MAVLINK_MODE_SELECT_NONE = 0,   // no INAV equivalent for this vehicle/mode
    MAVLINK_MODE_SELECT_BOXES,      // mask holds the mode box(es) to select
    MAVLINK_MODE_SELECT_LANDING,    // run the normal landing path
} mavlinkModeSelectAction_e;

bool mavlinkIsFixedWingVehicle(void);
uint8_t mavlinkGetVehicleType(void);
uint8_t mavlinkGetAutopilotEnum(void);
mavlinkModeSelection_t mavlinkSelectMode(void);
mavlinkModeSelectAction_e mavlinkSelectModeFromCustomMode(uint8_t customMode, boxBitmask_t *mask);

#ifdef USE_MAVLINK_STANDARD_MODES
void mavlinkSendAvailableModesForCurrentMode(void);
bool mavlinkSendAvailableModeForCurrentVehicle(uint8_t requestedIndex);
void mavlinkSendAvailableModesMonitor(void);
void mavlinkSendCurrentMode(const mavlinkModeSelection_t *modeSelection);
#endif
