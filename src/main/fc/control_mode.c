#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/control_mode.h"

#include "config/parameter_group_ids.h"
#include "fc/settings.h"

PG_REGISTER_WITH_RESET_TEMPLATE(controlModeConfig_t, controlModeConfig, PG_CONTROL_MODE_CONFIG, 0);

PG_RESET_TEMPLATE(controlModeConfig_t, controlModeConfig,
    .controlMode = SETTING_CONTROL_MODE_DEFAULT,
);

bool isAutopilotControlMode(void)
{
    return controlModeConfig()->controlMode == CONTROL_MODE_AUTOPILOT;
}

bool controlAllowsManualModes(void)
{
    return !isAutopilotControlMode();
}
