/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * telemetry_mavlink.c
 *
 * Author: Konstantin Sharlaimov
 */

#include "platform.h"

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_MAVLINK)

#include "common/printf.h"

#include "fc/settings.h"

#include "io/serial.h"

#include "mavlink/mavlink_runtime.h"

#include "telemetry/mavlink.h"
#include "telemetry/telemetry.h"

void initMAVLinkTelemetry(void)
{
    mavlinkRuntimeInit();
}

void handleMAVLinkTelemetry(timeUs_t currentTimeUs)
{
    mavlinkRuntimeHandle(currentTimeUs);
}

void checkMAVLinkTelemetryState(void)
{
    mavlinkRuntimeCheckState();
}

void freeMAVLinkTelemetryPort(void)
{
    mavlinkRuntimeFreePorts();
}

static bool mavlinkSerialPortIsActive(int8_t identifier)
{
    if (identifier == SERIAL_PORT_NONE) {
        return false;
    }
    const serialPortConfig_t *portConfig = serialFindPortConfiguration((serialPortIdentifier_e)identifier);
    return portConfig && (portConfig->functionMask & FUNCTION_TELEMETRY_MAVLINK);
}

static void mavlinkReportSlotSetting(uint8_t slot, unsigned *invalidSettingIndex)
{
    if (invalidSettingIndex) {
        char name[SETTING_MAX_NAME_LENGTH];
        tfp_sprintf(name, "mavlink_port%u_serial", (unsigned)(slot + 1));
        const setting_t *setting = settingFind(name);
        *invalidSettingIndex = setting ? settingGetIndex(setting) : 0;
    }
}

/*
 * Every serial port carrying FUNCTION_TELEMETRY_MAVLINK must be named by exactly
 * one mavlink_portN_serial slot: unnamed it is never opened (a MAVLink receiver
 * on it is a dead control link), named twice one instance is dropped. A slot
 * naming -1 or a port without the function is not an error.
 *
 * On failure invalidSettingIndex names the slot to fix: for a duplicate the
 * later slot, for an unnamed port the first slot not holding an active port.
 */
bool mavlinkPortBindingsValid(unsigned *invalidSettingIndex)
{
    for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
        const serialPortConfig_t *portConfig = &serialConfig()->portConfigs[i];
        if (!(portConfig->functionMask & FUNCTION_TELEMETRY_MAVLINK)) {
            continue;
        }

        uint8_t claims = 0;
        for (uint8_t slot = 0; slot < MAX_MAVLINK_PORTS; slot++) {
            if (telemetryConfig()->mavlink[slot].serial_port == portConfig->identifier && ++claims == 2) {
                mavlinkReportSlotSetting(slot, invalidSettingIndex);
                return false;
            }
        }

        if (claims == 0) {
            uint8_t slot = 0;
            while (slot < MAX_MAVLINK_PORTS - 1 && mavlinkSerialPortIsActive(telemetryConfig()->mavlink[slot].serial_port)) {
                slot++;
            }
            mavlinkReportSlotSetting(slot, invalidSettingIndex);
            return false;
        }
    }

    return true;
}

#endif
