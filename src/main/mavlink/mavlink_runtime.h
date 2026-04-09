#pragma once

#include <stdbool.h>

#include "common/time.h"

#include "telemetry/telemetry.h"

#define MAVLINK_GROUND_CONTROL_TIMEOUT_MS 5000

void mavlinkRuntimeInit(void);
void mavlinkRuntimeHandle(timeUs_t currentTimeUs);
void mavlinkRuntimeCheckState(void);
void mavlinkRuntimeFreePorts(void);

const mavlinkTelemetryPortConfig_t *mavlinkGetPortConfig(uint8_t portIndex);
const mavlinkTelemetryCommonConfig_t *mavlinkGetCommonConfig(void);
uint8_t mavlinkGetProtocolVersion(void);
void mavlinkSetActivePortContext(uint8_t portIndex);
void mavlinkSendMessage(void);
void mavlinkMarkGroundControlHeartbeat(void);
bool mavlinkHasRecentGroundControlHeartbeat(void);
