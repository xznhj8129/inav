/*
 * This file is part of INAV.
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
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERIAL_GIMBAL

#include <common/crc.h>
#include <common/utils.h>
#include <common/maths.h>
#include <build/debug.h>

#include <drivers/gimbal_common.h>
#include <drivers/headtracker_common.h>
#include <drivers/serial.h>
#include <drivers/time.h>

#include <io/gimbal_serial.h>
#include <io/serial.h>

#include <rx/rx.h>
#include <fc/rc_modes.h>

#include <flight/imu.h>
#include <navigation/navigation.h>

#include <config/parameter_group_ids.h>

#include "settings_generated.h"

PG_REGISTER_WITH_RESET_TEMPLATE(gimbalSerialConfig_t, gimbalSerialConfig, PG_GIMBAL_SERIAL_CONFIG, 0);

PG_RESET_TEMPLATE(gimbalSerialConfig_t, gimbalSerialConfig,
    .singleUart = SETTING_GIMBAL_SERIAL_SINGLE_UART_DEFAULT
);

STATIC_ASSERT(sizeof(gimbalHtkAttitudePkt_t) == 10, gimbalHtkAttitudePkt_t_size_not_10);

#define GIMBAL_SERIAL_BUFFER_SIZE 512

#ifndef GIMBAL_UNIT_TEST
static volatile uint8_t txBuffer[GIMBAL_SERIAL_BUFFER_SIZE];

#if defined(USE_HEADTRACKER) && defined(USE_HEADTRACKER_SERIAL)
static gimbalSerialHtrkState_t headTrackerState = { 
    .payloadSize = 0,
    .attitude = {},
    .state = WAITING_HDR1,
};
static serialPort_t *headTrackerPort = NULL;
#endif

#endif

static serialPort_t *gimbalPort = NULL;

static bool gimbalPoiUpdateAttitude(const gimbalConfig_t *cfg, gimbalHtkAttitudePkt_t *gimbalAttitude)
{
    const radar_pois_t *poi = &radar_pois[0];

    if (poi->gps.lat == 0 || poi->gps.lon == 0 || poi->state >= 2) {
        return false;
    }

    fpVector3_t poiLocal;
    if (!geoConvertGeodeticToLocalOrigin(&poiLocal, &poi->gps, GEO_ALT_ABSOLUTE)) {
        return false;
    }

    const float craftX = getEstimatedActualPosition(X);
    const float craftY = getEstimatedActualPosition(Y);
    const float craftZ = getEstimatedActualPosition(Z);

    const float deltaX = poiLocal.x - craftX;
    const float deltaY = poiLocal.y - craftY;
    const float deltaZ = poiLocal.z - craftZ;

    if (deltaX == 0.0f && deltaY == 0.0f && deltaZ == 0.0f) {
        return false;
    }

    const int32_t targetBearingCd = wrap_36000(RADIANS_TO_CENTIDEGREES(atan2_approx(deltaY, deltaX)));
    const int32_t craftHeadingCd = DECIDEGREES_TO_CENTIDEGREES(attitude.values.yaw);
    const int32_t relativeBearingCd = wrap_18000(targetBearingCd - craftHeadingCd);
    const float panDegrees = CENTIDEGREES_TO_DEGREES(relativeBearingCd);

    const float horizontalDistance = sqrtf(sq(deltaX) + sq(deltaY));
    const float tiltDegrees = RADIANS_TO_DEGREES(atan2_approx(deltaZ, horizontalDistance));

    int panPWM = lrintf(scaleRangef(constrainf(panDegrees, -180.0f, 180.0f), -180.0f, 180.0f, PWM_RANGE_MIN, PWM_RANGE_MAX));
    panPWM = constrain(panPWM + cfg->panTrim, PWM_RANGE_MIN, PWM_RANGE_MAX);

    int tiltPWM = lrintf(scaleRangef(constrainf(tiltDegrees, -90.0f, 90.0f), -90.0f, 90.0f, PWM_RANGE_MIN, PWM_RANGE_MAX));
    tiltPWM = constrain(tiltPWM + cfg->tiltTrim, PWM_RANGE_MIN, PWM_RANGE_MAX);

    gimbalAttitude->pan = gimbal_scale12(PWM_RANGE_MIN, PWM_RANGE_MAX, panPWM);
    gimbalAttitude->tilt = gimbal_scale12(PWM_RANGE_MIN, PWM_RANGE_MAX, tiltPWM);
    gimbalAttitude->mode |= (GIMBAL_MODE_PAN_LOCK | GIMBAL_MODE_TILT_LOCK);

    return true;
}

gimbalVTable_t gimbalSerialVTable = {
    .process = gimbalSerialProcess,
    .getDeviceType = gimbalSerialGetDeviceType,
    .isReady = gimbalSerialIsReady,
    .hasHeadTracker = gimbalSerialHasHeadTracker,

};

static gimbalDevice_t serialGimbalDevice = {
    .vTable = &gimbalSerialVTable,
    .currentPanPWM = PWM_RANGE_MIDDLE
};

#if (defined(USE_HEADTRACKER) && defined(USE_HEADTRACKER_SERIAL))

static headTrackerVTable_t headTrackerVTable = {
    .process = headtrackerSerialProcess,
    .getDeviceType = headtrackerSerialGetDeviceType,
};


headTrackerDevice_t headTrackerDevice = {
    .vTable = &headTrackerVTable,
    .pan = 0,
    .tilt = 0,
    .roll = 0,
    .expires = 0
};

#endif


gimbalDevType_e gimbalSerialGetDeviceType(const gimbalDevice_t *gimbalDevice)
{
    UNUSED(gimbalDevice);
    return GIMBAL_DEV_SERIAL;
}

bool gimbalSerialIsReady(const gimbalDevice_t *gimbalDevice)
{
    return gimbalPort != NULL && gimbalDevice->vTable != NULL;
}

bool gimbalSerialHasHeadTracker(const gimbalDevice_t *gimbalDevice)
{
    UNUSED(gimbalDevice);

    headTrackerDevice_t *htrk = headTrackerCommonDevice();
    return htrk != NULL && headTrackerCommonIsReady(htrk);
}

bool gimbalSerialInit(void)
{
    if (gimbalSerialDetect()) {
        SD(fprintf(stderr, "Setting gimbal device\n"));
        gimbalCommonSetDevice(&serialGimbalDevice);
        return true;
    }

    return false;
}

#ifdef GIMBAL_UNIT_TEST
bool gimbalSerialDetect(void)
{
    return false;
}
#else
bool gimbalSerialDetect(void)
{
    
    SD(fprintf(stderr, "[GIMBAL]: serial Detect...\n"));
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_GIMBAL);
    bool singleUart = gimbalSerialConfig()->singleUart;


    if (portConfig) {
        SD(fprintf(stderr, "[GIMBAL]: found port...\n"));
#if defined(USE_HEADTRACKER) && defined(USE_HEADTRACKER_SERIAL)
        gimbalPort = openSerialPort(portConfig->identifier, FUNCTION_GIMBAL, singleUart ? gimbalSerialHeadTrackerReceive : NULL, singleUart ? &headTrackerState :  NULL,
                baudRates[portConfig->peripheral_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
#else
        UNUSED(singleUart);
        gimbalPort = openSerialPort(portConfig->identifier, FUNCTION_GIMBAL, NULL, NULL,
                baudRates[portConfig->peripheral_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
#endif

        if (gimbalPort) {
            SD(fprintf(stderr, "[GIMBAL]: port open!\n"));
            gimbalPort->txBuffer = txBuffer;
            gimbalPort->txBufferSize = GIMBAL_SERIAL_BUFFER_SIZE;
            gimbalPort->txBufferTail = 0;
            gimbalPort->txBufferHead = 0;
        } else {
            SD(fprintf(stderr, "[GIMBAL]: port NOT open!\n"));
            return false;
        }
    }

    SD(fprintf(stderr, "[GIMBAL]: gimbalPort: %p\n", gimbalPort));
    return gimbalPort;
}
#endif

#ifdef GIMBAL_UNIT_TEST
void gimbalSerialProcess(gimbalDevice_t *gimbalDevice, timeUs_t currentTime)
{
    UNUSED(gimbalDevice);
    UNUSED(currentTime);
}
#else
void gimbalSerialProcess(gimbalDevice_t *gimbalDevice, timeUs_t currentTime)
{
    UNUSED(currentTime);

    if (!gimbalSerialIsReady(gimbalDevice)) {
        SD(fprintf(stderr, "[GIMBAL] gimbal not ready...\n"));
        return;
    }

    gimbalHtkAttitudePkt_t gimbalAttitude = {
        .sync = {HTKATTITUDE_SYNC0, HTKATTITUDE_SYNC1},
        .mode = GIMBAL_MODE_DEFAULT,
        .pan = 0,
        .tilt = 0,
        .roll = 0
    };

    const gimbalConfig_t *cfg = gimbalConfig();

    int panPWM = PWM_RANGE_MIDDLE + cfg->panTrim;
    int tiltPWM = PWM_RANGE_MIDDLE + cfg->tiltTrim;
    int rollPWM = PWM_RANGE_MIDDLE + cfg->rollTrim;

    if (IS_RC_MODE_ACTIVE(BOXGIMBALTLOCK)) {
        gimbalAttitude.mode |= GIMBAL_MODE_TILT_LOCK;
    }

    if (IS_RC_MODE_ACTIVE(BOXGIMBALRLOCK)) {
        gimbalAttitude.mode |= GIMBAL_MODE_ROLL_LOCK;
    }

    const bool centerActive = IS_RC_MODE_ACTIVE(BOXGIMBALCENTER);
    const bool headTrackerActive = IS_RC_MODE_ACTIVE(BOXGIMBALHTRK);
    const bool poiRequested = IS_RC_MODE_ACTIVE(BOXGIMBALPOI);

    if (centerActive || (headTrackerActive && !poiRequested)) {
        gimbalAttitude.mode = GIMBAL_MODE_FOLLOW;
    }

    if (rxAreFlightChannelsValid() && !centerActive) {
        if (cfg->panChannel > 0) {
            panPWM = rxGetChannelValue(cfg->panChannel - 1) + cfg->panTrim;
            panPWM = constrain(panPWM, PWM_RANGE_MIN, PWM_RANGE_MAX);
        }

        if (cfg->tiltChannel > 0) {
            tiltPWM = rxGetChannelValue(cfg->tiltChannel - 1) + cfg->tiltTrim;
            tiltPWM = constrain(tiltPWM, PWM_RANGE_MIN, PWM_RANGE_MAX);
        }

        if (cfg->rollChannel > 0) {
            rollPWM = rxGetChannelValue(cfg->rollChannel - 1) + cfg->rollTrim;
            rollPWM = constrain(rollPWM, PWM_RANGE_MIN, PWM_RANGE_MAX);
        }
    }

    const int16_t manualPan = gimbal_scale12(PWM_RANGE_MIN, PWM_RANGE_MAX, panPWM);
    const int16_t manualTilt = gimbal_scale12(PWM_RANGE_MIN, PWM_RANGE_MAX, tiltPWM);
    const int16_t manualRoll = gimbal_scale12(PWM_RANGE_MIN, PWM_RANGE_MAX, rollPWM);

    bool poiTrackingActive = false;
    if (!centerActive && poiRequested) {
        poiTrackingActive = gimbalPoiUpdateAttitude(cfg, &gimbalAttitude);
        if (poiTrackingActive) {
            gimbalAttitude.roll = manualRoll;
            DEBUG_SET(DEBUG_HEADTRACKING, 4, 3);
        }
    }

    if (!poiTrackingActive) {
#ifdef USE_HEADTRACKER
        if (headTrackerActive) {
            headTrackerDevice_t *dev = headTrackerCommonDevice();
            if (gimbalCommonHtrkIsEnabled() && dev && headTrackerCommonIsValid(dev)) {
                gimbalAttitude.pan = headTrackerCommonGetPan(dev);
                gimbalAttitude.tilt = headTrackerCommonGetTilt(dev);
                gimbalAttitude.roll = headTrackerCommonGetRoll(dev);

                DEBUG_SET(DEBUG_HEADTRACKING, 4, 1);
            } else {
                gimbalAttitude.pan = constrain(gimbal_scale12(PWM_RANGE_MIN, PWM_RANGE_MAX, PWM_RANGE_MIDDLE + cfg->panTrim), HEADTRACKER_RANGE_MIN, HEADTRACKER_RANGE_MAX);
                gimbalAttitude.tilt = constrain(gimbal_scale12(PWM_RANGE_MIN, PWM_RANGE_MAX, PWM_RANGE_MIDDLE + cfg->tiltTrim), HEADTRACKER_RANGE_MIN, HEADTRACKER_RANGE_MAX);
                gimbalAttitude.roll = constrain(gimbal_scale12(PWM_RANGE_MIN, PWM_RANGE_MAX, PWM_RANGE_MIDDLE + cfg->rollTrim), HEADTRACKER_RANGE_MIN, HEADTRACKER_RANGE_MAX);
                DEBUG_SET(DEBUG_HEADTRACKING, 4, -1);
            }
        } else
#endif
        {
            DEBUG_SET(DEBUG_HEADTRACKING, 4, 2);
            gimbalAttitude.pan = manualPan;
            gimbalAttitude.tilt = manualTilt;
            gimbalAttitude.roll = manualRoll;
        }
    }

    DEBUG_SET(DEBUG_HEADTRACKING, 5, gimbalAttitude.pan);
    DEBUG_SET(DEBUG_HEADTRACKING, 6, gimbalAttitude.tilt);
    DEBUG_SET(DEBUG_HEADTRACKING, 7, gimbalAttitude.roll);

    gimbalAttitude.sensibility = cfg->sensitivity;

    uint16_t crc16 = 0;
    uint8_t *b = (uint8_t *)&gimbalAttitude;
    for (uint8_t i = 0; i < sizeof(gimbalHtkAttitudePkt_t) - 2; i++) {
        crc16 = crc16_ccitt(crc16, *(b + i));
    }
    gimbalAttitude.crch = (crc16 >> 8) & 0xFF;
    gimbalAttitude.crcl = crc16 & 0xFF;

    serialGimbalDevice.currentPanPWM = gimbal2pwm(gimbalAttitude.pan);

    serialBeginWrite(gimbalPort);
    serialWriteBuf(gimbalPort, (uint8_t *)&gimbalAttitude, sizeof(gimbalHtkAttitudePkt_t));
    serialEndWrite(gimbalPort);
}
#endif

int16_t gimbal2pwm(int16_t value)
{
    int16_t ret = 0;
    ret = scaleRange(value, HEADTRACKER_RANGE_MIN, HEADTRACKER_RANGE_MAX, PWM_RANGE_MIN, PWM_RANGE_MAX);
    return ret;
}


int16_t gimbal_scale12(int16_t inputMin, int16_t inputMax, int16_t value)
{
    int16_t ret = 0;
    ret = scaleRange(value, inputMin, inputMax, HEADTRACKER_RANGE_MIN, HEADTRACKER_RANGE_MAX);
    return ret;
}

#ifndef GIMBAL_UNIT_TEST

#if (defined(USE_HEADTRACKER) && defined(USE_HEADTRACKER_SERIAL))
static void resetState(gimbalSerialHtrkState_t *state)
{
    state->state = WAITING_HDR1;
    state->payloadSize = 0;
}

static bool checkCrc(gimbalHtkAttitudePkt_t *attitude)
{
    uint8_t *attitudePkt = (uint8_t *)attitude;
    uint16_t crc = 0;

    for(uint8_t i = 0; i < sizeof(gimbalHtkAttitudePkt_t) - 2; ++i) {
        crc = crc16_ccitt(crc, attitudePkt[i]);
    }

    return (attitude->crch == ((crc >> 8) & 0xFF)) &&
           (attitude->crcl == (crc & 0xFF));
}

void gimbalSerialHeadTrackerReceive(uint16_t c, void *data)
{
    static int charCount = 0;
    static int pktCount = 0;
    static int errorCount = 0;
    gimbalSerialHtrkState_t *state = (gimbalSerialHtrkState_t *)data;
    uint8_t *payload = (uint8_t *)&(state->attitude);
    payload += 2;

    DEBUG_SET(DEBUG_HEADTRACKING, 0, charCount++);
    DEBUG_SET(DEBUG_HEADTRACKING, 1, state->state);

    switch(state->state) {
        case WAITING_HDR1:
            if(c == HTKATTITUDE_SYNC0) {
                state->attitude.sync[0] = c;
                state->state = WAITING_HDR2;
            }
            break;
        case WAITING_HDR2:
            if(c == HTKATTITUDE_SYNC1) {
                state->attitude.sync[1] = c;
                state->state = WAITING_PAYLOAD;
            } else {
                resetState(state);
            }
            break;
        case WAITING_PAYLOAD:
            payload[state->payloadSize++] = c;
            if(state->payloadSize == HEADTRACKER_PAYLOAD_SIZE)
            {
                state->state = WAITING_CRCH;
            }
            break;
        case WAITING_CRCH:
            state->attitude.crch = c;
            state->state = WAITING_CRCL;
            break;
        case WAITING_CRCL:
            state->attitude.crcl = c;
            if(checkCrc(&(state->attitude))) {
                headTrackerDevice.expires = micros() + MAX_HEADTRACKER_DATA_AGE_US;
                headTrackerDevice.pan = constrain(state->attitude.pan, HEADTRACKER_RANGE_MIN, HEADTRACKER_RANGE_MAX);
                headTrackerDevice.tilt = constrain(state->attitude.tilt, HEADTRACKER_RANGE_MIN, HEADTRACKER_RANGE_MAX);
                headTrackerDevice.roll = constrain(state->attitude.roll, HEADTRACKER_RANGE_MIN, HEADTRACKER_RANGE_MAX);
                DEBUG_SET(DEBUG_HEADTRACKING, 2, pktCount++);
            } else {
                DEBUG_SET(DEBUG_HEADTRACKING, 3, errorCount++);
            }
            resetState(state);
            break;
    }
}


bool gimbalSerialHeadTrackerDetect(void)
{
    bool singleUart = gimbalSerialConfig()->singleUart;

    SD(fprintf(stderr, "[GIMBAL_HTRK]: headtracker Detect...\n"));
    serialPortConfig_t *portConfig = singleUart ? NULL : findSerialPortConfig(FUNCTION_GIMBAL_HEADTRACKER);

    if (portConfig) {
        SD(fprintf(stderr, "[GIMBAL_HTRK]: found port...\n"));
        headTrackerPort = openSerialPort(portConfig->identifier, FUNCTION_GIMBAL_HEADTRACKER, gimbalSerialHeadTrackerReceive, &headTrackerState,
                baudRates[portConfig->peripheral_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);

        if (headTrackerPort) {
            SD(fprintf(stderr, "[GIMBAL_HTRK]: port open!\n"));
            headTrackerPort->txBuffer = txBuffer;
            headTrackerPort->txBufferSize = GIMBAL_SERIAL_BUFFER_SIZE;
            headTrackerPort->txBufferTail = 0;
            headTrackerPort->txBufferHead = 0;
        } else {
            SD(fprintf(stderr, "[GIMBAL_HTRK]: port NOT open!\n"));
            return false;
        }
    }

    SD(fprintf(stderr, "[GIMBAL]: gimbalPort: %p headTrackerPort: %p\n", gimbalPort, headTrackerPort));
    return (singleUart && gimbalPort) || headTrackerPort;
}

bool gimbalSerialHeadTrackerInit(void)
{
    if(headTrackerConfig()->devType == HEADTRACKER_SERIAL) {
        if (gimbalSerialHeadTrackerDetect()) {
            SD(fprintf(stderr, "Setting gimbal device\n"));
            headTrackerCommonSetDevice(&headTrackerDevice);

            return true;
        }
    }

    return false;
}

void headtrackerSerialProcess(headTrackerDevice_t *headTrackerDevice, timeUs_t currentTimeUs)
{
    UNUSED(headTrackerDevice);
    UNUSED(currentTimeUs);
    return;
}

headTrackerDevType_e headtrackerSerialGetDeviceType(const headTrackerDevice_t *headTrackerDevice)
{
    UNUSED(headTrackerDevice);
    return HEADTRACKER_SERIAL;
}

#else

void gimbalSerialHeadTrackerReceive(uint16_t c, void *data)
{
    UNUSED(c);
    UNUSED(data);
}

#endif

#endif

#endif