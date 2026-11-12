/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "target/SITL/sim/simSensors.h"
#include "target/SITL/sim/simHelper.h"

#include "common/maths.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/pitotmeter/pitotmeter_fake.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "io/gps.h"
#include "io/rangefinder.h"
#include "sensors/battery_sensor_fake.h"

// Scales of INAV's simulated sensor inputs
#define SIM_ACC_SCALE           1000.0f     // fake accelerometer: mm/s^2 (acc_1G = 9806)
#define SIM_GYRO_SCALE          16.0f       // fake gyro: deg/s * 16
#define SIM_MAG_SCALE           1024.0f     // fake magnetometer: unit field vector * 1024
#define SIM_BARO_TEMP_C         21
#define SIM_GPS_SATS            14
#define SIM_VBAT_CENTIVOLTS     1680        // a full 4S pack

static bool useImu;
static uint32_t gpsPeriodUs;
static uint32_t lastGpsUs;
static bool gpsStarted;
static bool simulatorMode;

void simSensorsInit(bool imu, uint16_t gpsRateHz)
{
    useImu = imu;
    gpsPeriodUs = 1000000 / gpsRateHz;
}

void simSensorsApplyBoot(const simSensorSample_t *s)
{
    // INAV's simulated sensors are forward-left-up, as the sample is
    fakeAccSet(
        constrainToInt16(s->specificForceFlu[X] * SIM_ACC_SCALE),
        constrainToInt16(s->specificForceFlu[Y] * SIM_ACC_SCALE),
        constrainToInt16(s->specificForceFlu[Z] * SIM_ACC_SCALE));
    fakeGyroSet(
        constrainToInt16(RADIANS_TO_DEGREES(s->rateFlu[X]) * SIM_GYRO_SCALE),
        constrainToInt16(RADIANS_TO_DEGREES(s->rateFlu[Y]) * SIM_GYRO_SCALE),
        constrainToInt16(RADIANS_TO_DEGREES(s->rateFlu[Z]) * SIM_GYRO_SCALE));
    fakeMagSet(
        constrainToInt16(s->magFlu[X] * SIM_MAG_SCALE),
        constrainToInt16(s->magFlu[Y] * SIM_MAG_SCALE),
        constrainToInt16(s->magFlu[Z] * SIM_MAG_SCALE));
    fakeBaroSet(lrintf(s->pressurePa), DEGREES_TO_CENTIDEGREES(SIM_BARO_TEMP_C));
    fakePitotSetAirspeed(s->airspeedMs * 100.0f);
    fakeRangefindersSetData(s->rangeM < 0.0f ? -1 : lrintf(s->rangeM * 100.0f));
    fakeBattSensorSetVbat(SIM_VBAT_CENTIVOLTS);
}

void simSensorsApply(const simSensorSample_t *s, uint32_t nowUs)
{
    simSensorsApplyBoot(s);

    if (!useImu) {
        int16_t roll, pitch, yaw;
        simAttitudeToInav(s->roll, s->pitch, s->yaw, &roll, &pitch, &yaw);
        imuSetAttitudeRPY(roll, pitch, yaw);
        imuUpdateAttitude(nowUs);
    }

    if (!gpsStarted || (uint32_t)(nowUs - lastGpsUs) >= gpsPeriodUs) {
        gpsStarted = true;
        lastGpsUs = nowUs;
        int16_t course = lrintf(RADIANS_TO_DECIDEGREES(atan2f(s->velNed[1], s->velNed[0])));
        if (course < 0) {
            course += 3600;
        }
        gpsFakeSet(
            GPS_FIX_3D,
            SIM_GPS_SATS,
            (int32_t)llround(s->latDeg * (double)1e7),
            (int32_t)llround(s->lonDeg * (double)1e7),
            lrintf(s->altMslM * 100.0f),
            constrainToInt16(sqrtf(s->velNed[0] * s->velNed[0] + s->velNed[1] * s->velNed[1]) * 100.0f),
            course,
            constrainToInt16(s->velNed[0] * 100.0f),
            constrainToInt16(s->velNed[1] * 100.0f),
            constrainToInt16(s->velNed[2] * 100.0f),
            0);
    }

    if (!simulatorMode) {
        simulatorMode = true;
        ENABLE_ARMING_FLAG(SIMULATOR_MODE_SITL);
        ENABLE_STATE(ACCELEROMETER_CALIBRATED);
    }
}

void simAttitudeToInav(float roll, float pitch, float yaw, int16_t *rollDd, int16_t *pitchDd, int16_t *yawDd)
{
    *rollDd = constrainToInt16(RADIANS_TO_DECIDEGREES(roll));
    *pitchDd = constrainToInt16(RADIANS_TO_DECIDEGREES(-pitch));
    *yawDd = lrintf(RADIANS_TO_DECIDEGREES(yaw)) % 3600;
    if (*yawDd < 0) {
        *yawDd += 3600;
    }
}

uint8_t simMotorCount(void)
{
    return MAX_SUPPORTED_MOTORS;
}

uint8_t simServoCount(void)
{
    return MAX_SUPPORTED_SERVOS;
}

float simMotorOutput(uint8_t index)
{
    return constrainf((motor[index] - 1000) / 1000.0f, 0.0f, 1.0f);
}

float simServoOutput(uint8_t index)
{
    return constrainf((servo[index] - 1500) / 500.0f, -1.0f, 1.0f);
}
