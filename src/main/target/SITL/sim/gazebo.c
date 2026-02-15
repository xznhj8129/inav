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
 * Free Software Foundation, either version 3, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <pthread.h>

#include "platform.h"
#include "target.h"
#include "target/SITL/sim/gazebo.h"
#include "target/SITL/sim/gazebo_native.h"
#include "target/SITL/sim/simHelper.h"

#include "common/maths.h"
#include "common/utils.h"
#include "drivers/time.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "sensors/battery_sensor_fake.h"
#include "sensors/acceleration.h"
#include "drivers/pitotmeter/pitotmeter_fake.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "io/rangefinder.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "io/gps.h"
#include "fc/runtime_config.h"
#include "rx/sim.h"

static uint8_t pwmMapping[GZ_MAX_PWM_OUTS];
static uint8_t mappingCount;
static bool useImu;
static bool initialized;
static pthread_t workerThread;
static uint32_t gazeboTxPublishedCount;
static uint32_t gazeboTxDroppedCount;
static uint32_t gazeboRxSampleCount;
static timeMs_t gazeboLastStatsMs;
static uint32_t gazeboLastImuSeq;
/* INAV QUADX motor order is RR,FR,RL,FL while Gazebo x500/x3 expects FR,RL,FL,RR.
        #0 = front right
        #1 = rear left
        #2 = front left
        #3 = rear right
        */
static const uint8_t defaultGazeboMotorOrder[GZ_MAX_PWM_OUTS] = {0, 3, 2, 1};
//static const uint8_t defaultGazeboMotorOrder[GZ_MAX_PWM_OUTS] = {1, 2, 3, 0};

static float clampUnit(float value)
{
    return constrainf(value, 0.0f, 1.0f);
}

static void getMotorCommands(float *motorRadS)
{
    for (uint8_t i = 0; i < GZ_MAX_PWM_OUTS; i++) {
        motorRadS[i] = 0.0f;
    }

    if (mappingCount == 0) {
        for (uint8_t i = 0; i < GZ_MAX_PWM_OUTS; i++) {
            motorRadS[i] = clampUnit(PWM_TO_FLOAT_0_1(motor[defaultGazeboMotorOrder[i]])) * 1000.0f;
        }
        return;
    }

    for (uint8_t i = 0; i < mappingCount && i < GZ_MAX_PWM_OUTS; i++) {
        if (!(pwmMapping[i] & 0x80)) {
            continue;
        }

        const uint8_t inavMotorIndex = pwmMapping[i] & 0x7f;
        motorRadS[i] = clampUnit(PWM_TO_FLOAT_0_1(motor[inavMotorIndex])) * 1000.0f;
    }
}

static void convertQuaternionToRPY(const float *quatWxyz, int16_t *rollInav, int16_t *pitchInav, int16_t *yawInav)
{
    const float qw = quatWxyz[0];
    const float qx = quatWxyz[1];
    const float qy = quatWxyz[2];
    const float qz = quatWxyz[3];

    const float sinrCosp = 2.0f * (qw * qx + qy * qz);
    const float cosrCosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    const float roll = atan2f(sinrCosp, cosrCosp);

    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp = constrainf(sinp, -1.0f, 1.0f);
    const float pitch = asinf(sinp);

    const float sinyCosp = 2.0f * (qw * qz + qx * qy);
    const float cosyCosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    float yaw = atan2f(sinyCosp, cosyCosp) * (180.0f / M_PIf);
    if (yaw < 0.0f) {
        yaw += 360.0f;
    }

    *rollInav = constrainToInt16(roll * (1800.0f / M_PIf));
    *pitchInav = constrainToInt16(-pitch * (1800.0f / M_PIf));
    *yawInav = constrainToInt16(yaw * 10.0f);
}

static void updateGps(const gazeboNativeSample_t *sample, int16_t yawInav)
{
    int16_t course = yawInav;
    int16_t groundspeed = 0;
    if (sample->hasVelocity) {
        const float velEast = sample->velocityEnuMss[0];
        const float velNorth = sample->velocityEnuMss[1];
        groundspeed = (int16_t)roundf(sqrtf(velEast * velEast + velNorth * velNorth) * 100.0f);
        course = (int16_t)roundf(RADIANS_TO_DECIDEGREES(atan2_approx(velEast, velNorth)));
        if (course < 0) {
            course += 3600;
        }
    }

    gpsFakeSet(
        GPS_FIX_3D,
        16,
        (int32_t)lroundf((float)sample->latitudeDeg * 10000000.0f),
        (int32_t)lroundf((float)sample->longitudeDeg * 10000000.0f),
        (int32_t)lroundf((float)sample->altitudeM * 100.0f),
        groundspeed,
        course,
        0,
        0,
        0,
        0
    );
}

static void updateGpsNoFix(void)
{
    gpsFakeSet(
        GPS_NO_FIX,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    );
}

static void updateMagnetometer(const gazeboNativeSample_t *sample, int16_t rollInav, int16_t pitchInav, int16_t yawInav)
{
    if (sample->hasMag) {
        fakeMagSet(
            constrainToInt16(sample->magTesla[0] * 10000.0f),
            constrainToInt16(-sample->magTesla[1] * 10000.0f),
            constrainToInt16(-sample->magTesla[2] * 10000.0f)
        );
        return;
    }

    fpQuaternion_t quat;
    fpVector3_t north;
    north.x = 1.0f;
    north.y = 0.0f;
    north.z = 0.0f;
    computeQuaternionFromRPY(&quat, rollInav, pitchInav, yawInav);
    transformVectorEarthToBody(&north, &quat);
    fakeMagSet(
        constrainToInt16(north.x * 1024.0f),
        constrainToInt16(north.y * 1024.0f),
        constrainToInt16(north.z * 1024.0f)
    );
}

static void *gazeboWorker(void *arg)
{
    UNUSED(arg);

    while (true) {
        float motorRadS[GZ_MAX_PWM_OUTS];
        getMotorCommands(motorRadS);
        if (gazeboNativePublishMotors(motorRadS, GZ_MAX_PWM_OUTS)) {
            gazeboTxPublishedCount++;
        } else {
            gazeboTxDroppedCount++;
        }

        const timeMs_t nowMs = millis();

        gazeboNativeSample_t sample;
        const bool hasSample = gazeboNativeGetSample(&sample);
        if (!hasSample || !sample.hasImu) {
            if ((uint32_t)(nowMs - gazeboLastStatsMs) >= 1000U) {
                gazeboNativeStats_t nativeStats;
                const bool hasStats = gazeboNativeGetStats(&nativeStats);
                fprintf(stderr, "[SIM][GZ] tx_ok=%" PRIu32 " tx_fail=%" PRIu32 " rx=%" PRIu32 " sample=%u imu=%u pose=%u navsat=%u baro=%u mag=%u cb_imu=%" PRIu32 " cb_pose=%" PRIu32 " cb_navsat=%" PRIu32 " cb_baro=%" PRIu32 " cb_mag=%" PRIu32 " cb_odom=%" PRIu32 " imu_dt_us=%" PRIu32 "/%" PRIu32 "/%" PRIu32 " m0=%.1f m1=%.1f m2=%.1f m3=%.1f\n",
                    gazeboTxPublishedCount,
                    gazeboTxDroppedCount,
                    gazeboRxSampleCount,
                    hasSample ? 1 : 0,
                    hasSample && sample.hasImu ? 1 : 0,
                    hasSample && sample.hasPose ? 1 : 0,
                    hasSample && sample.hasNavSat ? 1 : 0,
                    hasSample && sample.hasBaro ? 1 : 0,
                    hasSample && sample.hasMag ? 1 : 0,
                    hasStats ? nativeStats.imuUpdates : 0,
                    hasStats ? nativeStats.poseUpdates : 0,
                    hasStats ? nativeStats.navsatUpdates : 0,
                    hasStats ? nativeStats.baroUpdates : 0,
                    hasStats ? nativeStats.magUpdates : 0,
                    hasStats ? nativeStats.odomUpdates : 0,
                    hasStats ? nativeStats.imuDtUsMin : 0,
                    hasStats ? nativeStats.imuDtUsLast : 0,
                    hasStats ? nativeStats.imuDtUsMax : 0,
                    (double)motorRadS[0],
                    (double)motorRadS[1],
                    (double)motorRadS[2],
                    (double)motorRadS[3]);
                gazeboLastStatsMs = nowMs;
            }
            delay(2);
            continue;
        }

        if (sample.imuSeq == gazeboLastImuSeq) {
            delay(1);
            continue;
        }
        gazeboLastImuSeq = sample.imuSeq;

        gazeboRxSampleCount++;

        int16_t rollInav;
        int16_t pitchInav;
        int16_t yawInav;
        convertQuaternionToRPY(sample.quatWxyz, &rollInav, &pitchInav, &yawInav);
        const float rollDeg = rollInav * 0.1f;
        const float pitchDeg = pitchInav * 0.1f;
        const float yawDeg = yawInav * 0.1f;

        if (sample.hasNavSat) {
            updateGps(&sample, yawInav);
        } else {
            updateGpsNoFix();
        }

        if (!useImu) {
            imuSetAttitudeRPY(rollInav, pitchInav, yawInav);
            imuUpdateAttitude(micros());
        }

        const int16_t accX = constrainToInt16(-sample.accelMss[0] * 1000.0f);
        const int16_t accY = constrainToInt16(sample.accelMss[1] * 1000.0f);
        const int16_t accZ = constrainToInt16(sample.accelMss[2] * 1000.0f);
        fakeAccSet(accX, accY, accZ);

        const int16_t gyroX = constrainToInt16(RADIANS_TO_DEGREES(sample.gyroRadS[0]) * 16.0f);
        const int16_t gyroY = constrainToInt16(RADIANS_TO_DEGREES(-sample.gyroRadS[1]) * 16.0f);
        const int16_t gyroZ = constrainToInt16(RADIANS_TO_DEGREES(-sample.gyroRadS[2]) * 16.0f);
        fakeGyroSet(gyroX, gyroY, gyroZ);

        if (sample.hasBaro) {
            fakeBaroSet((int32_t)roundf(sample.pressurePa), DEGREES_TO_CENTIDEGREES(21));
        }

        if (sample.hasVelocity) {
            const float velEast = sample.velocityEnuMss[0];
            const float velNorth = sample.velocityEnuMss[1];
            fakePitotSetAirspeed(sqrtf(velEast * velEast + velNorth * velNorth) * 100.0f);
        }

        updateMagnetometer(&sample, rollInav, pitchInav, yawInav);

        fakeRangefindersSetData(-1);
        fakeBattSensorSetVbat(16.8f * 100);

        if ((uint32_t)(nowMs - gazeboLastStatsMs) >= 1000U) {
            const float horizontalSpeedMss = sample.hasVelocity
                ? sqrtf(sample.velocityEnuMss[0] * sample.velocityEnuMss[0] + sample.velocityEnuMss[1] * sample.velocityEnuMss[1])
                : 0.0f;
            gazeboNativeStats_t nativeStats;
            const bool hasStats = gazeboNativeGetStats(&nativeStats);
            fprintf(stderr, "[SIM][GZ] tx_ok=%" PRIu32 " tx_fail=%" PRIu32 " rx=%" PRIu32 " imu=%u pose=%u navsat=%u baro=%u vel=%u cb_imu=%" PRIu32 " cb_pose=%" PRIu32 " cb_navsat=%" PRIu32 " cb_baro=%" PRIu32 " cb_mag=%" PRIu32 " cb_odom=%" PRIu32 " imu_dt_us=%" PRIu32 "/%" PRIu32 "/%" PRIu32 " age_us=%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 " lat=%.6f lon=%.6f alt=%.2f vxy=%.2f rpy=%.1f,%.1f,%.1f acc_raw=%.3f,%.3f,%.3f acc_fc=%d,%d,%d gyro_raw=%.3f,%.3f,%.3f gyro_fc=%d,%d,%d m0=%.1f m1=%.1f m2=%.1f m3=%.1f\n",
                gazeboTxPublishedCount,
                gazeboTxDroppedCount,
                gazeboRxSampleCount,
                sample.hasImu ? 1 : 0,
                sample.hasPose ? 1 : 0,
                sample.hasNavSat ? 1 : 0,
                sample.hasBaro ? 1 : 0,
                sample.hasVelocity ? 1 : 0,
                hasStats ? nativeStats.imuUpdates : 0,
                hasStats ? nativeStats.poseUpdates : 0,
                hasStats ? nativeStats.navsatUpdates : 0,
                hasStats ? nativeStats.baroUpdates : 0,
                hasStats ? nativeStats.magUpdates : 0,
                hasStats ? nativeStats.odomUpdates : 0,
                hasStats ? nativeStats.imuDtUsMin : 0,
                hasStats ? nativeStats.imuDtUsLast : 0,
                hasStats ? nativeStats.imuDtUsMax : 0,
                hasStats ? nativeStats.imuAgeUs : 0,
                hasStats ? nativeStats.poseAgeUs : 0,
                hasStats ? nativeStats.navsatAgeUs : 0,
                hasStats ? nativeStats.baroAgeUs : 0,
                hasStats ? nativeStats.magAgeUs : 0,
                sample.latitudeDeg,
                sample.longitudeDeg,
                sample.altitudeM,
                (double)horizontalSpeedMss,
                (double)rollDeg,
                (double)pitchDeg,
                (double)yawDeg,
                (double)sample.accelMss[0],
                (double)sample.accelMss[1],
                (double)sample.accelMss[2],
                accX,
                accY,
                accZ,
                (double)sample.gyroRadS[0],
                (double)sample.gyroRadS[1],
                (double)sample.gyroRadS[2],
                gyroX,
                gyroY,
                gyroZ,
                (double)motorRadS[0],
                (double)motorRadS[1],
                (double)motorRadS[2],
                (double)motorRadS[3]);
            gazeboLastStatsMs = nowMs;
        }

        if (!initialized) {
            ENABLE_ARMING_FLAG(SIMULATOR_MODE_SITL);
            ENABLE_STATE(ACCELEROMETER_CALIBRATED);
            initialized = true;
        }

        unlockMainPID();
    }

    return NULL;
}

bool simGazeboInit(char *worldName, char *modelName, uint8_t *mapping, uint8_t mapCount, bool imu)
{
    memcpy(pwmMapping, mapping, mapCount);
    mappingCount = mapCount;
    useImu = imu;
    gazeboTxPublishedCount = 0;
    gazeboTxDroppedCount = 0;
    gazeboRxSampleCount = 0;
    gazeboLastStatsMs = 0;
    gazeboLastImuSeq = 0;

    if (!gazeboNativeInit(worldName, modelName)) {
#if defined(USE_GAZEBO_NATIVE)
        fprintf(stderr, "[SIM] Gazebo transport initialization failed for world='%s' model='%s'.\n", worldName, modelName);
#else
        fprintf(stderr, "[SIM] Gazebo support not built. Install gz-transport/gz-msgs development packages and rebuild SITL.\n");
#endif
        return false;
    }

    if (pthread_create(&workerThread, NULL, gazeboWorker, NULL) != 0) {
        gazeboNativeClose();
        return false;
    }

    while (!initialized) {
        delay(250);
    }

    return true;
}