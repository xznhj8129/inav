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

#pragma once

// Shared SITL simulator layer: the only code that writes INAV's simulated
// sensors and reads its outputs for a physics backend. Plain C types only, so
// a C++ backend can include it.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// What a simulated vehicle's sensors read, in SI units. Body vectors are
// forward-left-up (FLU), the frame INAV's simulated sensors take; a backend
// converts its own frames once, into this.
typedef struct {
    float specificForceFlu[3];  // m/s^2, what an accelerometer reads: (0, 0, +9.81) level at rest
    float rateFlu[3];           // rad/s
    float magFlu[3];            // direction of Earth's field, unit vector
    float roll;                 // rad, right wing down positive
    float pitch;                // rad, nose up positive
    float yaw;                  // rad, true heading, clockwise from north
    double latDeg;
    double lonDeg;
    float altMslM;
    float velNed[3];            // m/s
    float pressurePa;
    float airspeedMs;
    float rangeM;               // distance to the ground along body down; negative when out of range
} simSensorSample_t;

// Where the simulated vehicle really is, for scenario checks.
typedef struct {
    uint32_t simTimeUs;
    double latDeg;
    double lonDeg;
    float altMslM;
    float aglM;
    float roll;                 // rad, same conventions as simSensorSample_t
    float pitch;
    float yaw;
    float velNed[3];            // m/s
    float airspeedMs;
} simTruth_t;

// A physics backend running inside the SITL process on the stepped clock.
typedef struct {
    // Run physics up to simTimeUs and apply the sensors. The first call only
    // sets the start time. True when physics advanced.
    bool (*advanceTo)(uint32_t simTimeUs);
    void (*getTruth)(simTruth_t *truth);
} simInProcessBackend_t;

void simSensorsInit(bool useImu, uint16_t gpsRateHz);

// Before the stepped clock starts: inertial sensors, baro and pitot only, so
// boot calibration sees the start condition. No GPS fix yet.
void simSensorsApplyBoot(const simSensorSample_t *sample);

// Every sensor; GPS at its rate. The first call marks SITL as simulator-driven.
void simSensorsApply(const simSensorSample_t *sample, uint32_t nowUs);

// Attitude in INAV's units and signs: decidegrees, pitch positive nose down,
// heading 0..3600.
void simAttitudeToInav(float roll, float pitch, float yaw, int16_t *rollDd, int16_t *pitchDd, int16_t *yawDd);

// Flight controller state a simulated launcher acts on
bool simArmed(void);
bool simLaunchReady(void);      // NAV LAUNCH is waiting for the throw ("READY TO LAUNCH")

uint8_t simMotorCount(void);
uint8_t simServoCount(void);
float simMotorOutput(uint8_t index);    // 0..1
float simServoOutput(uint8_t index);    // -1..1

#ifdef __cplusplus
}
#endif
