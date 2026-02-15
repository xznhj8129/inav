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

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool hasImu;
    bool hasPose;
    bool hasBaro;
    bool hasMag;
    bool hasNavSat;
    bool hasVelocity;
    uint32_t imuSeq;
    uint64_t imuSimStampUs;
    uint64_t imuStampUs;
    uint64_t poseStampUs;
    uint64_t baroStampUs;
    uint64_t magStampUs;
    uint64_t navsatStampUs;
    uint64_t odomStampUs;
    float accelMss[3];
    float gyroRadS[3];
    float quatWxyz[4];
    float poseQuatWxyz[4];
    float magTesla[3];
    float pressurePa;
    double latitudeDeg;
    double longitudeDeg;
    double altitudeM;
    float velocityEnuMss[3];
} gazeboNativeSample_t;

typedef struct {
    uint32_t imuUpdates;
    uint32_t baroUpdates;
    uint32_t magUpdates;
    uint32_t navsatUpdates;
    uint32_t odomUpdates;
    uint32_t poseUpdates;
    uint32_t imuDtUsLast;
    uint32_t imuDtUsMin;
    uint32_t imuDtUsMax;
    uint32_t imuAgeUs;
    uint32_t poseAgeUs;
    uint32_t navsatAgeUs;
    uint32_t baroAgeUs;
    uint32_t magAgeUs;
} gazeboNativeStats_t;

bool gazeboNativeInit(const char *worldName, const char *modelName);
bool gazeboNativePublishMotors(const float *motorRadS, uint8_t motorCount);
bool gazeboNativeGetSample(gazeboNativeSample_t *sample);
bool gazeboNativeGetStats(gazeboNativeStats_t *stats);
void gazeboNativeClose(void);

#ifdef __cplusplus
}
#endif
