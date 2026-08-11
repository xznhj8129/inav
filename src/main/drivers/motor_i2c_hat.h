/*
 * This file is part of INAV.
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

#include <stdbool.h>
#include <stdint.h>

// Adafruit DC & Stepper Motor HAT (product 2348): a PCA9685 driving two TB6612 dual
// H-bridges, giving four bidirectional DC motor outputs M1..M4.
#define MOTOR_I2C_HAT_I2C_ADDRESS   0x60
#define MOTOR_I2C_HAT_MOTOR_COUNT   4

// Rate at which cached motor commands are flushed to the HAT. A skid-steer rover
// drivetrain does not benefit from anything faster, and the I2C bus is shared with
// the baro/mag.
#define MOTOR_I2C_HAT_UPDATE_HZ     100

bool motorI2CHatInit(void);
bool motorI2CHatIsReady(void);

// Caches the desired command for one motor. value is a centered command in the
// 1000..2000 domain: below the reversible-motors deadband is reverse, above it is
// forward, inside it the motor is released.
void motorI2CHatWrite(uint8_t motorIndex, uint16_t value);

// Flushes changed motor commands to the HAT. Rate limited internally, so it is safe
// to call from the fast output-completion path.
void motorI2CHatUpdate(void);

// Immediately coasts all four motors. Used on disarm, shutdown and init failure.
void motorI2CHatReleaseAll(void);
