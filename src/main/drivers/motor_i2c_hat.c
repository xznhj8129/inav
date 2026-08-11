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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_MOTOR_I2C_HAT

#include "common/log.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/motor_i2c_hat.h"
#include "drivers/time.h"

#include "flight/mixer.h"

// PCA9685 registers
#define PCA9685_MODE1               0x00
#define PCA9685_MODE2               0x01
#define PCA9685_LED0_ON_L           0x06
#define PCA9685_ALL_LED_ON_L        0xFA
#define PCA9685_PRESCALE            0xFE

#define PCA9685_MODE1_RESTART       0x80
#define PCA9685_MODE1_AI            0x20
#define PCA9685_MODE1_SLEEP         0x10
#define PCA9685_MODE1_ALLCALL       0x01
#define PCA9685_MODE2_OUTDRV        0x04

#define PCA9685_OSC_HZ              25000000
#define PCA9685_PWM_STEPS           4096
#define PCA9685_PWM_MAX             (PCA9685_PWM_STEPS - 1)
#define PCA9685_PRESCALE_MIN        3
#define PCA9685_PRESCALE_MAX        255

// The HAT drives the motor PWM channels at ~1.6kHz, same as the original Adafruit
// library (Adafruit_MotorHAT.__init__ defaults to freq=1600).
#define MOTOR_I2C_HAT_PWM_FREQ_HZ   1600

#define MOTOR_I2C_HAT_UPDATE_INTERVAL_US    (1000000 / MOTOR_I2C_HAT_UPDATE_HZ)

typedef enum {
    HAT_MOTOR_RELEASE = 0,
    HAT_MOTOR_FORWARD,
    HAT_MOTOR_REVERSE,
} hatMotorDirection_e;

typedef struct {
    uint8_t pwm;
    uint8_t in1;
    uint8_t in2;
} hatMotorChannels_t;

/*
 * PCA9685 channel assignment of the Adafruit Motor HAT, taken from
 * Adafruit_DCMotor.__init__() in Adafruit-Motor-HAT-Python-Library. Index 0 is the
 * connector silkscreened M1.
 */
static const hatMotorChannels_t hatMotorChannels[MOTOR_I2C_HAT_MOTOR_COUNT] = {
    [0] = { .pwm =  8, .in1 = 10, .in2 =  9 },
    [1] = { .pwm = 13, .in1 = 11, .in2 = 12 },
    [2] = { .pwm =  2, .in1 =  4, .in2 =  3 },
    [3] = { .pwm =  7, .in1 =  5, .in2 =  6 },
};

typedef struct {
    hatMotorDirection_e direction;
    uint16_t duty;                  // 0..4095
} hatMotorState_t;

static busDevice_t *busDev = NULL;
static bool hatReady = false;

static hatMotorState_t requestedState[MOTOR_I2C_HAT_MOTOR_COUNT];
static hatMotorState_t appliedState[MOTOR_I2C_HAT_MOTOR_COUNT];

static timeUs_t lastUpdateUs = 0;

static bool pca9685SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    const uint8_t data[4] = { on & 0xFF, on >> 8, off & 0xFF, off >> 8 };
    return busWriteBuf(busDev, PCA9685_LED0_ON_L + 4 * channel, data, sizeof(data));
}

/*
 * Direction pins are driven with the PCA9685 full-on / full-off bit (bit 12 of the ON
 * or OFF counter) rather than an arbitrary duty cycle, matching Adafruit_MotorHAT.setPin().
 */
static bool pca9685SetPin(uint8_t channel, bool high)
{
    return high ? pca9685SetPWM(channel, PCA9685_PWM_STEPS, 0)
                : pca9685SetPWM(channel, 0, PCA9685_PWM_STEPS);
}

static bool pca9685SetDuty(uint8_t channel, uint16_t duty)
{
    if (duty == 0) {
        return pca9685SetPin(channel, false);
    }

    if (duty >= PCA9685_PWM_MAX) {
        return pca9685SetPin(channel, true);
    }

    return pca9685SetPWM(channel, 0, duty);
}

static bool pca9685Reset(void)
{
    const uint8_t allOff[4] = { 0, 0, 0, 0 };
    uint8_t mode1;

    /*
     * Auto-increment must be enabled before anything else: this driver writes each
     * channel's four LEDn_ON_L..OFF_H bytes as a single burst, and with AI off the
     * PCA9685 drops all four into LEDn_ON_L, leaving ON_H/OFF_L/OFF_H untouched - the
     * outputs then never assert. The Adafruit library writes those registers one at a
     * time, which is why it never has to set this bit.
     */
    if (!busWrite(busDev, PCA9685_MODE1, PCA9685_MODE1_ALLCALL | PCA9685_MODE1_AI)) {
        return false;
    }

    if (!busWrite(busDev, PCA9685_MODE2, PCA9685_MODE2_OUTDRV)) {
        return false;
    }

    if (!busWriteBuf(busDev, PCA9685_ALL_LED_ON_L, allOff, sizeof(allOff))) {
        return false;
    }

    delay(5);

    if (!busRead(busDev, PCA9685_MODE1, &mode1)) {
        return false;
    }

    // Wake the oscillator up
    if (!busWrite(busDev, PCA9685_MODE1, mode1 & ~PCA9685_MODE1_SLEEP)) {
        return false;
    }

    delay(5);

    return true;
}

static bool pca9685SetFrequency(uint16_t freqHz)
{
    // prescale = round(osc / (4096 * freq)) - 1; 1600Hz gives 3, i.e. an actual 1526Hz
    const uint32_t divisor = PCA9685_PWM_STEPS * (uint32_t)freqHz;
    const uint32_t prescale = constrain(((PCA9685_OSC_HZ + divisor / 2) / divisor) - 1,
                                        PCA9685_PRESCALE_MIN, PCA9685_PRESCALE_MAX);
    uint8_t oldMode;

    if (!busRead(busDev, PCA9685_MODE1, &oldMode)) {
        return false;
    }

    // PRESCALE is only writable while the oscillator is asleep
    if (!busWrite(busDev, PCA9685_MODE1, (oldMode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP)) {
        return false;
    }

    if (!busWrite(busDev, PCA9685_PRESCALE, prescale)) {
        return false;
    }

    if (!busWrite(busDev, PCA9685_MODE1, oldMode)) {
        return false;
    }

    delay(5);

    return busWrite(busDev, PCA9685_MODE1, oldMode | PCA9685_MODE1_RESTART);
}

static bool applyMotorState(uint8_t motorIndex)
{
    const hatMotorChannels_t *channels = &hatMotorChannels[motorIndex];
    const hatMotorState_t *requested = &requestedState[motorIndex];
    hatMotorState_t *applied = &appliedState[motorIndex];
    bool ok = true;

    if (requested->direction != applied->direction) {
        // TB6612: IN1 high / IN2 low is forward, the inverse is reverse, both low coasts
        ok = pca9685SetPin(channels->in1, requested->direction == HAT_MOTOR_FORWARD) && ok;
        ok = pca9685SetPin(channels->in2, requested->direction == HAT_MOTOR_REVERSE) && ok;
        applied->direction = requested->direction;
    }

    if (requested->duty != applied->duty) {
        ok = pca9685SetDuty(channels->pwm, requested->duty) && ok;
        applied->duty = requested->duty;
    }

    return ok;
}

bool motorI2CHatIsReady(void)
{
    return hatReady;
}

bool motorI2CHatInit(void)
{
    hatReady = false;

    busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_ADAFRUIT_MOTOR_HAT, 0, OWNER_MOTOR);
    if (busDev == NULL) {
        LOG_ERROR(PWM, "I2C motor HAT bus device not registered");
        return false;
    }

    if (!pca9685Reset() || !pca9685SetFrequency(MOTOR_I2C_HAT_PWM_FREQ_HZ)) {
        LOG_ERROR(PWM, "I2C motor HAT not responding at 0x%02X", MOTOR_I2C_HAT_I2C_ADDRESS);
        busDeviceDeInit(busDev);
        busDev = NULL;
        return false;
    }

    hatReady = true;

    // Force the cached state to be written out, whatever the chip was doing before
    memset(requestedState, 0, sizeof(requestedState));
    for (int i = 0; i < MOTOR_I2C_HAT_MOTOR_COUNT; i++) {
        appliedState[i].direction = HAT_MOTOR_FORWARD;
        appliedState[i].duty = PCA9685_PWM_MAX;
    }

    motorI2CHatReleaseAll();

    return true;
}

void motorI2CHatWrite(uint8_t motorIndex, uint16_t value)
{
    if (motorIndex >= MOTOR_I2C_HAT_MOTOR_COUNT) {
        return;
    }

    const uint16_t deadbandLow = reversibleMotorsConfig()->deadband_low;
    const uint16_t deadbandHigh = reversibleMotorsConfig()->deadband_high;
    const uint16_t fullForward = getMaxThrottle();
    const uint16_t fullReverse = motorConfig()->mincommand;

    hatMotorState_t *requested = &requestedState[motorIndex];

    if (value > deadbandHigh && fullForward > deadbandHigh) {
        requested->direction = HAT_MOTOR_FORWARD;
        requested->duty = scaleRange(constrain(value, deadbandHigh, fullForward),
                                     deadbandHigh, fullForward, 0, PCA9685_PWM_MAX);
    } else if (value < deadbandLow && deadbandLow > fullReverse) {
        requested->direction = HAT_MOTOR_REVERSE;
        requested->duty = scaleRange(constrain(value, fullReverse, deadbandLow),
                                     deadbandLow, fullReverse, 0, PCA9685_PWM_MAX);
    } else {
        requested->direction = HAT_MOTOR_RELEASE;
        requested->duty = 0;
    }
}

void motorI2CHatUpdate(void)
{
    if (!hatReady) {
        return;
    }

    const timeUs_t currentTimeUs = micros();
    if ((currentTimeUs - lastUpdateUs) < MOTOR_I2C_HAT_UPDATE_INTERVAL_US) {
        return;
    }
    lastUpdateUs = currentTimeUs;

    for (int i = 0; i < MOTOR_I2C_HAT_MOTOR_COUNT; i++) {
        applyMotorState(i);
    }
}

void motorI2CHatReleaseAll(void)
{
    if (!hatReady) {
        return;
    }

    for (int i = 0; i < MOTOR_I2C_HAT_MOTOR_COUNT; i++) {
        requestedState[i].direction = HAT_MOTOR_RELEASE;
        requestedState[i].duty = 0;
        applyMotorState(i);
    }

    lastUpdateUs = micros();
}

#endif
