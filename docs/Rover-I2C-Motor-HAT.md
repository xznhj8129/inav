# Adafruit I2C Motor HAT drivetrain (`motor_pwm_protocol = I2C_HAT`)

Drives an Adafruit DC & Stepper Motor HAT (product 2348) directly from the flight
controller over I2C, as the propulsion output of a skid-steer rover. No Raspberry Pi is
involved: the HAT is used purely as a PCA9685 + two TB6612 H-bridges.

```
RC / navigation -> INAV motor mixer -> motor[0..3] -> I2C_HAT -> PCA9685 @ 0x60 -> TB6612 -> M1..M4
```

The flight controller's motor PWM pins are not used by this protocol, so all timer
outputs stay available for ordinary servos.

## Wiring

Four wires between the FC and the HAT's 40-pin header:

| FC       | HAT      |
|----------|----------|
| 3V3      | 3.3V     |
| GND      | GND      |
| SDA      | SDA      |
| SCL      | SCL      |

Motor power is supplied separately to the HAT's own power terminal.

## Motor mapping

| INAV motor | HAT output | PCA9685 PWM | IN1 | IN2 |
|------------|------------|-------------|-----|-----|
| 0          | M1         | 8           | 10  | 9   |
| 1          | M2         | 13          | 11  | 12  |
| 2          | M3         | 2           | 4   | 3   |
| 3          | M4         | 7           | 5   | 6   |

Which wheel each output drives is a mixer/wiring decision, not a driver one. Maximum four
motors; asking the mixer for more fails motor initialisation and blocks arming.

## Command range

I2C_HAT motors are *centered bidirectional*: each motor is mixed independently over the
full 1000..2000 range with neutral in the middle.

| motor value               | result                          |
|---------------------------|---------------------------------|
| 2000                      | full forward                    |
| `3d_deadband_high`..2000  | forward, duty scaled 0..100%    |
| inside the 3D deadband    | released (coasting)             |
| 1000..`3d_deadband_low`   | reverse, duty scaled 0..100%    |
| 1000                      | full reverse                    |

`FEATURE_REVERSIBLE_MOTORS` is required and is enabled automatically when this protocol
is selected - it supplies the neutral/deadband semantics that centered-stick arming
depends on. Unlike normal reversible motors there is no single global motor direction, so
the mixer is free to run the left wheels forward while the right wheels reverse. Neutral
throttle does not stop the mixer, so the rover can pivot on the spot with yaw alone.

Because 1500 is "stopped", `max_throttle` for this protocol is 2000 rather than the usual
rover limit of 1850.

Steering comes entirely from the `mmix` yaw coefficients. Do not expect the motor driver
to know anything about left and right.

## Base configuration

```
set platform_type = ROVER
set motor_pwm_protocol = I2C_HAT
set min_command = 1000
set 3d_deadband_low = 1450
set 3d_deadband_high = 1550
set 3d_neutral = 1500
feature REVERSIBLE_MOTORS
save
```

## Working out the mixer

Which HAT output drives which wheel depends entirely on how the motors were wired, and
it is rarely the tidy left/right arrangement you would draw. Measure it rather than
assume it - trying to infer the mapping from how the vehicle pivots does not work,
because several different wirings produce similar-looking wrong behaviour.

With the wheels off the ground and the vehicle disarmed, drive each motor on its own and
note which wheel turns. `MSP_SET_MOTOR` writes `motor_disarmed[]`, which `mixTable()`
copies straight to `motor[]` while disarmed, so one motor above `3d_deadband_high` and
the rest at neutral spins exactly one wheel. Work through indices 0..3 in turn.

Note the direction too: any wheel that runs backwards under a forward command has its
leads reversed, which is fixed by swapping that motor's two leads at the terminal block,
not in the mixer.

Then group the indices by side and assign the yaw coefficients:

- left-hand wheels get `yaw = +1.000`
- right-hand wheels get `yaw = -1.000`
- all four keep `throttle = 1.000`, since INAV derives the motor count from the number
  of mixer entries with a non-zero throttle coefficient

The sign convention follows from INAV itself: `rcCommand[YAW]` is negated relative to the
stick in `processPilotAndFailSafeActions()`, and `mixTable()` negates again, so a right
yaw stick gives a positive contribution to motors with a positive yaw coefficient. A
right turn needs the left wheels driven forward, hence left = positive. If the whole
vehicle steers backwards, swap the sign of all four.

### Worked example

A rover that measured out as M1 = front left, M2 = rear right, M3 = front right,
M4 = rear left - so the left side is mixer indices 0 and 3, and the right side is 1
and 2:

```
mmix reset
mmix 0 1.000 0.000 0.000  1.000
mmix 1 1.000 0.000 0.000 -1.000
mmix 2 1.000 0.000 0.000 -1.000
mmix 3 1.000 0.000 0.000  1.000
save
```

## Behaviour and limits

- I2C writes are rate limited to 100 Hz and only issued for motors whose direction or
  duty actually changed.
- Disarm, motor disable and reboot release all four outputs (both TB6612 inputs low,
  PWM off), so the wheels coast.
- If the HAT does not acknowledge at 0x60 during boot, motor init fails, the CLI reports
  `I2C motor HAT not detected`, and arming is blocked.
- If the flight controller stops executing entirely, the PCA9685 keeps its last state.
  Software cannot cover that case; a hardware enable/OE cutoff would be needed.
- Autonomous *forward* throttle is mapped from the conventional cruise/failsafe throttle
  settings into the neutral..2000 range. Autonomous reverse is not supported.
