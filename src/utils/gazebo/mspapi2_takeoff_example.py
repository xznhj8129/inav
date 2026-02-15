#!/usr/bin/env python3
"""
Usage:
  python src/utils/gazebo/mspapi2_takeoff_example.py --tcp localhost:5761
"""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import sys
import time

try:
    from mspapi2.msp_api import MSPApi
    from mspapi2.lib import InavMSP
except ModuleNotFoundError:
    script_dir = Path(__file__).resolve().parent
    msp_repo = script_dir.parent.parent / "mspapi2"
    if not msp_repo.exists():
        msp_repo = script_dir.parent.parent.parent / "mspapi2"
    if msp_repo.exists():
        sys.path.insert(0, str(msp_repo))
    from mspapi2.msp_api import MSPApi
    from mspapi2.lib import InavMSP


ROLL_IDX = 0
PITCH_IDX = 1
THROTTLE_IDX = 2
YAW_IDX = 3
CH5_IDX = 4
CH6_IDX = 5
CH7_IDX = 6
MIN_CHANNEL_COUNT = 8
TARGET_CHANNEL_COUNT = 16

PWM_LOW = 910
PWM_MID = 1500
PWM_HIGH = 2099

ROLL_CENTER = 1500
PITCH_CENTER = 1500
YAW_CENTER = 1500
ARM_LOW = 1000
ARM_HIGH = 2099
AUX_LOW = 910
THROTTLE_MIN = 1000
THROTTLE_MID = 1500
THROTTLE_TAKEOFF = 2099
WAIT_BEFORE_ARM_S = 5.0
WAIT_AFTER_ARM_S = 3.0
ARMING_TIMEOUT_S = 15.0
SEQUENCE_TIMEOUT_S = 120.0
RC_HZ = 200.0
STATUS_PERIOD_S = 0.2
READ_TIMEOUT_MS = 10.0
WRITE_TIMEOUT_MS = 20.0
TARGET_ALTITUDE_M = 5.0
ALTITUDE_POLL_HZ = 20.0
STATUS_POLL_HZ = 2.0
ATTITUDE_POLL_HZ = 2.0
IMU_POLL_HZ = 20.0
STICK_FULL_SCALE_PWM = 500.0
JIGGLE_DEFLECTION = 0.5
JIGGLE_STEP_SECONDS = 1.0
JIGGLE_PWM = int(round(JIGGLE_DEFLECTION * STICK_FULL_SCALE_PWM))
ALTHOLD_THROTTLE = THROTTLE_MID
JIGGLE_START_DELAY_S = 0.5
ALTHOLD_PRESETTLE_S = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MSPApi takeoff example via CH5 arm + 5m climb + ANGLE+ALTHOLD jiggle sequence.")
    parser.add_argument("--port", default=None, help="Serial device path (ignored when --tcp/--udp is used)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--tcp", metavar="HOST:PORT", help="Connect via TCP (example: localhost:5760)")
    parser.add_argument("--udp", metavar="HOST:PORT", help="Connect via UDP (example: localhost:27072)")
    parser.add_argument("--force-mspv2", action="store_true", help="Force MSPv2 framing")
    parser.add_argument("--takeoff-throttle", type=int, default=THROTTLE_TAKEOFF, help="Takeoff PWM value while climbing")
    parser.add_argument("--target-altitude-m", type=float, default=TARGET_ALTITUDE_M, help="Relative altitude target in meters")
    parser.add_argument("--status-period", type=float, default=STATUS_PERIOD_S, help="Seconds between telemetry debug prints")
    parser.add_argument("--altitude-poll-hz", type=float, default=ALTITUDE_POLL_HZ, help="Altitude polling frequency in Hz")
    parser.add_argument("--status-poll-hz", type=float, default=STATUS_POLL_HZ, help="Status polling frequency in Hz")
    parser.add_argument("--attitude-poll-hz", type=float, default=ATTITUDE_POLL_HZ, help="Attitude polling frequency in Hz")
    parser.add_argument("--imu-poll-hz", type=float, default=IMU_POLL_HZ, help="Raw IMU polling frequency in Hz")
    parser.add_argument("--althold-throttle", type=int, default=ALTHOLD_THROTTLE, help="Throttle PWM used while ALTHOLD is active")
    parser.add_argument("--jiggle-start-delay-s", type=float, default=JIGGLE_START_DELAY_S, help="Seconds to wait after ALTHOLD engages before jiggle")
    parser.add_argument("--read-timeout-ms", type=float, default=READ_TIMEOUT_MS, help="MSP read timeout in milliseconds")
    parser.add_argument("--write-timeout-ms", type=float, default=WRITE_TIMEOUT_MS, help="MSP write timeout in milliseconds")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.tcp and args.udp:
        raise ValueError("Provide only one of --tcp or --udp")
    if not args.tcp and not args.udp:
        args.tcp = "localhost:5761"

    port = None if (args.tcp or args.udp) else args.port

    with MSPApi(
        port=port,
        baudrate=args.baudrate,
        read_timeout_ms=args.read_timeout_ms,
        write_timeout_ms=args.write_timeout_ms,
        tcp_endpoint=args.tcp,
        udp_endpoint=args.udp,
        force_msp_v2=args.force_mspv2,
    ) as api:
        channels = [AUX_LOW] * TARGET_CHANNEL_COUNT
        if len(channels) < MIN_CHANNEL_COUNT:
            raise ValueError(f"RC frame too short: {len(channels)} channels")

        for idx in range(CH5_IDX, len(channels)):
            channels[idx] = AUX_LOW
        channels[ROLL_IDX] = ROLL_CENTER
        channels[PITCH_IDX] = PITCH_CENTER
        channels[YAW_IDX] = YAW_CENTER
        channels[THROTTLE_IDX] = THROTTLE_MIN
        channels[CH5_IDX] = PWM_LOW
        rc_period = 1.0 / RC_HZ if RC_HZ > 0 else 0.02
        next_rc_send = time.monotonic()
        start = time.monotonic()
        arm_at = WAIT_BEFORE_ARM_S
        last_status_print = start
        tx_total = 0
        tx_window = 0
        last_tx_window = start
        phase = "prearm"
        current_baro_alt = None
        takeoff_reference_baro = None
        climb_reached_at = None
        althold_request_at = None
        althold_started_at = None
        jiggle_started_at = None
        jiggle_step_index = -1
        disarm_started_at = None
        armed_since = None
        next_altitude_poll = start
        altitude_poll_period = 1.0 / args.altitude_poll_hz
        next_status_poll = start
        status_poll_period = 1.0 / args.status_poll_hz
        next_attitude_poll = start
        attitude_poll_period = 1.0 / args.attitude_poll_hz
        next_imu_poll = start
        imu_poll_period = 1.0 / args.imu_poll_hz
        last_altitude = None
        last_status = None
        last_attitude = None
        last_imu = None
        is_armed = False
        jiggle_steps = [
            ("roll_left", -JIGGLE_PWM, 0),
            ("neutral_1", 0, 0),
            ("roll_right", JIGGLE_PWM, 0),
            ("neutral_2", 0, 0),
            ("pitch_forward", 0, JIGGLE_PWM),
            ("neutral_3", 0, 0),
            ("pitch_back", 0, -JIGGLE_PWM),
            ("neutral_4", 0, 0),
        ]
        print(f"Streaming RC via UART at {RC_HZ:.0f}Hz on {TARGET_CHANNEL_COUNT} channels.", flush=True)

        while True:
            now = time.monotonic()
            elapsed = now - start
            current_relative_alt = None
            if takeoff_reference_baro is not None and current_baro_alt is not None:
                current_relative_alt = current_baro_alt - takeoff_reference_baro
            if elapsed > SEQUENCE_TIMEOUT_S:
                raise RuntimeError(f"Timeout after {SEQUENCE_TIMEOUT_S:.1f}s while phase={phase}")
            arming_flag_names = set()
            active_mode_names = set()
            if last_status is not None:
                arming_flag_names = {flag.name for flag in last_status["armingFlags"]}
                active_mode_names = {mode.name for mode in last_status["activeModes"]}
            sensors_calibrating = "ARMING_DISABLED_SENSORS_CALIBRATING" in arming_flag_names
            arm_switch_blocked = "ARMING_DISABLED_ARM_SWITCH" in arming_flag_names
            althold_active = "BOXNAVALTHOLD" in active_mode_names
            channels[CH6_IDX] = PWM_MID
            channels[CH7_IDX] = AUX_LOW
            channels[ROLL_IDX] = ROLL_CENTER
            channels[PITCH_IDX] = PITCH_CENTER

            if disarm_started_at is not None:
                phase = "disarm"
                channels[CH6_IDX] = PWM_MID
                channels[CH7_IDX] = AUX_LOW
                channels[CH5_IDX] = ARM_LOW
                channels[THROTTLE_IDX] = THROTTLE_MIN
            elif elapsed < arm_at or sensors_calibrating:
                phase = "prearm"
                channels[CH5_IDX] = ARM_LOW
                channels[THROTTLE_IDX] = THROTTLE_MIN
            elif not is_armed:
                phase = "arming_wait"
                if arm_switch_blocked:
                    channels[CH5_IDX] = ARM_LOW
                    channels[THROTTLE_IDX] = THROTTLE_MIN
                    arm_at = elapsed + 0.5
                else:
                    channels[CH5_IDX] = ARM_HIGH
                    channels[THROTTLE_IDX] = THROTTLE_MIN
                if elapsed > (arm_at + ARMING_TIMEOUT_S):
                    raise RuntimeError(f"Arming timeout after {ARMING_TIMEOUT_S:.1f}s. Last arming flags: {last_status['armingFlags'] if last_status else None}")
            elif armed_since is None or (now - armed_since) < WAIT_AFTER_ARM_S:
                phase = "armed_idle"
                channels[CH5_IDX] = ARM_HIGH
                channels[THROTTLE_IDX] = THROTTLE_MIN
            elif climb_reached_at is None:
                phase = "full_throttle_climb"
                channels[CH5_IDX] = ARM_HIGH
                channels[THROTTLE_IDX] = args.takeoff_throttle
                if takeoff_reference_baro is None and current_baro_alt is not None:
                    takeoff_reference_baro = current_baro_alt
                    print(f"Takeoff reference baro altitude: {takeoff_reference_baro:.2f}m", flush=True)
                if takeoff_reference_baro is not None and current_baro_alt is not None:
                    relative_alt = current_baro_alt - takeoff_reference_baro
                    variometer = 0.0 if last_altitude is None else last_altitude["variometer"]
                    if relative_alt >= args.target_altitude_m:
                        climb_reached_at = now
                        althold_request_at = now
                        channels[THROTTLE_IDX] = args.althold_throttle
                        print(
                            f"CLIMB_TARGET_REACHED t={elapsed:.2f}s rel_alt={relative_alt:.2f}m "
                            f"baro_alt={current_baro_alt:.2f}m thr={channels[THROTTLE_IDX]}",
                            flush=True,
                        )
                        print(
                            f"ALTHOLD_PRESETTLE t={elapsed:.2f}s rel_alt={relative_alt:.2f} variometer={variometer:.2f}",
                            flush=True,
                        )
            elif althold_started_at is None:
                phase = "althold_presettle"
                channels[CH5_IDX] = ARM_HIGH
                channels[CH6_IDX] = PWM_MID
                channels[CH7_IDX] = AUX_LOW
                channels[THROTTLE_IDX] = args.althold_throttle
                if (now - althold_request_at) >= ALTHOLD_PRESETTLE_S:
                    channels[CH7_IDX] = PWM_HIGH
                    althold_started_at = now
                    print(
                        f"ALTHOLD_REQUEST t={elapsed:.2f}s rel_alt={current_relative_alt}",
                        flush=True,
                    )
            elif jiggle_started_at is None:
                phase = "althold_hold"
                channels[CH5_IDX] = ARM_HIGH
                channels[CH6_IDX] = PWM_MID
                channels[CH7_IDX] = PWM_HIGH
                channels[THROTTLE_IDX] = args.althold_throttle
                if althold_active and (now - althold_started_at) >= args.jiggle_start_delay_s:
                    jiggle_started_at = now
                    jiggle_step_index = -1
                    print(
                        f"Starting jiggle in ANGLE (CH6={PWM_MID}) + ALTHOLD ACTIVE (CH7={PWM_HIGH}) at throttle {args.althold_throttle}",
                        flush=True,
                    )
            else:
                phase = "jiggle"
                channels[CH5_IDX] = ARM_HIGH
                channels[CH6_IDX] = PWM_MID
                channels[CH7_IDX] = PWM_HIGH
                channels[THROTTLE_IDX] = args.althold_throttle
                jiggle_elapsed = now - jiggle_started_at
                step_index = int(jiggle_elapsed // JIGGLE_STEP_SECONDS)
                if step_index >= len(jiggle_steps):
                    disarm_started_at = now
                else:
                    if step_index != jiggle_step_index:
                        jiggle_step_index = step_index
                        step_name, step_roll, step_pitch = jiggle_steps[step_index]
                        print(
                            f"JIGGLE step={step_name} roll_pwm={ROLL_CENTER + step_roll} pitch_pwm={PITCH_CENTER + step_pitch}",
                            flush=True,
                        )
                    _, step_roll, step_pitch = jiggle_steps[step_index]
                    channels[ROLL_IDX] = ROLL_CENTER + step_roll
                    channels[PITCH_IDX] = PITCH_CENTER + step_pitch

            if now - next_rc_send > 0.5:
                next_rc_send = now
            while now >= next_rc_send:
                payload = struct.pack(f"<{TARGET_CHANNEL_COUNT}H", *channels)
                api._serial.send(int(InavMSP.MSP_SET_RAW_RC), payload)
                tx_total += 1
                tx_window += 1
                next_rc_send += rc_period

            poll_jobs = []
            if now >= next_altitude_poll:
                poll_jobs.append(("altitude", next_altitude_poll))
            if now >= next_status_poll:
                poll_jobs.append(("status", next_status_poll))
            if now >= next_attitude_poll:
                poll_jobs.append(("attitude", next_attitude_poll))
            if now >= next_imu_poll:
                poll_jobs.append(("imu", next_imu_poll))
            if poll_jobs:
                poll_name = min(poll_jobs, key=lambda item: item[1])[0]
                if poll_name == "altitude":
                    last_altitude = api.get_altitude()
                    current_baro_alt = last_altitude["baroAltitude"]
                    next_altitude_poll += altitude_poll_period
                elif poll_name == "status":
                    last_status = api.get_inav_status()
                    is_armed = any(flag.name == "ARMED" for flag in last_status["armingFlags"])
                    if is_armed and armed_since is None:
                        armed_since = now
                    next_status_poll += status_poll_period
                elif poll_name == "attitude":
                    last_attitude = api.get_attitude()
                    next_attitude_poll += attitude_poll_period
                elif poll_name == "imu":
                    last_imu = api.get_imu()
                    next_imu_poll += imu_poll_period

            if now - last_status_print >= args.status_period:
                dt_window = now - last_tx_window
                tx_hz = tx_window / dt_window
                print(
                    f"T={elapsed:.2f}s tx_total={tx_total} tx_hz={tx_hz:.1f} reconnects={api._serial.reconnects} "
                    f"phase={phase} roll={channels[ROLL_IDX]} pitch={channels[PITCH_IDX]} thr={channels[THROTTLE_IDX]} "
                    f"ch5={channels[CH5_IDX]} ch6={channels[CH6_IDX]} ch7={channels[CH7_IDX]}"
                , flush=True)
                if last_status is not None:
                    print(f"  arming_flags={last_status['armingFlags']} active_modes={last_status['activeModes']}", flush=True)
                if last_imu is not None:
                    gx = last_imu["gyro"]["X"]
                    gy = last_imu["gyro"]["Y"]
                    gz = last_imu["gyro"]["Z"]
                    gnorm = (gx * gx + gy * gy + gz * gz) ** 0.5
                    print(f"  gyro_dps={{'X': {gx}, 'Y': {gy}, 'Z': {gz}}} gyro_norm_dps={gnorm:.1f}", flush=True)
                print(f"  attitude={last_attitude} altitude={last_altitude} rel_alt={current_relative_alt}", flush=True)
                last_status_print = now
                last_tx_window = now
                tx_window = 0

            if disarm_started_at is not None and (now - disarm_started_at) >= 1.0:
                break
            time.sleep(0.001)

        print("Takeoff sequence complete.", flush=True)


if __name__ == "__main__":
    main()
