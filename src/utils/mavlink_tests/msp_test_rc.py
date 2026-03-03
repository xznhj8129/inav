#!/usr/bin/env python3
"""
Usage:
  conda run -n drone python mydev/branch/mav_multi/msp_test_rc.py --msp-endpoint 127.0.0.1:5760 --duration 20
  conda run -n drone python mydev/branch/mav_multi/msp_test_rc.py --msp-endpoint 127.0.0.1:5760 --poll-hz 25 --roll MID --pitch MID --yaw MID --throttle LOW
"""

from __future__ import annotations

import argparse
import struct
import time
from collections import Counter
from pathlib import Path
from typing import List

from serial.serialutil import SerialException

try:
    from mspapi2.msp_api import MSPApi
    from mspapi2.lib import InavMSP
except ModuleNotFoundError:
    repo_root_guess = Path(__file__).resolve().parents[3]
    mspapi2_repo = repo_root_guess.parent / "mspapi2"
    if mspapi2_repo.exists():
        import sys

        sys.path.insert(0, str(mspapi2_repo))
    from mspapi2.msp_api import MSPApi
    from mspapi2.lib import InavMSP


CHANNEL_ORDER = [
    "roll",
    "pitch",
    "throttle",
    "yaw",
    "ch5",
    "ch6",
    "ch7",
    "ch8",
    "ch9",
    "ch10",
    "ch11",
    "ch12",
    "ch13",
    "ch14",
    "ch15",
    "ch16",
    "ch17",
    "ch18",
]
DEFAULT_CHANNELS = [900] * 18
DEFAULT_CHANNELS[0] = 1500
DEFAULT_CHANNELS[1] = 1500
DEFAULT_CHANNELS[3] = 1500
LEVEL_TO_PWM = {"LOW": 900, "MID": 1500, "HIGH": 2100}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Poll MSP_RC and monitor RC values against expected channels.")
    parser.add_argument("--msp-endpoint", required=True, help='MSP endpoint, e.g. "127.0.0.1:5760"')
    parser.add_argument("--poll-hz", type=float, default=25.0, help="MSP_RC poll rate in Hz")
    parser.add_argument("--timeout-s", type=float, default=0.2, help="Per MSP request timeout")
    parser.add_argument("--duration", type=float, default=0.0, help="Test duration in seconds, 0 means run forever")
    parser.add_argument("--echo-tolerance", type=int, default=40, help="Allowed absolute PWM error per channel")
    for channel_name in CHANNEL_ORDER:
        parser.add_argument(f"--{channel_name}", default=None, help="LOW/MID/HIGH or integer PWM")
    args = parser.parse_args()

    expected_channels: List[int] = list(DEFAULT_CHANNELS)
    for idx, channel_name in enumerate(CHANNEL_ORDER):
        raw_value = getattr(args, channel_name)
        if raw_value is None:
            continue
        level_name = raw_value.upper()
        if level_name in LEVEL_TO_PWM:
            expected_channels[idx] = LEVEL_TO_PWM[level_name]
        else:
            expected_channels[idx] = int(raw_value)

    print(
        f"msp_endpoint={args.msp_endpoint} poll_hz={args.poll_hz} duration={args.duration} "
        f"timeout_s={args.timeout_s} expected_channels={expected_channels}",
        flush=True,
    )

    poll_period_s = 1.0 / args.poll_hz
    success_count = 0
    fail_count = 0
    mismatch_count = 0
    mismatch_hist = Counter()
    last_success_time = 0.0
    start_t = time.monotonic()
    next_poll_t = start_t
    next_report_t = start_t + 1.0

    with MSPApi(tcp_endpoint=args.msp_endpoint) as msp_api:
        msp_api._serial._max_retries = 1
        msp_api._serial._reconnect_delay = 0.05
        while True:
            now = time.monotonic()
            if args.duration > 0 and (now - start_t) >= args.duration:
                break

            if now >= next_poll_t:
                try:
                    _, payload = msp_api._request_raw(InavMSP.MSP_RC, timeout=args.timeout_s)
                    if len(payload) % 2:
                        raise ValueError("MSP RC payload length must be even")
                    channel_count = len(payload) // 2
                    observed_channels = list(struct.unpack(f"<{channel_count}H", payload))
                    success_count += 1
                    last_success_time = now
                    for i in range(min(channel_count, len(expected_channels))):
                        error = abs(observed_channels[i] - expected_channels[i])
                        if error > args.echo_tolerance:
                            mismatch_count += 1
                            mismatch_hist[i + 1] += 1
                except (SerialException, RuntimeError, TimeoutError, ConnectionRefusedError):
                    fail_count += 1
                next_poll_t += poll_period_s
                if next_poll_t < now:
                    next_poll_t = now + poll_period_s

            if now >= next_report_t:
                last_ok_age_s = -1.0 if last_success_time == 0.0 else (now - last_success_time)
                total = success_count + fail_count
                print(
                    f"status_s={now - start_t:.1f} ok={success_count} fail={fail_count} fail_rate={fail_count / max(total, 1):.6f} "
                    f"mismatch={mismatch_count} mismatch_rate={mismatch_count / max(success_count, 1):.6f} "
                    f"last_ok_age_s={last_ok_age_s:.3f} mismatch_hist={dict(mismatch_hist)}",
                    flush=True,
                )
                next_report_t += 1.0

            time.sleep(0.001)

    elapsed = max(time.monotonic() - start_t, 1e-6)
    total = success_count + fail_count
    print(
        f"summary elapsed_s={elapsed:.2f} ok={success_count} fail={fail_count} fail_rate={fail_count / max(total, 1):.6f} "
        f"mismatch={mismatch_count} mismatch_rate={mismatch_count / max(success_count, 1):.6f} poll_hz={success_count / elapsed:.2f}",
        flush=True,
    )
