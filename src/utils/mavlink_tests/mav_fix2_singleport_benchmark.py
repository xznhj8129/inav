#!/usr/bin/env python3
"""
Usage:
  /home/anon/miniconda3/envs/drone/bin/python mydev/branch/mav_multi/mav_fix2_singleport_benchmark.py --config mydev/branch/mav_multi/test_config_mav_fix2.yaml
"""

from __future__ import annotations

import argparse
import contextlib
import math
import socket
import struct
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Tuple

import yaml
from pymavlink import mavutil
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


CHANNELS = [1500, 1500, 900, 1500] + [900] * 14
MAV_CMD_REQUEST_MESSAGE = int(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE)


def load_config(config_path: Path) -> Dict[str, Any]:
    return yaml.safe_load(config_path.read_text(encoding="utf-8"))


def wait_for_tcp_port(host: str, port: int, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                return
        except (ConnectionRefusedError, socket.timeout, OSError):
            time.sleep(0.2)
    raise TimeoutError(f"TCP port {host}:{port} did not become available within {timeout_s}s")


def cli_read_until_prompt(cli_socket: socket.socket) -> str:
    data = b""
    while b"\n# " not in data:
        chunk = cli_socket.recv(65536)
        if not chunk:
            raise ConnectionError("CLI socket closed before prompt")
        data += chunk
    return data.decode("utf-8", errors="replace")


def run_cli_commands(host: str, port: int, commands: List[str]) -> None:
    with socket.create_connection((host, port), timeout=5.0) as cli_socket:
        cli_socket.settimeout(3.0)
        cli_socket.sendall(b"#\n")
        cli_read_until_prompt(cli_socket)
        for command in commands:
            cli_socket.sendall(command.encode("utf-8") + b"\n")
            if command == "save":
                break
            cli_read_until_prompt(cli_socket)
    time.sleep(1.0)


def build_serial_command(index: int, function_mask: int, baud: int) -> str:
    return f"serial {index} {function_mask} {baud} {baud} 0 {baud}"


def build_cli_commands(config: Dict[str, Any]) -> List[str]:
    cli_cfg = config["cli"]
    rc_baud = int(cli_cfg["rc_baud"])
    commands = [
        "feature TELEMETRY",
        "set receiver_type = SERIAL",
        "set serialrx_provider = MAVLINK",
        build_serial_command(0, 1, 115200),  # UART1 MSP
        build_serial_command(1, 256, rc_baud),  # UART2 MAVLINK single port (RC-capable)
        build_serial_command(2, 0, rc_baud),
        build_serial_command(3, 0, rc_baud),
        "set mavlink_version = 2",
        f"set mavlink_port1_ext_status_rate = {int(cli_cfg['ext_status_rate_hz'])}",
        f"set mavlink_port1_rc_chan_rate = {int(cli_cfg['rc_chan_rate_hz'])}",
        f"set mavlink_port1_pos_rate = {int(cli_cfg['pos_rate_hz'])}",
        f"set mavlink_port1_extra1_rate = {int(cli_cfg['extra1_rate_hz'])}",
        f"set mavlink_port1_extra2_rate = {int(cli_cfg['extra2_rate_hz'])}",
        f"set mavlink_port1_extra3_rate = {int(cli_cfg['extra3_rate_hz'])}",
        "save",
    ]
    return commands


def run_workload(
    config: Dict[str, Any],
    sitl_pid: int,
    duration_s: float,
    command_rate_hz: float,
    rc_tx_hz_override: float | None = None,
) -> Dict[str, Any]:
    tests_cfg = config["tests"]
    ports_cfg = config["ports"]
    rc_port = int(ports_cfg["rc"])
    msp_port = int(ports_cfg["msp"])

    wait_for_tcp_port("127.0.0.1", rc_port, float(tests_cfg["port_ready_timeout_s"]))
    wait_for_tcp_port("127.0.0.1", msp_port, float(tests_cfg["port_ready_timeout_s"]))
    msp_enabled = msp_port != rc_port

    heartbeat_expected_hz = float(tests_cfg["heartbeat_expected_hz"])
    rc_expected_hz = float(tests_cfg["rc_expected_hz"])
    rc_tx_hz = float(tests_cfg["rc_tx_hz"]) if rc_tx_hz_override is None else float(rc_tx_hz_override)
    msp_poll_hz = float(tests_cfg["msp_poll_hz"])
    msp_request_timeout_s = float(tests_cfg["msp_request_timeout_s"])
    command_message_id = int(tests_cfg["stress_command_message_id"])
    target_system = int(tests_cfg["rc_target_system"])
    target_component = int(tests_cfg["rc_target_component"])

    listener = mavutil.mavlink_connection(
        f"tcp:127.0.0.1:{rc_port}",
        source_system=170,
        source_component=190,
        autoreconnect=True,
    )
    sender = mavutil.mavlink_connection(
        f"tcp:127.0.0.1:{rc_port}",
        source_system=239,
        source_component=191,
        autoreconnect=True,
    )

    # Prime telemetry stream startup from GCS side.
    handshake_deadline = time.monotonic() + 3.0
    while time.monotonic() < handshake_deadline:
        sender.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        msg = listener.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2)
        if msg is not None:
            target_system = int(msg.get_srcSystem())
            target_component = int(msg.get_srcComponent())
            break

    hb_count = 0
    hb_est_lost = 0
    last_hb_t: float | None = None
    rc_count = 0
    rc_est_lost = 0
    last_rc_t: float | None = None
    rc_mismatch = 0
    command_sent = 0
    command_ack_total = 0
    command_ack_ok = 0
    msp_success = 0
    msp_fail = 0
    msp_mismatch = 0
    rc_sent = 0
    fc_cpu_load_samples: List[float] = []
    fc_cycle_time_samples: List[float] = []
    fc_status_fail = 0
    fc_status_last_error = ""

    rc_period_s = 1.0 / rc_tx_hz
    next_rc_send_t = time.monotonic()
    command_period_s = 0.0 if command_rate_hz <= 0 else (1.0 / command_rate_hz)
    next_command_send_t = next_rc_send_t
    next_heartbeat_send_t = next_rc_send_t

    prev_sample_t = time.monotonic()
    next_resource_sample_t = prev_sample_t + 1.0
    next_msp_poll_t = prev_sample_t + (1.0 / msp_poll_hz)
    workload_end_t = prev_sample_t + duration_s

    with (MSPApi(tcp_endpoint=f"127.0.0.1:{msp_port}") if msp_enabled else contextlib.nullcontext()) as msp_api:
        if msp_enabled:
            msp_api._serial._max_retries = 1
            msp_api._serial._reconnect_delay = 0.05
        while time.monotonic() < workload_end_t:
            now = time.monotonic()

            if now >= next_heartbeat_send_t:
                listener.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                sender.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                next_heartbeat_send_t += 1.0
                if next_heartbeat_send_t < now:
                    next_heartbeat_send_t = now + 1.0

            if now >= next_rc_send_t:
                sender.mav.rc_channels_override_send(target_system, target_component, *CHANNELS[:8])
                rc_sent += 1
                next_rc_send_t += rc_period_s
                if next_rc_send_t < now:
                    next_rc_send_t = now + rc_period_s

            if command_period_s > 0 and now >= next_command_send_t:
                sender.mav.command_long_send(
                    target_system,
                    target_component,
                    MAV_CMD_REQUEST_MESSAGE,
                    0,
                    float(command_message_id),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                command_sent += 1
                next_command_send_t += command_period_s
                if next_command_send_t < now:
                    next_command_send_t = now + command_period_s

            while True:
                try:
                    msg = listener.recv_match(blocking=False)
                except (ConnectionRefusedError, OSError):
                    wait_for_tcp_port("127.0.0.1", rc_port, float(tests_cfg["port_ready_timeout_s"]))
                    break
                if msg is None:
                    break
                msg_type = msg.get_type()
                if msg_type == "HEARTBEAT":
                    target_system = int(msg.get_srcSystem())
                    target_component = int(msg.get_srcComponent())
                    hb_count += 1
                    if last_hb_t is not None:
                        dt = now - last_hb_t
                        hb_est_lost += max(int(math.floor(dt * heartbeat_expected_hz) - 1), 0)
                    last_hb_t = now
                elif msg_type == "RC_CHANNELS":
                    rc_count += 1
                    if last_rc_t is not None:
                        dt = now - last_rc_t
                        rc_est_lost += max(int(math.floor(dt * rc_expected_hz) - 1), 0)
                    last_rc_t = now
                    if (
                        abs(int(msg.chan1_raw) - CHANNELS[0]) > 20
                        or abs(int(msg.chan2_raw) - CHANNELS[1]) > 20
                        or abs(int(msg.chan3_raw) - CHANNELS[2]) > 20
                        or abs(int(msg.chan4_raw) - CHANNELS[3]) > 20
                    ):
                        rc_mismatch += 1
                elif msg_type == "COMMAND_ACK":
                    if int(msg.command) == MAV_CMD_REQUEST_MESSAGE:
                        command_ack_total += 1
                        if int(msg.result) == int(mavutil.mavlink.MAV_RESULT_ACCEPTED):
                            command_ack_ok += 1

            if msp_enabled and now >= next_msp_poll_t:
                try:
                    _, payload = msp_api._request_raw(InavMSP.MSP_RC, timeout=msp_request_timeout_s)
                    if len(payload) % 2:
                        raise ValueError("MSP RC payload length must be even")
                    channel_count = len(payload) // 2
                    channels = list(struct.unpack(f"<{channel_count}H", payload))
                    msp_success += 1
                    if (
                        abs(channels[0] - CHANNELS[0]) > 40
                        or abs(channels[1] - CHANNELS[1]) > 40
                        or abs(channels[2] - CHANNELS[2]) > 40
                        or abs(channels[3] - CHANNELS[3]) > 40
                    ):
                        msp_mismatch += 1
                except (SerialException, RuntimeError, TimeoutError, ConnectionRefusedError):
                    msp_fail += 1
                next_msp_poll_t += 1.0 / msp_poll_hz

            if now >= next_resource_sample_t:
                if msp_enabled:
                    try:
                        status = msp_api.get_inav_status()
                        fc_cpu_load_samples.append(float(status["cpuLoad"]))
                        fc_cycle_time_samples.append(float(status["cycleTime"]))
                    except (SerialException, RuntimeError, TimeoutError, ConnectionRefusedError, KeyError, ValueError) as exc:
                        fc_status_fail += 1
                        fc_status_last_error = str(exc)
                prev_sample_t = now
                next_resource_sample_t += 1.0

            time.sleep(0.001)

    if msp_enabled and not fc_cpu_load_samples:
        raise RuntimeError(f"No MSP2_INAV_STATUS samples collected; last_error={fc_status_last_error} fail_count={fc_status_fail}")

    listener.close()
    sender.close()

    hb_total = hb_count + hb_est_lost
    rc_total = rc_count + rc_est_lost
    return {
        "duration_s": duration_s,
        "heartbeat_count": hb_count,
        "heartbeat_est_lost": hb_est_lost,
        "heartbeat_loss_rate": (hb_est_lost / hb_total) if hb_total > 0 else 0.0,
        "rc_count": rc_count,
        "rc_est_lost": rc_est_lost,
        "rc_loss_rate": (rc_est_lost / rc_total) if rc_total > 0 else 0.0,
        "rc_mismatch": rc_mismatch,
        "rc_mismatch_rate": (rc_mismatch / max(rc_count, 1)),
        "msp_success": msp_success,
        "msp_fail": msp_fail,
        "msp_failure_rate": (msp_fail / max(msp_success + msp_fail, 1)),
        "msp_mismatch": msp_mismatch,
        "msp_mismatch_rate": (msp_mismatch / max(msp_success, 1)),
        "command_sent": command_sent,
        "command_ack_total": command_ack_total,
        "command_ack_ok": command_ack_ok,
        "command_ack_failure_rate": ((command_sent - command_ack_total) / command_sent) if command_sent > 0 else 0.0,
        "command_ack_non_ok_rate": ((command_ack_total - command_ack_ok) / command_ack_total) if command_ack_total > 0 else 0.0,
        "rc_sent": rc_sent,
        "rc_sent_hz": (rc_sent / max(duration_s, 1e-6)),
        "fc_cpu_load_avg_pct": (sum(fc_cpu_load_samples) / len(fc_cpu_load_samples)) if fc_cpu_load_samples else 0.0,
        "fc_cpu_load_max_pct": max(fc_cpu_load_samples) if fc_cpu_load_samples else 0.0,
        "fc_cycle_time_avg_us": (sum(fc_cycle_time_samples) / len(fc_cycle_time_samples)) if fc_cycle_time_samples else 0.0,
        "fc_cycle_time_max_us": max(fc_cycle_time_samples) if fc_cycle_time_samples else 0.0,
        "fc_status_samples": len(fc_cpu_load_samples),
    }


def write_report(config: Dict[str, Any], baseline: Dict[str, Any], stress: Dict[str, Any]) -> None:
    out_path = Path(config["output"]["testing_md"])
    branch_name = str(config.get("meta", {}).get("branch_name", "unknown"))
    timestamp = datetime.now(timezone.utc).isoformat()
    lines: List[str] = []
    lines.append(f"# MAVLink Single-Port Baseline (`{branch_name}`)")
    lines.append("")
    lines.append(f"- Generated: `{timestamp}`")
    lines.append(f"- Branch: `{branch_name}`")
    lines.append(f"- SITL binary: `{config['sitl']['binary']}`")
    lines.append(f"- EEPROM: `{config['sitl']['eeprom_path']}`")
    lines.append("- MSP kept on UART1 by design (`serial 0 ... functionMask=1`).")
    lines.append("- CLI under test sets `receiver_type=SERIAL` and `serialrx_provider=MAVLINK`.")
    lines.append(
        f"- CLI baud config: `rc_baud={config['cli']['rc_baud']}`, "
        f"`telemetry_baud={config['cli'].get('telemetry_baud', config['cli']['rc_baud'])}`."
    )
    lines.append("")
    lines.append("## Baseline: Single MAVLink RC+Telemetry Port")
    lines.append("")
    lines.append(
        f"- Duration: `{baseline['duration_s']}s` | FC cpuLoad avg/max: `{baseline['fc_cpu_load_avg_pct']:.2f}% / {baseline['fc_cpu_load_max_pct']:.2f}%` | "
        f"FC cycleTime avg/max: `{baseline['fc_cycle_time_avg_us']:.1f} / {baseline['fc_cycle_time_max_us']:.1f}` us | "
        f"Status samples: `{baseline['fc_status_samples']}`"
    )
    lines.append(
        f"- MSP success/fail: `{baseline['msp_success']}/{baseline['msp_fail']}` | MSP failure rate: `{baseline['msp_failure_rate']:.4f}` | "
        f"MSP RC mismatch rate: `{baseline['msp_mismatch_rate']:.4f}`"
    )
    lines.append(f"- RC override send rate: `{baseline['rc_sent_hz']:.2f} Hz` (`{baseline['rc_sent']}` packets)")
    lines.append(
        f"- HB count/loss: `{baseline['heartbeat_count']}/{baseline['heartbeat_est_lost']}` | HB loss rate: `{baseline['heartbeat_loss_rate']:.4f}`"
    )
    lines.append(
        f"- RC count/loss: `{baseline['rc_count']}/{baseline['rc_est_lost']}` | RC loss rate: `{baseline['rc_loss_rate']:.4f}` | "
        f"RC mismatch rate: `{baseline['rc_mismatch_rate']:.4f}`"
    )
    lines.append("")
    lines.append("## Stress: Single Port Overload")
    lines.append("")
    lines.append(
        f"- Command load: `{config['tests']['stress_rate_hz']} req/s` for `{stress['duration_s']}s` | "
        f"FC cpuLoad avg/max: `{stress['fc_cpu_load_avg_pct']:.2f}% / {stress['fc_cpu_load_max_pct']:.2f}%` | "
        f"FC cycleTime avg/max: `{stress['fc_cycle_time_avg_us']:.1f} / {stress['fc_cycle_time_max_us']:.1f}` us | "
        f"Status samples: `{stress['fc_status_samples']}`"
    )
    lines.append(
        f"- MSP success/fail: `{stress['msp_success']}/{stress['msp_fail']}` | MSP failure rate: `{stress['msp_failure_rate']:.4f}` | "
        f"MSP RC mismatch rate: `{stress['msp_mismatch_rate']:.4f}`"
    )
    lines.append(f"- RC override send rate: `{stress['rc_sent_hz']:.2f} Hz` (`{stress['rc_sent']}` packets)")
    lines.append(
        f"- HB count/loss: `{stress['heartbeat_count']}/{stress['heartbeat_est_lost']}` | HB loss rate: `{stress['heartbeat_loss_rate']:.4f}`"
    )
    lines.append(
        f"- RC count/loss: `{stress['rc_count']}/{stress['rc_est_lost']}` | RC loss rate: `{stress['rc_loss_rate']:.4f}` | "
        f"RC mismatch rate: `{stress['rc_mismatch_rate']:.4f}`"
    )
    lines.append(
        f"- Command sent/ack: `{stress['command_sent']}/{stress['command_ack_total']}` | "
        f"ack failure rate: `{stress['command_ack_failure_rate']:.4f}` | non-ok ack rate: `{stress['command_ack_non_ok_rate']:.4f}`"
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run single-port MAVLink benchmark and write a baseline markdown report.")
    parser.add_argument("--config", required=True, help="YAML configuration path")
    args = parser.parse_args()

    config = load_config(Path(args.config))
    sitl_bin = Path(config["sitl"]["binary"])
    sitl_workdir = Path(config["sitl"]["workdir"])
    sitl_eeprom = str(config["sitl"]["eeprom_path"])
    sitl_log = Path(config["sitl"]["runtime_log"])
    sitl_log.parent.mkdir(parents=True, exist_ok=True)

    log_handle = sitl_log.open("w", encoding="utf-8")
    sitl_proc = subprocess.Popen(
        [str(sitl_bin), f"--path={sitl_eeprom}"],
        cwd=str(sitl_workdir),
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        wait_for_tcp_port("127.0.0.1", int(config["ports"]["msp"]), float(config["tests"]["port_ready_timeout_s"]))
        run_cli_commands("127.0.0.1", int(config["ports"]["msp"]), build_cli_commands(config))
        wait_for_tcp_port("127.0.0.1", int(config["ports"]["msp"]), float(config["tests"]["port_ready_timeout_s"]))
        wait_for_tcp_port("127.0.0.1", int(config["ports"]["rc"]), float(config["tests"]["port_ready_timeout_s"]))

        baseline = run_workload(
            config=config,
            sitl_pid=sitl_proc.pid,
            duration_s=float(config["tests"]["baseline_duration_s"]),
            command_rate_hz=0.0,
        )
        stress = run_workload(
            config=config,
            sitl_pid=sitl_proc.pid,
            duration_s=float(config["tests"]["stress_duration_s"]),
            command_rate_hz=float(config["tests"]["stress_rate_hz"]),
            rc_tx_hz_override=float(config["tests"]["rc_tx_hz_max"]),
        )
        write_report(config, baseline, stress)
        print(f"report_written={config['output']['testing_md']}", flush=True)
    finally:
        sitl_proc.terminate()
        try:
            sitl_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            sitl_proc.kill()
            sitl_proc.wait(timeout=5.0)
        log_handle.close()
