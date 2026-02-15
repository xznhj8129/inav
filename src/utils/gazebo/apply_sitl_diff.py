#!/usr/bin/env python3
"""
Usage:
  python src/utils/gazebo/apply_sitl_diff.py --tcp 127.0.0.1:5761 --diff src/utils/gazebo/gz_sitl_diff.txt
  python src/utils/gazebo/apply_sitl_diff.py --tcp 127.0.0.1:5761 --skip-save
"""

from __future__ import annotations

import argparse
import socket
import time
from pathlib import Path

CLI_PROMPT = b"\n# "
READ_SIZE = 4096
CONNECT_TIMEOUT_S = 5.0
IO_TIMEOUT_S = 2.0
POST_SAVE_READ_S = 2.0
DEFAULT_DIFF_PATH = Path(__file__).resolve().parent / "gz_sitl_diff.txt"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Apply an INAV CLI diff text file to SITL over TCP UART.")
    parser.add_argument("--tcp", required=True, help="SITL UART endpoint in HOST:PORT format, for example 127.0.0.1:5761")
    parser.add_argument("--diff", default=str(DEFAULT_DIFF_PATH), help="Path to CLI diff text file")
    parser.add_argument("--skip-save", action="store_true", help="Skip 'save' command so SITL does not reboot")
    args = parser.parse_args()

    host, port_text = args.tcp.rsplit(":", 1)
    port = int(port_text)
    diff_path = Path(args.diff)
    lines = diff_path.read_text(encoding="utf-8").splitlines()
    commands = []
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            continue
        commands.append(stripped)

    print(f"diff_path={diff_path} command_count={len(commands)} host={host} port={port}", flush=True)

    sock = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT_S)
    sock.settimeout(IO_TIMEOUT_S)
    sock.sendall(b"#\n")

    buffer = b""
    while CLI_PROMPT not in buffer:
        buffer += sock.recv(READ_SIZE)
    print(f"cli_entered_bytes={len(buffer)}", flush=True)

    for idx, command in enumerate(commands, start=1):
        if args.skip_save and command == "save":
            print(f"skip idx={idx}/{len(commands)} cmd={command}", flush=True)
            continue
        print(f"send idx={idx}/{len(commands)} cmd={command}", flush=True)
        sock.sendall(command.encode("utf-8") + b"\n")
        if command == "save":
            deadline = time.time() + POST_SAVE_READ_S
            while time.time() < deadline:
                chunk = sock.recv(READ_SIZE)
                if not chunk:
                    break
            break

        response = b""
        while CLI_PROMPT not in response:
            response += sock.recv(READ_SIZE)
        if response:
            tail = response[-180:].decode("utf-8", errors="replace").replace("\r", "")
            print(f"resp_tail={tail}", flush=True)

    sock.close()
    print("diff_apply_done=1", flush=True)
