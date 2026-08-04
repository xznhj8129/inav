#!/usr/bin/env python3
"""Complete the deterministic tunnel refactor after apply_review_fixes.py."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]


def read(path: str) -> str:
    return (ROOT / path).read_text()


def write(path: str, content: str) -> None:
    (ROOT / path).write_text(content)


def replace_once(content: str, old: str, new: str, path: str) -> str:
    count = content.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one match, found {count}")
    return content.replace(old, new, 1)


def patch_ports_header() -> None:
    path = "src/main/mavlink/mavlink_ports.h"
    content = read(path)
    content = replace_once(
        content,
        """void configureMAVLinkTelemetryPort(uint8_t portIndex);
void freeMAVLinkTelemetryPortByIndex(uint8_t portIndex);""",
        """void configureMAVLinkTelemetryPort(uint8_t portIndex);
void freeMAVLinkTelemetryPortByIndex(uint8_t portIndex);
#ifdef USE_MAVLINK_MSP_TUNNEL
void mavlinkResetTunnelPortState(uint8_t portIndex);
#endif""",
        path,
    )
    write(path, content)


def patch_ports_source() -> None:
    path = "src/main/mavlink/mavlink_ports.c"
    content = read(path)
    content = replace_once(
        content,
        """#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_MAVLINK)

static void resetMAVLinkPortRuntimeState(uint8_t portIndex)
""",
        """#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_MAVLINK)

#ifdef USE_MAVLINK_MSP_TUNNEL
void mavlinkResetTunnelPortState(uint8_t portIndex)
{
    resetMspPort(&mavTunnelMspPorts[portIndex], NULL);
    mavTunnelRemoteSystemIds[portIndex] = 0;
    mavTunnelRemoteComponentIds[portIndex] = 0;
}
#endif

static void resetMAVLinkPortRuntimeState(uint8_t portIndex)
""",
        path,
    )
    content = replace_once(
        content,
        """    memset(&state->mlrs, 0, sizeof(state->mlrs));
    resetMspPort(&mavTunnelMspPorts[portIndex], NULL);
    mavTunnelRemoteSystemIds[portIndex] = 0;
    mavTunnelRemoteComponentIds[portIndex] = 0;
""",
        """    memset(&state->mlrs, 0, sizeof(state->mlrs));
#ifdef USE_MAVLINK_MSP_TUNNEL
    mavlinkResetTunnelPortState(portIndex);
#endif
""",
        path,
    )
    write(path, content)


def patch_fc_source() -> None:
    path = "src/main/fc/fc_mavlink.c"
    content = read(path)
    content = replace_once(
        content,
        """#include "mavlink/mavlink_mission.h"
#include "mavlink/mavlink_runtime.h"
""",
        """#include "mavlink/mavlink_mission.h"
#include "mavlink/mavlink_ports.h"
#include "mavlink/mavlink_runtime.h"
""",
        path,
    )
    content = replace_once(
        content,
        """#ifdef USE_MAVLINK_MSP_TUNNEL
static void mavlinkResetTunnelState(uint8_t ingressPortIndex)
{
    resetMspPort(&mavTunnelMspPorts[ingressPortIndex], NULL);
    mavTunnelRemoteSystemIds[ingressPortIndex] = 0;
    mavTunnelRemoteComponentIds[ingressPortIndex] = 0;
}

static void mavlinkReleaseTunnelOwnerIfIdle""",
        """#ifdef USE_MAVLINK_MSP_TUNNEL
static void mavlinkReleaseTunnelOwnerIfIdle""",
        path,
    )
    content = content.replace("mavlinkResetTunnelState(", "mavlinkResetTunnelPortState(")
    write(path, content)


def patch_unit_test_self_insert() -> None:
    path = "src/test/unit/mavlink_unittest.cc"
    content = read(path)
    content = replace_once(
        content,
        """    std::vector<uint8_t> expected = encodeMspV1Reply(testSimpleMspCommand, MSP_RESULT_ACK);
    expected.insert(expected.end(), expected.begin(), expected.end());
    EXPECT_EQ(collectTunnelPayload(parseTxMessages()), expected);
""",
        """    const std::vector<uint8_t> singleReply = encodeMspV1Reply(testSimpleMspCommand, MSP_RESULT_ACK);
    std::vector<uint8_t> expected = singleReply;
    expected.insert(expected.end(), singleReply.begin(), singleReply.end());
    EXPECT_EQ(collectTunnelPayload(parseTxMessages()), expected);
""",
        path,
    )
    write(path, content)


def finalize() -> None:
    for relative in (
        "src/test/mavlink/tunnel/CODEX_REVIEW_FIX_HANDOFF.md",
        "src/test/mavlink/tunnel/apply_review_fixes.py",
        "src/test/mavlink/tunnel/finish_review_fixes.py",
    ):
        path = ROOT / relative
        if path.exists():
            path.unlink()


def main() -> None:
    patch_ports_header()
    patch_ports_source()
    patch_fc_source()
    patch_unit_test_self_insert()
    finalize()
    print("Completed and finalized MSP-over-MAVLink tunnel review fixes.")


if __name__ == "__main__":
    main()
