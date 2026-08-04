#!/usr/bin/env python3
"""Apply the reviewed MSP-over-MAVLink tunnel refactor.

This is intentionally deterministic. It contains the implementation decisions;
the local runner only applies them and runs builds/tests that require a checkout.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]


def read(path: str) -> str:
    return (ROOT / path).read_text()


def write(path: str, content: str) -> None:
    (ROOT / path).write_text(content)


def replace_once(content: str, old: str, new: str, path: str) -> str:
    count = content.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one literal match, found {count}")
    return content.replace(old, new, 1)


def replace_regex_once(content: str, pattern: str, replacement: str, path: str) -> str:
    updated, count = re.subn(pattern, replacement, content, count=1, flags=re.S)
    if count != 1:
        raise RuntimeError(f"{path}: expected one regex match, found {count}")
    return updated


def patch_msp_serial_header() -> None:
    path = "src/main/msp/msp_serial.h"
    content = read(path)
    content = replace_once(
        content,
        "#define MSP_MAX_HEADER_SIZE     9\n\nstruct serialPort_s;",
        """#define MSP_MAX_HEADER_SIZE     9
#define MSP_MAX_TRAILER_SIZE    2

typedef struct {
    uint8_t header[16];
    uint8_t headerLength;
    const uint8_t *payload;
    uint16_t payloadLength;
    uint8_t trailer[MSP_MAX_TRAILER_SIZE];
    uint8_t trailerLength;
    uint16_t totalLength;
} mspEncodedFrame_t;

struct serialPort_s;""",
        path,
    )
    content = replace_once(
        content,
        "int mspSerialEncodePacket(mspPacket_t *packet, mspVersion_e mspVersion, uint8_t *frameBuf, int frameBufSize);",
        """bool mspSerialPrepareFrame(mspPacket_t *packet, mspVersion_e mspVersion, mspEncodedFrame_t *frame);
int mspSerialFrameCopyRange(const mspEncodedFrame_t *frame, int offset, uint8_t *destination, int destinationSize);""",
        path,
    )
    write(path, content)


def patch_msp_serial_source() -> None:
    path = "src/main/msp/msp_serial.c"
    content = read(path)
    replacement = r'''#define V1_CHECKSUM_STARTPOS 3

bool mspSerialPrepareFrame(mspPacket_t *packet, mspVersion_e mspVersion, mspEncodedFrame_t *frame)
{
    static const uint8_t mspMagic[MSP_VERSION_COUNT] = MSP_VERSION_MAGIC_INITIALIZER;

    if (!packet || !frame || mspVersion >= MSP_VERSION_COUNT) {
        return false;
    }

    const int dataLen = sbufBytesRemaining(&packet->buf);
    if (dataLen < 0 || dataLen > UINT16_MAX) {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->header[0] = '$';
    frame->header[1] = mspMagic[mspVersion];
    frame->header[2] = packet->result == MSP_RESULT_ERROR ? '!' : '>';

    int headerLength = 3;
    int trailerLength = 0;

    if (mspVersion == MSP_V1) {
        mspHeaderV1_t *headerV1 = (mspHeaderV1_t *)&frame->header[headerLength];
        headerLength += sizeof(*headerV1);
        headerV1->cmd = packet->cmd;

        if (dataLen >= JUMBO_FRAME_SIZE_LIMIT) {
            mspHeaderJUMBO_t *headerJumbo = (mspHeaderJUMBO_t *)&frame->header[headerLength];
            headerLength += sizeof(*headerJumbo);
            headerV1->size = JUMBO_FRAME_SIZE_LIMIT;
            headerJumbo->size = dataLen;
        } else {
            headerV1->size = dataLen;
        }

        frame->trailer[trailerLength] = mspSerialChecksumBuf(
            0,
            frame->header + V1_CHECKSUM_STARTPOS,
            headerLength - V1_CHECKSUM_STARTPOS);
        frame->trailer[trailerLength] = mspSerialChecksumBuf(
            frame->trailer[trailerLength],
            sbufPtr(&packet->buf),
            dataLen);
        trailerLength++;
    } else if (mspVersion == MSP_V2_OVER_V1) {
        mspHeaderV1_t *headerV1 = (mspHeaderV1_t *)&frame->header[headerLength];
        headerLength += sizeof(*headerV1);

        mspHeaderV2_t *headerV2 = (mspHeaderV2_t *)&frame->header[headerLength];
        headerLength += sizeof(*headerV2);

        const int v1PayloadSize = sizeof(*headerV2) + dataLen + 1;
        headerV1->cmd = MSP_V2_FRAME_ID;

        if (v1PayloadSize >= JUMBO_FRAME_SIZE_LIMIT) {
            mspHeaderJUMBO_t *headerJumbo = (mspHeaderJUMBO_t *)&frame->header[headerLength];
            headerLength += sizeof(*headerJumbo);
            headerV1->size = JUMBO_FRAME_SIZE_LIMIT;
            headerJumbo->size = v1PayloadSize;
        } else {
            headerV1->size = v1PayloadSize;
        }

        headerV2->flags = packet->flags;
        headerV2->cmd = packet->cmd;
        headerV2->size = dataLen;

        frame->trailer[trailerLength] = crc8_dvb_s2_update(0, (uint8_t *)headerV2, sizeof(*headerV2));
        frame->trailer[trailerLength] = crc8_dvb_s2_update(
            frame->trailer[trailerLength],
            sbufPtr(&packet->buf),
            dataLen);
        trailerLength++;

        frame->trailer[trailerLength] = mspSerialChecksumBuf(
            0,
            frame->header + V1_CHECKSUM_STARTPOS,
            headerLength - V1_CHECKSUM_STARTPOS);
        frame->trailer[trailerLength] = mspSerialChecksumBuf(
            frame->trailer[trailerLength],
            sbufPtr(&packet->buf),
            dataLen);
        frame->trailer[trailerLength] = mspSerialChecksumBuf(
            frame->trailer[trailerLength],
            frame->trailer,
            trailerLength);
        trailerLength++;
    } else if (mspVersion == MSP_V2_NATIVE) {
        mspHeaderV2_t *headerV2 = (mspHeaderV2_t *)&frame->header[headerLength];
        headerLength += sizeof(*headerV2);

        headerV2->flags = packet->flags;
        headerV2->cmd = packet->cmd;
        headerV2->size = dataLen;

        frame->trailer[trailerLength] = crc8_dvb_s2_update(0, (uint8_t *)headerV2, sizeof(*headerV2));
        frame->trailer[trailerLength] = crc8_dvb_s2_update(
            frame->trailer[trailerLength],
            sbufPtr(&packet->buf),
            dataLen);
        trailerLength++;
    } else {
        return false;
    }

    frame->headerLength = headerLength;
    frame->payload = sbufPtr(&packet->buf);
    frame->payloadLength = dataLen;
    frame->trailerLength = trailerLength;
    frame->totalLength = headerLength + dataLen + trailerLength;
    return true;
}

int mspSerialFrameCopyRange(const mspEncodedFrame_t *frame, int offset, uint8_t *destination, int destinationSize)
{
    if (!frame || !destination || offset < 0 || destinationSize <= 0 || offset >= frame->totalLength) {
        return 0;
    }

    int copied = 0;
    while (copied < destinationSize && offset + copied < frame->totalLength) {
        const int position = offset + copied;
        const uint8_t *segment;
        int segmentOffset;
        int segmentRemaining;

        if (position < frame->headerLength) {
            segment = frame->header;
            segmentOffset = position;
            segmentRemaining = frame->headerLength - segmentOffset;
        } else if (position < frame->headerLength + frame->payloadLength) {
            segment = frame->payload;
            segmentOffset = position - frame->headerLength;
            segmentRemaining = frame->payloadLength - segmentOffset;
        } else {
            segment = frame->trailer;
            segmentOffset = position - frame->headerLength - frame->payloadLength;
            segmentRemaining = frame->trailerLength - segmentOffset;
        }

        const int copyLength = MIN(destinationSize - copied, segmentRemaining);
        memcpy(destination + copied, segment + segmentOffset, copyLength);
        copied += copyLength;
    }

    return copied;
}

static int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet, mspVersion_e mspVersion)
{
    mspEncodedFrame_t frame;
    if (!mspSerialPrepareFrame(packet, mspVersion, &frame)) {
        return 0;
    }

    return mspSerialSendFrame(
        msp,
        frame.header,
        frame.headerLength,
        frame.payload,
        frame.payloadLength,
        frame.trailer,
        frame.trailerLength);
}

static mspPostProcessFnPtr'''
    pattern = r"static int mspSerialEncode\(mspPort_t \*msp, mspPacket_t \*packet, mspVersion_e mspVersion\).*?\n}\n\nint mspSerialEncodePacket\(mspPacket_t \*packet, mspVersion_e mspVersion, uint8_t \*frameBuf, int frameBufSize\).*?\n}\n\nstatic mspPostProcessFnPtr"
    content = replace_regex_once(content, pattern, replacement, path)
    write(path, content)


def patch_mavlink_internal_header() -> None:
    path = "src/main/mavlink/mavlink_internal.h"
    content = read(path)
    content = replace_once(
        content,
        """#define MAVLINK_TUNNEL_PAYLOAD_TYPE_INAV_MSP 0x8001
#define MAVLINK_TUNNEL_MSP_TIMEOUT_MS 1000
#define MAVLINK_TUNNEL_MSP_FRAMEBUF_SIZE (MSP_PORT_OUTBUF_SIZE + 16)
""",
        """#ifdef USE_MAVLINK_MSP_TUNNEL
#define MAVLINK_TUNNEL_PAYLOAD_TYPE_INAV_MSP 0x8001
#define MAVLINK_TUNNEL_MSP_TIMEOUT_MS 1000
#endif
""",
        path,
    )
    content = replace_once(
        content,
        """    mspPort_t tunnelMspPorts[MAX_MAVLINK_PORTS];
    uint8_t tunnelRemoteSystemIds[MAX_MAVLINK_PORTS];
    uint8_t tunnelRemoteComponentIds[MAX_MAVLINK_PORTS];
""",
        """#ifdef USE_MAVLINK_MSP_TUNNEL
    mspPort_t tunnelMspPorts[MAX_MAVLINK_PORTS];
    uint8_t tunnelRemoteSystemIds[MAX_MAVLINK_PORTS];
    uint8_t tunnelRemoteComponentIds[MAX_MAVLINK_PORTS];
    uint8_t tunnelReplyPayloadBuf[MSP_PORT_OUTBUF_SIZE];
#endif
""",
        path,
    )
    content = replace_once(
        content,
        """    uint8_t componentId;
    uint8_t tunnelReplyPayloadBuf[MSP_PORT_OUTBUF_SIZE];
    uint8_t tunnelFrameBuf[MAVLINK_TUNNEL_MSP_FRAMEBUF_SIZE];
    uint8_t recvPortIndex;
""",
        """    uint8_t componentId;
    uint8_t recvPortIndex;
""",
        path,
    )
    content = replace_once(
        content,
        """#define mavTunnelMspPorts (mavlinkContext.tunnelMspPorts)
#define mavTunnelRemoteSystemIds (mavlinkContext.tunnelRemoteSystemIds)
#define mavTunnelRemoteComponentIds (mavlinkContext.tunnelRemoteComponentIds)
""",
        """#ifdef USE_MAVLINK_MSP_TUNNEL
#define mavTunnelMspPorts (mavlinkContext.tunnelMspPorts)
#define mavTunnelRemoteSystemIds (mavlinkContext.tunnelRemoteSystemIds)
#define mavTunnelRemoteComponentIds (mavlinkContext.tunnelRemoteComponentIds)
#define mavTunnelReplyPayloadBuf (mavlinkContext.tunnelReplyPayloadBuf)
#endif
""",
        path,
    )
    content = replace_once(
        content,
        """#define mavComponentId (mavlinkContext.componentId)
#define mavTunnelReplyPayloadBuf (mavlinkContext.tunnelReplyPayloadBuf)
#define mavTunnelFrameBuf (mavlinkContext.tunnelFrameBuf)
#define mavRecvPortIndex (mavlinkContext.recvPortIndex)
""",
        """#define mavComponentId (mavlinkContext.componentId)
#define mavRecvPortIndex (mavlinkContext.recvPortIndex)
""",
        path,
    )
    write(path, content)


def patch_feature_guard() -> None:
    path = "src/main/target/common_post.h"
    content = read(path)
    marker = "// Touch up configuration\n\n#pragma once\n"
    replacement = """// Touch up configuration

#pragma once

// MSP-over-MAVLink is independently removable from MAVLink targets that cannot
// afford its reply buffer. Define DISABLE_MAVLINK_MSP_TUNNEL in a target to
// retain normal MAVLink without the tunnel RAM/code cost.
#if defined(USE_TELEMETRY_MAVLINK) && !defined(DISABLE_MAVLINK_MSP_TUNNEL)
#define USE_MAVLINK_MSP_TUNNEL
#endif
"""
    content = replace_once(content, marker, replacement, path)
    write(path, content)


def patch_fc_mavlink() -> None:
    path = "src/main/fc/fc_mavlink.c"
    content = read(path)
    replacement = r'''#ifdef USE_MAVLINK_MSP_TUNNEL
static void mavlinkResetTunnelState(uint8_t ingressPortIndex)
{
    resetMspPort(&mavTunnelMspPorts[ingressPortIndex], NULL);
    mavTunnelRemoteSystemIds[ingressPortIndex] = 0;
    mavTunnelRemoteComponentIds[ingressPortIndex] = 0;
}

static void mavlinkReleaseTunnelOwnerIfIdle(uint8_t ingressPortIndex)
{
    if (mavTunnelMspPorts[ingressPortIndex].c_state == MSP_IDLE) {
        mavTunnelRemoteSystemIds[ingressPortIndex] = 0;
        mavTunnelRemoteComponentIds[ingressPortIndex] = 0;
    }
}

static void mavlinkSendTunnelReply(uint8_t targetSystem, uint8_t targetComponent, const uint8_t *payload, uint8_t payloadLength)
{
    uint8_t tunnelPayload[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN] = { 0 };
    memcpy(tunnelPayload, payload, payloadLength);

    mavlink_msg_tunnel_pack(
        mavSystemId,
        mavComponentId,
        &mavSendMsg,
        targetSystem,
        targetComponent,
        MAVLINK_TUNNEL_PAYLOAD_TYPE_INAV_MSP,
        payloadLength,
        tunnelPayload);
    mavlinkSendMessage();
}

static bool mavlinkSendTunnelMspReply(uint8_t targetSystem, uint8_t targetComponent, mspPacket_t *reply, uint8_t *replyPayloadHead, mspVersion_e mspVersion)
{
    sbufSwitchToReader(&reply->buf, replyPayloadHead);

    mspEncodedFrame_t frame;
    if (!mspSerialPrepareFrame(reply, mspVersion, &frame)) {
        return false;
    }

    uint8_t chunk[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN];
    for (int offset = 0; offset < frame.totalLength; offset += MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN) {
        const int chunkLength = mspSerialFrameCopyRange(&frame, offset, chunk, sizeof(chunk));
        if (chunkLength <= 0) {
            return false;
        }
        mavlinkSendTunnelReply(targetSystem, targetComponent, chunk, chunkLength);
    }
    return true;
}

static bool mavlinkTunnelMessageTargetsLocalFc(const mavlink_tunnel_t *msg)
{
    return msg->payload_type == MAVLINK_TUNNEL_PAYLOAD_TYPE_INAV_MSP &&
        msg->target_system == mavSystemId &&
        (msg->target_component == 0 || msg->target_component == mavComponentId);
}

static void mavlinkPrepareTunnelParser(uint8_t ingressPortIndex, timeMs_t now)
{
    mspPort_t *mspPort = &mavTunnelMspPorts[ingressPortIndex];
    const bool senderChanged =
        mavTunnelRemoteSystemIds[ingressPortIndex] != mavlinkContext.recvMsg.sysid ||
        mavTunnelRemoteComponentIds[ingressPortIndex] != mavlinkContext.recvMsg.compid;
    const bool timedOut = (now - mspPort->lastActivityMs) >= MAVLINK_TUNNEL_MSP_TIMEOUT_MS;

    if (mspPort->c_state != MSP_IDLE && (timedOut || senderChanged)) {
        mavlinkResetTunnelState(ingressPortIndex);
    }

    mavTunnelRemoteSystemIds[ingressPortIndex] = mavlinkContext.recvMsg.sysid;
    mavTunnelRemoteComponentIds[ingressPortIndex] = mavlinkContext.recvMsg.compid;
    mspPort->lastActivityMs = now;
}

static bool mavlinkProcessCompletedTunnelCommand(uint8_t ingressPortIndex)
{
    mspPort_t *mspPort = &mavTunnelMspPorts[ingressPortIndex];
    const mspVersion_e mspVersion = mspPort->mspVersion;

    mspPacket_t reply = {
        .buf = { .ptr = mavTunnelReplyPayloadBuf, .end = ARRAYEND(mavTunnelReplyPayloadBuf), },
        .cmd = -1,
        .flags = 0,
        .result = 0,
    };
    uint8_t *replyPayloadHead = reply.buf.ptr;

    if (mspPort->cmdMSP == MSP_SET_PASSTHROUGH) {
        reply.cmd = MSP_SET_PASSTHROUGH;
        reply.result = MSP_RESULT_ERROR;
        mspPort->c_state = MSP_IDLE;
        mavlinkSendTunnelMspReply(
            mavlinkContext.recvMsg.sysid,
            mavlinkContext.recvMsg.compid,
            &reply,
            replyPayloadHead,
            mspVersion);
        return false;
    }

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    const uint16_t command = mspPort->cmdMSP;
    mspResult_e status = mspSerialProcessCommand(mspPort, mspFcProcessCommand, &reply, &mspPostProcessFn);

    if (mspPostProcessFn && command != MSP_REBOOT) {
        sbufInit(&reply.buf, mavTunnelReplyPayloadBuf, ARRAYEND(mavTunnelReplyPayloadBuf));
        reply.result = MSP_RESULT_ERROR;
        mspPostProcessFn = NULL;
        status = MSP_RESULT_ERROR;
    }

    if (status != MSP_RESULT_NO_REPLY) {
        mavlinkSendTunnelMspReply(
            mavlinkContext.recvMsg.sysid,
            mavlinkContext.recvMsg.compid,
            &reply,
            replyPayloadHead,
            mspVersion);
    }

    if (!mspPostProcessFn) {
        return false;
    }

    waitForSerialPortToFinishTransmitting(mavPortStates[ingressPortIndex].port);
    mspPostProcessFn(mavPortStates[ingressPortIndex].port);
    return true;
}

static bool handleIncoming_TUNNEL(uint8_t ingressPortIndex)
{
    if (mavlinkGetProtocolVersion() == 1) {
        return false;
    }

    mavlink_tunnel_t msg;
    mavlink_msg_tunnel_decode(&mavlinkContext.recvMsg, &msg);

    if (!mavlinkTunnelMessageTargetsLocalFc(&msg)) {
        return false;
    }

    if (msg.payload_length > MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN) {
        mavlinkResetTunnelState(ingressPortIndex);
        return false;
    }

    mavlinkPrepareTunnelParser(ingressPortIndex, millis());

    mspPort_t *mspPort = &mavTunnelMspPorts[ingressPortIndex];
    bool handled = false;
    for (uint8_t payloadIndex = 0; payloadIndex < msg.payload_length; payloadIndex++) {
        if (!mspSerialProcessReceivedByte(mspPort, msg.payload[payloadIndex])) {
            continue;
        }

        handled = true;
        if (mspPort->c_state != MSP_COMMAND_RECEIVED) {
            continue;
        }

        if (mavlinkProcessCompletedTunnelCommand(ingressPortIndex)) {
            mavlinkResetTunnelState(ingressPortIndex);
            break;
        }
    }

    mavlinkReleaseTunnelOwnerIfIdle(ingressPortIndex);
    return handled;
}
#endif

static bool handleIncoming_RC_CHANNELS_OVERRIDE'''
    pattern = r"static void mavlinkResetTunnelState\(uint8_t ingressPortIndex\).*?\n}\n\nstatic bool handleIncoming_RC_CHANNELS_OVERRIDE"
    content = replace_regex_once(content, pattern, replacement, path)
    content = replace_once(
        content,
        """    case MAVLINK_MSG_ID_TUNNEL:
        return handleIncoming_TUNNEL(ingressPortIndex) ? MAVLINK_FC_DISPATCH_HANDLED_ACTIVITY : MAVLINK_FC_DISPATCH_NOT_HANDLED;
""",
        """#ifdef USE_MAVLINK_MSP_TUNNEL
    case MAVLINK_MSG_ID_TUNNEL:
        return handleIncoming_TUNNEL(ingressPortIndex) ? MAVLINK_FC_DISPATCH_HANDLED_ACTIVITY : MAVLINK_FC_DISPATCH_NOT_HANDLED;
#endif
""",
        path,
    )
    write(path, content)


def patch_unit_cmake() -> None:
    path = "src/test/unit/CMakeLists.txt"
    content = read(path)
    content = replace_once(
        content,
        "set_property(SOURCE mavlink_unittest.cc PROPERTY definitions USE_TELEMETRY USE_TELEMETRY_MAVLINK)",
        "set_property(SOURCE mavlink_unittest.cc PROPERTY definitions USE_TELEMETRY USE_TELEMETRY_MAVLINK USE_MAVLINK_MSP_TUNNEL)",
        path,
    )
    write(path, content)


def patch_unit_tests() -> None:
    path = "src/test/unit/mavlink_unittest.cc"
    content = read(path)
    content = replace_once(
        content,
        "static const size_t testMspFrameBufSize = MSP_PORT_OUTBUF_SIZE + 16;\n",
        "static uint16_t testReplyPayloadLength;\n",
        path,
    )

    old_helper = r'''static std::vector<uint8_t> encodeMspV1Reply(uint8_t cmd, int16_t result, const std::vector<uint8_t> &payload = {})
{
    uint8_t payloadBuf[MSP_PORT_OUTBUF_SIZE];
    mspPacket_t reply = {
        .buf = { .ptr = payloadBuf, .end = ARRAYEND(payloadBuf), },
        .cmd = cmd,
        .flags = 0,
        .result = result,
    };
    uint8_t *payloadHead = reply.buf.ptr;
    if (!payload.empty()) {
        sbufWriteData(&reply.buf, payload.data(), (int)payload.size());
    }
    sbufSwitchToReader(&reply.buf, payloadHead);

    uint8_t frameBuf[testMspFrameBufSize];
    const int frameLength = mspSerialEncodePacket(&reply, MSP_V1, frameBuf, sizeof(frameBuf));
    return std::vector<uint8_t>(frameBuf, frameBuf + frameLength);
}
'''
    new_helper = r'''static std::vector<uint8_t> encodeMspReply(uint16_t cmd, int16_t result, mspVersion_e version, const std::vector<uint8_t> &payload = {}, uint8_t flags = 0)
{
    uint8_t payloadBuf[MSP_PORT_OUTBUF_SIZE];
    mspPacket_t reply = {
        .buf = { .ptr = payloadBuf, .end = ARRAYEND(payloadBuf), },
        .cmd = cmd,
        .flags = flags,
        .result = result,
    };
    uint8_t *payloadHead = reply.buf.ptr;
    if (!payload.empty()) {
        sbufWriteData(&reply.buf, payload.data(), (int)payload.size());
    }
    sbufSwitchToReader(&reply.buf, payloadHead);

    mspEncodedFrame_t frame;
    EXPECT_TRUE(mspSerialPrepareFrame(&reply, version, &frame));
    std::vector<uint8_t> encoded(frame.totalLength);
    EXPECT_EQ(mspSerialFrameCopyRange(&frame, 0, encoded.data(), (int)encoded.size()), (int)encoded.size());
    return encoded;
}

static std::vector<uint8_t> encodeMspV1Reply(uint8_t cmd, int16_t result, const std::vector<uint8_t> &payload = {})
{
    return encodeMspReply(cmd, result, MSP_V1, payload);
}
'''
    content = replace_once(content, old_helper, new_helper, path)

    old_push = r'''static void pushTunnelPayload(uint8_t payloadLength, const std::vector<uint8_t> &payload, uint8_t targetComponent = testTargetComponent)
{
    uint8_t tunnelPayload[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN] = { 0 };
'''
    new_push = r'''static void pushTunnelPayload(
    uint8_t payloadLength,
    const std::vector<uint8_t> &payload,
    uint8_t targetComponent = testTargetComponent,
    uint8_t sourceSystem = testTunnelSourceSystem,
    uint8_t sourceComponent = testTunnelSourceComponent)
{
    uint8_t tunnelPayload[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN] = { 0 };
'''
    content = replace_once(content, old_push, new_push, path)
    content = replace_once(
        content,
        """    mavlink_msg_tunnel_pack(
        testTunnelSourceSystem,
        testTunnelSourceComponent,
""",
        """    mavlink_msg_tunnel_pack(
        sourceSystem,
        sourceComponent,
""",
        path,
    )

    old_collect = r'''static std::vector<uint8_t> collectTunnelPayload(const std::vector<mavlink_message_t> &messages)
{
    std::vector<uint8_t> payload;
'''
    new_collect = r'''static std::vector<uint8_t> collectTunnelPayload(
    const std::vector<mavlink_message_t> &messages,
    uint8_t expectedTargetSystem = testTunnelSourceSystem,
    uint8_t expectedTargetComponent = testTunnelSourceComponent)
{
    std::vector<uint8_t> payload;
'''
    content = replace_once(content, old_collect, new_collect, path)
    content = replace_once(
        content,
        """        EXPECT_EQ(tunnel.target_system, testTunnelSourceSystem);
        EXPECT_EQ(tunnel.target_component, testTunnelSourceComponent);
""",
        """        EXPECT_EQ(tunnel.target_system, expectedTargetSystem);
        EXPECT_EQ(tunnel.target_component, expectedTargetComponent);
""",
        path,
    )
    content = replace_once(
        content,
        "    mspCommandCallCount = 0;\n",
        "    mspCommandCallCount = 0;\n    testReplyPayloadLength = 300;\n",
        path,
    )

    test_block = r'''
TEST(MavlinkTelemetryTest, TunnelProcessesEveryCompleteFrameInOnePayload)
{
    initMavlinkTestState();

    const std::vector<uint8_t> request = makeMspV1Request(testSimpleMspCommand);
    std::vector<uint8_t> combined = request;
    combined.insert(combined.end(), request.begin(), request.end());

    pushTunnelPayload((uint8_t)combined.size(), combined);
    handleMAVLinkTelemetry(1000);

    EXPECT_EQ(mspCommandCallCount, 2);
    std::vector<uint8_t> expected = encodeMspV1Reply(testSimpleMspCommand, MSP_RESULT_ACK);
    expected.insert(expected.end(), expected.begin(), expected.end());
    EXPECT_EQ(collectTunnelPayload(parseTxMessages()), expected);
}

TEST(MavlinkTelemetryTest, TunnelPreservesPartialFrameAfterCompletedFrame)
{
    initMavlinkTestState();

    const std::vector<uint8_t> request = makeMspV1Request(testSimpleMspCommand);
    std::vector<uint8_t> firstPayload = request;
    firstPayload.insert(firstPayload.end(), request.begin(), request.begin() + 3);

    pushTunnelPayload((uint8_t)firstPayload.size(), firstPayload);
    handleMAVLinkTelemetry(1000);
    EXPECT_EQ(mspCommandCallCount, 1);

    resetSerialBuffers();
    const std::vector<uint8_t> remainder(request.begin() + 3, request.end());
    pushTunnelPayload((uint8_t)remainder.size(), remainder);
    handleMAVLinkTelemetry(1000);

    EXPECT_EQ(mspCommandCallCount, 2);
    EXPECT_EQ(collectTunnelPayload(parseTxMessages()), encodeMspV1Reply(testSimpleMspCommand, MSP_RESULT_ACK));
}

TEST(MavlinkTelemetryTest, TunnelTimeoutResetsAtExactBoundary)
{
    initMavlinkTestState();
    fakeMillis = 1000;

    pushTunnelPayload(3, {'$', 'M', '<'});
    handleMAVLinkTelemetry(1000);
    EXPECT_EQ(mspCommandCallCount, 0);

    resetSerialBuffers();
    fakeMillis = 2000;
    const std::vector<uint8_t> request = makeMspV1Request(testSimpleMspCommand);
    pushTunnelPayload((uint8_t)request.size(), request);
    handleMAVLinkTelemetry(1000);

    EXPECT_EQ(mspCommandCallCount, 1);
    EXPECT_EQ(collectTunnelPayload(parseTxMessages()), encodeMspV1Reply(testSimpleMspCommand, MSP_RESULT_ACK));
}

TEST(MavlinkTelemetryTest, TunnelDifferentSenderDisplacesPartialTransaction)
{
    initMavlinkTestState();

    pushTunnelPayload(3, {'$', 'M', '<'});
    handleMAVLinkTelemetry(1000);

    resetSerialBuffers();
    const uint8_t replacementSystem = testTunnelSourceSystem + 1;
    const std::vector<uint8_t> request = makeMspV1Request(testSimpleMspCommand);
    pushTunnelPayload(
        (uint8_t)request.size(),
        request,
        testTargetComponent,
        replacementSystem,
        testTunnelSourceComponent);
    handleMAVLinkTelemetry(1000);

    EXPECT_EQ(mspCommandCallCount, 1);
    EXPECT_EQ(
        collectTunnelPayload(parseTxMessages(), replacementSystem, testTunnelSourceComponent),
        encodeMspV1Reply(testSimpleMspCommand, MSP_RESULT_ACK));
}

TEST(MavlinkTelemetryTest, TunnelReplyFragmentationIsExactAt127128And129Bytes)
{
    const uint16_t payloadLengths[] = { 121, 122, 123 };
    const size_t expectedMessageCounts[] = { 1, 1, 2 };

    for (size_t caseIndex = 0; caseIndex < ARRAYLEN(payloadLengths); caseIndex++) {
        initMavlinkTestState();
        testReplyPayloadLength = payloadLengths[caseIndex];

        const std::vector<uint8_t> request = makeMspV1Request(testLargeReplyMspCommand);
        pushTunnelPayload((uint8_t)request.size(), request);
        handleMAVLinkTelemetry(1000);

        std::vector<uint8_t> expectedPayload(testReplyPayloadLength);
        for (size_t i = 0; i < expectedPayload.size(); i++) {
            expectedPayload[i] = (uint8_t)i;
        }

        const std::vector<mavlink_message_t> messages = parseTxMessages();
        EXPECT_EQ(messages.size(), expectedMessageCounts[caseIndex]);
        EXPECT_EQ(
            collectTunnelPayload(messages),
            encodeMspV1Reply(testLargeReplyMspCommand, MSP_RESULT_ACK, expectedPayload));
    }
}

TEST(MavlinkTelemetryTest, PreparedFrameMatchesExactV1V2AndV2OverV1Bytes)
{
    const std::vector<uint8_t> payload = { 0x10, 0x20 };

    std::vector<uint8_t> v1 = encodeMspReply(0x5a, MSP_RESULT_ACK, MSP_V1, payload);
    std::vector<uint8_t> expectedV1 = { '$', 'M', '>', 2, 0x5a, 0x10, 0x20 };
    expectedV1.push_back(2 ^ 0x5a ^ 0x10 ^ 0x20);
    EXPECT_EQ(v1, expectedV1);

    const uint16_t v2Command = 0x1234;
    const uint8_t flags = 0x07;
    std::vector<uint8_t> native = encodeMspReply(v2Command, MSP_RESULT_ACK, MSP_V2_NATIVE, payload, flags);
    std::vector<uint8_t> expectedNative = {
        '$', 'X', '>', flags,
        (uint8_t)v2Command, (uint8_t)(v2Command >> 8),
        2, 0,
        0x10, 0x20,
    };
    uint8_t nativeCrc = crc8_dvb_s2_update(0, expectedNative.data() + 3, 5);
    nativeCrc = crc8_dvb_s2_update(nativeCrc, payload.data(), payload.size());
    expectedNative.push_back(nativeCrc);
    EXPECT_EQ(native, expectedNative);

    std::vector<uint8_t> overV1 = encodeMspReply(v2Command, MSP_RESULT_ACK, MSP_V2_OVER_V1, payload, flags);
    std::vector<uint8_t> expectedOverV1 = {
        '$', 'M', '>', 8, MSP_V2_FRAME_ID,
        flags, (uint8_t)v2Command, (uint8_t)(v2Command >> 8), 2, 0,
        0x10, 0x20,
    };
    uint8_t overV1Crc = crc8_dvb_s2_update(0, expectedOverV1.data() + 5, 5);
    overV1Crc = crc8_dvb_s2_update(overV1Crc, payload.data(), payload.size());
    expectedOverV1.push_back(overV1Crc);
    uint8_t v1Checksum = 0;
    for (size_t i = 3; i < expectedOverV1.size(); i++) {
        v1Checksum ^= expectedOverV1[i];
    }
    expectedOverV1.push_back(v1Checksum);
    EXPECT_EQ(overV1, expectedOverV1);
}
'''
    marker = "TEST(MavlinkTelemetryTest, MlrsRadioLinkStatsUpdateRxStatisticsOnMavlinkSerialRxPort)"
    content = replace_once(content, marker, test_block + "\n" + marker, path)

    content = replace_once(
        content,
        """    case testLargeReplyMspCommand:
        for (uint16_t i = 0; i < 300; i++) {
""",
        """    case testLargeReplyMspCommand:
        for (uint16_t i = 0; i < testReplyPayloadLength; i++) {
""",
        path,
    )
    write(path, content)


def finalize() -> None:
    handoff = ROOT / "src/test/mavlink/tunnel/CODEX_REVIEW_FIX_HANDOFF.md"
    if handoff.exists():
        handoff.unlink()
    Path(__file__).unlink()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--finalize", action="store_true", help="remove the staging files after applying the source changes")
    args = parser.parse_args()

    patch_msp_serial_header()
    patch_msp_serial_source()
    patch_mavlink_internal_header()
    patch_feature_guard()
    patch_fc_mavlink()
    patch_unit_cmake()
    patch_unit_tests()

    if args.finalize:
        finalize()

    print("Applied MSP-over-MAVLink tunnel review fixes.")


if __name__ == "__main__":
    main()
