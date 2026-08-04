# Codex task: MSP-over-MAVLink tunnel review fixes

## Repository and branch

- Repository: `xznhj8129/inav`
- Work branch: `mav/07-tunnel-review-fixes`
- Exact starting commit: `2a16c78610f3670d994a74f099aee858d7d1f4ec`
- Original branch `mav/07-tunnel` is read-only for this task. Do not push to it.
- PR context: upstream `iNavFlight/inav#11718`, part 7/7 of the stacked MAVLink multiport series.

Before editing:

```bash
git fetch origin
git switch mav/07-tunnel-review-fixes
git reset --hard origin/mav/07-tunnel-review-fixes
git status --short --branch
```

Do not rebase this branch onto another base. Do not pull unrelated upstream changes into it.

## Goal

Address the maintainer review of the MSP-over-MAVLink TUNNEL implementation while preserving all existing tunnel behavior.

The implementation must end with:

1. One canonical MSP framing implementation shared by serial MSP and MAVLink tunnel replies.
2. No second full-size encoded-frame buffer.
3. TUNNEL replies streamed/chunked directly from logical MSP frame segments: header, existing reply payload, trailer/checksum.
4. Correct handling of every complete MSP frame present in a TUNNEL payload, including a complete frame followed by a partial next frame.
5. A dedicated build guard for the tunnel so MAVLink builds that do not enable the tunnel do not allocate its large reply buffer or parser state.
6. Focused helpers in `fc_mavlink.c` so validation, stale transaction handling, and completed-command handling are not one monolithic function.
7. Tests that prove framing equivalence, fragmentation boundaries, sender ownership, timeout boundaries, multiple-frame behavior, save/reboot/reconnect, and target build memory impact.

## Existing behavior that must remain true

- MAVLink 2 only.
- TUNNEL private payload type `0x8001`.
- `target_system` must match local `mavlink_sysid`.
- `target_component` may be `0` or `MAV_COMP_ID_AUTOPILOT1`.
- Reply returns on the ingress MAVLink port only.
- MSPv1 request gets MSPv1 reply.
- MSPv2-over-v1 request gets MSPv2-over-v1 reply.
- Native MSPv2 request gets native MSPv2 reply.
- Stale partial transactions reset after `MAVLINK_TUNNEL_MSP_TIMEOUT_MS`.
- A different sender displaces a partial transaction according to the existing intended ownership semantics.
- `MSP_SET_PASSTHROUGH` and unsupported passthrough paths are rejected before dispatch.
- `MSP_REBOOT` reply is transmitted before post-processing/reboot.
- Other post-process callbacks remain rejected over the tunnel.
- Malformed TUNNEL payload lengths do not poison later valid requests.
- No large response buffer may be placed on the telemetry task stack.

## Required implementation direction

### 1. Replace duplicate encoders with a prepared-frame representation

`mspSerialEncodePacket()` currently duplicates most of `mspSerialEncode()`. Remove that duplication.

Create one framing helper in `msp_serial.[ch]` that prepares a logical frame containing small owned header/trailer arrays and a borrowed pointer to the existing reply payload. A reasonable shape is:

```c
typedef struct {
    uint8_t header[16];
    uint8_t headerLength;
    const uint8_t *payload;
    uint16_t payloadLength;
    uint8_t trailer[2];
    uint8_t trailerLength;
    uint16_t totalLength;
} mspEncodedFrame_t;
```

Names and exact integer types may change if a better fit exists, but preserve these semantics.

Provide a helper similar to:

```c
bool mspSerialPrepareFrame(
    const mspPacket_t *packet,
    mspVersion_e version,
    mspEncodedFrame_t *frame);
```

The helper must be the only implementation of:

- MSPv1 framing
- MSPv1 jumbo framing
- MSPv2-over-v1 framing
- native MSPv2 framing
- XOR checksum
- CRC8-DVB-S2 checksum

The normal serial path writes the three prepared segments to the serial port. The tunnel path reads ranges from the same three segments.

Delete `mspSerialEncodePacket()` once no caller needs a contiguous frame.

### 2. Remove the redundant full encoded-frame buffer

Delete:

- `MAVLINK_TUNNEL_MSP_FRAMEBUF_SIZE`
- `mavlinkContext_t::tunnelFrameBuf`
- `mavTunnelFrameBuf`
- any equivalent replacement full-frame buffer

The existing `tunnelReplyPayloadBuf[MSP_PORT_OUTBUF_SIZE]` may remain because `mspFcProcessCommand()` writes its reply payload contiguously. Do not add another payload-sized or frame-sized scratch allocation.

Implement a bounded range-copy helper that copies an arbitrary logical frame range into one MAVLink TUNNEL payload without unsigned subtraction underflow. For example:

```c
size_t mspSerialCopyFrameRange(
    const mspEncodedFrame_t *frame,
    size_t offset,
    uint8_t *destination,
    size_t destinationSize);
```

The tunnel sender should iterate over `frame.totalLength`, fill at most `MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN` bytes per packet, and transmit each chunk.

Do not paste conceptual pseudocode that performs expressions such as `offset - headerLength` before proving the offset is in that segment.

### 3. Separate parser reset from transaction ownership reset

The current full tunnel reset clears both parser state and remote sender ownership. That is not always correct after completing one frame when more bytes remain in the same TUNNEL payload.

Introduce explicit operations with clear semantics, for example:

- reset only the MSP frame parser so it can parse the next frame while retaining the current sender transaction
- abandon/reset the full tunnel transaction, including sender ownership

After processing a completed command, continue consuming remaining bytes in the same TUNNEL payload. Do not unconditionally `break` after the first frame.

Required case:

```text
TUNNEL packet 1: [complete request A][first part of request B]
TUNNEL packet 2: [remainder of request B]
```

Both requests must execute exactly once, both replies must be correct, and request B must retain ownership across the two TUNNEL packets.

A TUNNEL payload containing two complete requests must produce two replies in order.

### 4. Timeout semantics

Use an explicit and tested boundary. Prefer expiry when elapsed time is greater than or equal to `MAVLINK_TUNNEL_MSP_TIMEOUT_MS`, unless an established INAV convention requires otherwise.

Add tests at timeout minus one millisecond, exactly timeout, and timeout plus one millisecond.

### 5. Dedicated feature guard

Add a dedicated compile-time feature such as `USE_MAVLINK_MSP_TUNNEL` following existing INAV feature-selection conventions.

Requirements:

- Tunnel parser state, remote sender arrays, and `MSP_PORT_OUTBUF_SIZE` reply buffer are absent when the guard is disabled.
- TUNNEL handling compiles out cleanly when disabled.
- SITL and the MAVLink unit test target enable the guard.
- Enable it on the intended production target set using the narrowest existing project convention. Do not silently make every `USE_TELEMETRY_MAVLINK` build pay the RAM cost merely by defining the guard automatically in `fc_mavlink.c`.
- Document exactly where and why the feature is enabled.

If project feature conventions make the production opt-in ambiguous, choose the smallest non-invasive implementation, explain it in the final report, and do not perform broad unrelated target-file edits.

### 6. Refactor `handleIncoming_TUNNEL()` into focused helpers

Keep the top-level handler readable. Extract cohesive helpers for at least:

- packet/protocol/target validation and decode
- stale or mismatched sender transaction reset
- completed MSP command processing and reply/post-process handling

Do not create excessive one-line wrappers. The point is to make state transitions reviewable.

## Required tests

### C/C++ unit tests

Keep all existing tests passing and add focused tests for:

1. Shared frame preparation produces exact expected bytes for:
   - MSPv1 ACK and error
   - MSPv1 jumbo payload
   - MSPv2-over-v1
   - native MSPv2
   - payload lengths `0`, `1`, `127`, `128`, `129`, `254`, `255`, `256`, and maximum supported reply length where practical
2. Frame range copying across every segment transition:
   - within header
   - header to payload
   - within payload
   - payload to trailer
   - within the two-byte trailer for v2-over-v1
   - exact total lengths `127`, `128`, `129`, `255`, `256`, `257` where constructible
3. A TUNNEL payload with two complete MSP requests produces two ordered replies.
4. A complete request followed by a partial second request, completed by the next TUNNEL packet, produces both replies exactly once.
5. Partial request ownership survives completion of a preceding request from the same sender.
6. Sender change during a partial frame cannot splice frames or receive another sender's reply.
7. Timeout at `999`, `1000`, and `1001` milliseconds, adjusted if the timeout constant changes.
8. Malformed length, wrong payload type, wrong target system/component, and corrupt MSP frame recovery.
9. Existing passthrough rejection, reboot post-processing, ingress-port reply, and large-reply fragmentation tests continue to pass.

Prefer testing the public prepared-frame/range-copy seam directly instead of reproducing the framing algorithm inside the test helper. It is acceptable to keep a small independent expected-byte builder for differential assertions.

### SITL smoke test

Use and extend:

```text
src/test/mavlink/tunnel/mavlink_msp_tunnel_smoketest.py
```

The smoke test must still verify:

- target discovery by heartbeat
- fragmented MSP request
- API/variant/version/build-info reads
- EEPROM write
- reboot acknowledgement before reboot
- heartbeat recovery after reboot
- post-reboot MSP request

Add an optional direct-MSP endpoint mode if practical, e.g. `--direct-msp-endpoint tcp:127.0.0.1:5760`, that compares stable read-only MSP replies over direct MSP and MAVLink tunnel. Do not compare volatile telemetry fields byte-for-byte.

Also add a smoke-test operation that sends two complete read-only MSP requests in one TUNNEL payload and verifies two replies.

### Build and size checks

At minimum run:

```bash
cmake -S . -B build-tests -DTOOLCHAIN=none
cmake --build build-tests --target run-mavlink_unittest -j"$(nproc)"
```

Build SITL with warnings as errors using the repository's current CMake/Docker convention. The repository documents Docker SITL build via:

```bash
docker build --build-arg USER_ID="$(id -u)" --build-arg GROUP_ID="$(id -g)" -t inav-build .
docker run --rm --entrypoint /src/cmake/docker_build_sitl.sh -v "$PWD:/src" inav-build
```

Run SITL and the smoke test. Adjust only for the actual generated SITL binary path and local EEPROM setup; record every exact command used.

Build representative targets with the tunnel enabled and disabled where the feature system permits. Include at least one F405 target, one F7/H7 target, and SITL. Record `.text`, `.data`, and `.bss`, plus the before/after delta from removing `tunnelFrameBuf`.

Use the repository's `build.sh <TARGET>` or equivalent CMake target. Do not guess success from compilation alone; inspect the map or `size` output and confirm the removed full-frame symbol is absent.

## Validation discipline

- Run formatting/lint checks used by this repository for changed C/C++ and Python files.
- Build with warnings as errors.
- Do not weaken tests, compiler warnings, feature checks, or buffer bounds to get green results.
- Do not alter unrelated MAVLink mission, routing, guided-mode, documentation, generated MAVLink, or Configurator behavior.
- Do not update generated documentation unless a source setting actually changes and regeneration is required.
- Avoid large mechanical formatting changes.

## Commit and push requirements

Make implementation commits that are easy to audit. Suggested split:

1. `refactor: share MSP frame preparation`
2. `fix: stream MSP tunnel replies and preserve parser ownership`
3. `test: cover MSP tunnel framing and multi-frame input`
4. Optional dedicated feature-guard commit if it is substantial

Before pushing:

```bash
git diff --check
git status --short
git log --oneline --decorate -8
```

Remove this handoff file in the final implementation commit or in a final cleanup commit. It must not be part of the eventual upstream PR patch.

Push only:

```bash
git push origin mav/07-tunnel-review-fixes
```

Do not force-push unless the branch has only your own work and a re-run explicitly requires it. Do not open or modify the upstream PR. Do not push or merge into `mav/07-tunnel`.

## Final report required from Codex

Return:

1. Commit SHAs and one-line purposes.
2. Exact files changed.
3. Exact unit-test, SITL, smoke-test, lint, and target-build commands.
4. Pass/fail counts and relevant output excerpts.
5. `.text/.data/.bss` comparison and confirmation that the full-frame buffer symbol is gone.
6. Any behavior or feature-enable decision that remains debatable.
7. Confirmation that `mav/07-tunnel` was untouched and only `mav/07-tunnel-review-fixes` was pushed.
