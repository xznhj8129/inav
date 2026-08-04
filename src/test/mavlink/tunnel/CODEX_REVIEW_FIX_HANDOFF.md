# Local execution handoff: MSP-over-MAVLink tunnel review fixes

The implementation has already been authored in two deterministic transformation scripts on this branch. Codex must not redesign, rewrite, or substitute its own implementation.

## Repository and branch

- Repository: `xznhj8129/inav`
- Branch: `mav/07-tunnel-review-fixes`
- Do not touch `mav/07-tunnel`.
- Do not rebase or merge unrelated branches.

## Apply the authored implementation

```bash
git fetch origin
git switch mav/07-tunnel-review-fixes
git reset --hard origin/mav/07-tunnel-review-fixes

python3 src/test/mavlink/tunnel/apply_review_fixes.py
python3 src/test/mavlink/tunnel/finish_review_fixes.py

git diff --check
git status --short
```

The second script removes both transformation scripts and this handoff after applying the source changes.

If either script reports a missing or duplicate match, stop and report the exact error. Do not improvise a replacement implementation.

## Required local validation

Run the repository's normal dependency/bootstrap procedure if the build directory is absent.

### Unit test build and MAVLink suite

```bash
cmake -S . -B build-tests -DTOOLCHAIN=none -DWARNINGS_AS_ERRORS=ON
cmake --build build-tests --target mavlink_unittest -j"$(nproc)"
./build-tests/src/test/unit/mavlink_unittest
```

Then run the complete unit suite:

```bash
cmake --build build-tests --target check -j"$(nproc)"
```

### SITL build

Use the normal repository SITL build method. With Docker:

```bash
docker build --build-arg USER_ID="$(id -u)" --build-arg GROUP_ID="$(id -g)" -t inav-build .
docker run --rm --entrypoint /src/cmake/docker_build_sitl.sh -v "$PWD:/src" inav-build
```

Or use the existing native SITL build environment if already configured.

Run SITL with UART2 configured for MAVLink and execute:

```bash
python3 src/test/mavlink/tunnel/mavlink_msp_tunnel_smoketest.py \
  --mavlink-endpoint tcp:127.0.0.1:5761
```

The smoke test must pass API discovery, FC identification, EEPROM write, reboot acknowledgement, heartbeat rediscovery, and post-reboot MSP.

### Target builds and memory evidence

Build at least:

- one representative F405 target used by this work
- one F7 or H7 target
- SITL

Record `.text`, `.data`, and `.bss` for the original head `2a16c78610f3670d994a74f099aee858d7d1f4ec` and the fixed branch. Confirm that the removed full-frame buffer no longer appears in the map or symbol listing.

Also compile one MAVLink target with `DISABLE_MAVLINK_MSP_TUNNEL` defined and confirm that tunnel parser state and `tunnelReplyPayloadBuf` are absent from the resulting symbols/BSS.

## Inspect before committing

Confirm these facts in the final diff:

- `mspSerialEncodePacket()` is gone.
- `mspSerialPrepareFrame()` is the sole MSP response framing implementation.
- serial MSP and MAVLink tunnel both consume the same prepared frame.
- `mavTunnelFrameBuf` and `MAVLINK_TUNNEL_MSP_FRAMEBUF_SIZE` are gone.
- TUNNEL chunks are copied directly from header, payload, and trailer segments.
- multiple complete MSP frames in one TUNNEL payload are processed.
- a complete frame followed by a partial next frame preserves the partial parser state and sender ownership.
- timeout comparison is exact at `>= MAVLINK_TUNNEL_MSP_TIMEOUT_MS`.
- tunnel allocations and dispatch are guarded by `USE_MAVLINK_MSP_TUNNEL`.
- port lifecycle and parser abandonment use the same tunnel reset helper.

Do not change behavior outside this scope unless required to fix a compile or test failure caused directly by the authored patch. Report such a change before making it.

## Commit and push

After all required checks pass:

```bash
git add src/main src/test
git commit -m "Refactor MSP tunnel framing and transaction handling"
git push origin mav/07-tunnel-review-fixes
```

Report:

- final commit SHA
- exact commands run
- unit-test totals
- SITL smoke-test output summary
- target size deltas
- disabled-tunnel BSS/symbol evidence
- any failure or unresolved concern
