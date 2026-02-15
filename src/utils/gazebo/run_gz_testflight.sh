#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SITL_GZ_ROOT="${REPO_ROOT}/src/utils/gazebo"
STATE_DIR="${SITL_GZ_ROOT}/state"
LOG_DIR="${STATE_DIR}/logs"
SITL_BIN="${STATE_DIR}/build/bin/SITL.elf"
RUN_LOG="${LOG_DIR}/gz_run.log"
APPLY_LOG="${LOG_DIR}/gz_apply.log"
TCP_ENDPOINT="127.0.0.1:5761"
MODEL="x500_mono_camera"
WORLD="forest"
CHANMAP="M01-01,M04-02,M03-03,M02-04"
APPLY_DIFF=0
BUILD=0
RESET_EEPROM=0
EXTRA_SITL_ARGS=()
SITL_START_DELAY_S=14

while [[ $# -gt 0 ]]; do
    case "$1" in
        --apply-diff)
            APPLY_DIFF=1
            shift
            ;;
        --build)
            BUILD=1
            shift
            ;;
        --reset-eeprom)
            RESET_EEPROM=1
            shift
            ;;
        --model)
            MODEL="$2"
            shift 2
            ;;
        --world)
            WORLD="$2"
            shift 2
            ;;
        --chanmap)
            CHANMAP="$2"
            shift 2
            ;;
        --sitl-bin)
            SITL_BIN="$2"
            shift 2
            ;;
        --)
            shift
            EXTRA_SITL_ARGS=("$@")
            break
            ;;
        *)
            EXTRA_SITL_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ "${#EXTRA_SITL_ARGS[@]}" -gt 0 ]]; then
    for idx in "${!EXTRA_SITL_ARGS[@]}"; do
        echo "extra_sitl_arg_${idx}=${EXTRA_SITL_ARGS[idx]}"
    done
fi

cleanup() {
    if [[ -n "${RUN_PID:-}" ]] && kill -0 "${RUN_PID}" 2>/dev/null; then
        kill "${RUN_PID}" 2>/dev/null || true
        wait "${RUN_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

pkill -f "gz sim" 2>/dev/null || true
pkill -f "bin/SITL.elf|inav_.*_SITL|cmake/run_sitl_gazebo.sh" 2>/dev/null || true

if [[ "${BUILD}" -eq 1 ]]; then
    (unset LD_LIBRARY_PATH; "${REPO_ROOT}/cmake/docker_build_sitl.sh")
fi

mkdir -p "${LOG_DIR}"
rm -f "${RUN_LOG}" "${APPLY_LOG}"

launch_sitl() {
    local reset_arg=()
    if [[ "${RESET_EEPROM}" -eq 1 ]]; then
        reset_arg+=(--reset-eeprom)
    fi

    (cd "${REPO_ROOT}" && ./cmake/run_sitl_gazebo.sh \
        --sitl-bin "${SITL_BIN}" \
        --world "${WORLD}" \
        --model "${MODEL}" \
        --chanmap "${CHANMAP}" \
        "${reset_arg[@]}" \
        -- --useimu "${EXTRA_SITL_ARGS[@]}" >"${RUN_LOG}" 2>&1) &
    RUN_PID=$!
}

launch_sitl
sleep "${SITL_START_DELAY_S}"
if ! kill -0 "${RUN_PID}" 2>/dev/null; then
    tail -n 200 "${RUN_LOG}" || true
    exit 1
fi

if [[ "${APPLY_DIFF}" -eq 1 ]]; then
    (cd "${REPO_ROOT}" && python src/utils/gazebo/apply_sitl_diff.py --tcp "${TCP_ENDPOINT}" --skip-save >"${APPLY_LOG}" 2>&1)
    sleep 2
fi

echo "sitl_bin=${SITL_BIN}"
echo "model=${MODEL}"
echo "world=${WORLD}"
echo "chanmap=${CHANMAP}"
echo "msp_tcp=${TCP_ENDPOINT}"
echo "configurator_tcp=127.0.0.1:5760"
echo "run_log=${RUN_LOG}"
echo "apply_log=${APPLY_LOG}"
echo "sitl_pid=${RUN_PID}"

wait "${RUN_PID}"
