#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SITL_GZ_ROOT="${REPO_ROOT}/src/utils/gazebo"
STATE_DIR="${SITL_GZ_ROOT}/state"
LOG_DIR="${STATE_DIR}/logs"
SITL_BIN="${STATE_DIR}/build/bin/SITL.elf"
RUN_LOG="${LOG_DIR}/gz_run.log"
TAKEOFF_LOG="${LOG_DIR}/takeoff_run.log"
APPLY_LOG="${LOG_DIR}/gz_apply.log"
TCP_ENDPOINT="127.0.0.1:5761"
APPLY_DIFF=0
CHANMAP="M01-01,M04-02,M03-03,M02-04"
USE_IMU=1
SITL_START_DELAY_S=24
MODEL_NAME="x500_mono_camera"
WORLD_NAME="forest"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --apply-diff)
            APPLY_DIFF=1
            shift
            ;;
        --chanmap)
            CHANMAP="$2"
            shift 2
            ;;
        --model)
            MODEL_NAME="$2"
            shift 2
            ;;
        --world)
            WORLD_NAME="$2"
            shift 2
            ;;
        --no-useimu)
            USE_IMU=0
            shift
            ;;
        *)
            echo "Unknown arg: $1" >&2
            exit 1
            ;;
    esac
done

cleanup() {
    if [[ -n "${RUN_PID:-}" ]] && kill -0 "${RUN_PID}" 2>/dev/null; then
        kill "${RUN_PID}" 2>/dev/null || true
        wait "${RUN_PID}" 2>/dev/null || true
    fi
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "bin/SITL.elf|inav_.*_SITL" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

mkdir -p "${LOG_DIR}"
rm -f "${RUN_LOG}" "${TAKEOFF_LOG}" "${APPLY_LOG}"
SIM_ARGS=()
if [[ "${USE_IMU}" -eq 1 ]]; then
    SIM_ARGS+=(--useimu)
fi

(cd "${REPO_ROOT}" && ./cmake/run_sitl_gazebo.sh --sitl-bin "${SITL_BIN}" --world "${WORLD_NAME}" --model "${MODEL_NAME}" --chanmap "${CHANMAP}" -- "${SIM_ARGS[@]}" >"${RUN_LOG}" 2>&1) &
RUN_PID=$!
echo "sitl_pid=${RUN_PID} run_log=${RUN_LOG}"
sleep "${SITL_START_DELAY_S}"
if ! kill -0 "${RUN_PID}" 2>/dev/null; then
    echo "sitl_started=0 sitl_exited_early=1"
    tail -n 160 "${RUN_LOG}" || true
    exit 1
fi
echo "sitl_started=1 tcp=${TCP_ENDPOINT}"

if [[ "${APPLY_DIFF}" -eq 1 ]]; then
    (cd "${REPO_ROOT}" && python src/utils/gazebo/apply_sitl_diff.py --tcp "${TCP_ENDPOINT}" --skip-save >"${APPLY_LOG}" 2>&1)
    echo "diff_applied=1 apply_log=${APPLY_LOG}"
    sleep 2
fi

(cd "${REPO_ROOT}" && python src/utils/gazebo/mspapi2_takeoff_example.py --tcp "localhost:5761" | tee "${TAKEOFF_LOG}")
TAKEOFF_RC=$?
echo "takeoff_rc=${TAKEOFF_RC} takeoff_log=${TAKEOFF_LOG}"
exit "${TAKEOFF_RC}"
