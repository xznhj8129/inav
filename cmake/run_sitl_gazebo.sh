#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RESOURCE_ROOT="${REPO_ROOT}/src/main/target/SITL/gazebo"
SITL_GZ_ROOT="${REPO_ROOT}/src/utils/gazebo"
STATE_ROOT="${SITL_GZ_ROOT}/state"
GUI_CONFIG_FILE="${SITL_GZ_ROOT}/gui.config"
STATE_BUILD_DIR="${STATE_ROOT}/build"
STATE_LOG_DIR="${STATE_ROOT}/logs"
STATE_EEPROM_PATH="${STATE_ROOT}/eeprom.bin"
LEGACY_STATE_EEPROM_PATH="${REPO_ROOT}/utils/gazebo/sitl_state/eeprom.bin"
LEGACY_EEPROM_PATH="${REPO_ROOT}/build_SITL/eeprom.bin"

WORLD_NAME="forest"
MODEL_NAME="x500_mono_camera"
MODEL_RESOURCE="${MODEL_NAME}"
CHANMAP="M01-01,M04-02,M03-03,M02-04"
TCP_BASE_PORT="5760"
EEPROM_PATH=""
RESET_EEPROM=0
DO_BUILD=0
SITL_BIN=""
EXTRA_SITL_ARGS=()

usage() {
    cat <<USAGE
Usage: ${0##*/} [options] [-- <extra SITL args>]

Options:
  --build                 Run cmake/docker_build_sitl.sh before launch.
  --world <name>          Gazebo world name (default: ${WORLD_NAME}).
  --model <name>          Gazebo model instance name (default: ${MODEL_NAME}).
  --chanmap <map>         SITL channel map (default: ${CHANMAP}).
  --tcpbaseport <port>    SITL base TCP port (default: ${TCP_BASE_PORT}).
  --eeprom <path>         EEPROM file path (default: src/utils/gazebo/state/eeprom.bin).
  --reset-eeprom          Reset EEPROM before launch (default keeps persisted config).
  --keep-eeprom           Keep EEPROM content between runs (explicit no-op).
  --sitl-bin <path>       Explicit SITL binary path.
  --help                  Show this help.

Example:
  ${0##*/} --build
  ${0##*/} --chanmap M01-01,M04-02,M03-03,M02-04 -- --tcpbaseport=5860
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            DO_BUILD=1
            shift
            ;;
        --world)
            WORLD_NAME="$2"
            shift 2
            ;;
        --model)
            MODEL_NAME="$2"
            MODEL_RESOURCE="$2"
            shift 2
            ;;
        --chanmap)
            CHANMAP="$2"
            shift 2
            ;;
        --tcpbaseport)
            TCP_BASE_PORT="$2"
            shift 2
            ;;
        --eeprom)
            EEPROM_PATH="$2"
            shift 2
            ;;
        --keep-eeprom)
            RESET_EEPROM=0
            shift
            ;;
        --reset-eeprom)
            RESET_EEPROM=1
            shift
            ;;
        --sitl-bin)
            SITL_BIN="$2"
            shift 2
            ;;
        --help)
            usage
            exit 0
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

if [[ ${DO_BUILD} -eq 1 ]]; then
    (unset LD_LIBRARY_PATH; "${REPO_ROOT}/cmake/docker_build_sitl.sh")
fi

mapfile -t gz_pids < <(pgrep -f "gz sim" || true)
if [[ ${#gz_pids[@]} -gt 0 ]]; then
    kill "${gz_pids[@]}" 2>/dev/null || true
    sleep 0.5
    kill -9 "${gz_pids[@]}" 2>/dev/null || true
fi

if [[ -z "${SITL_BIN}" ]]; then
    mapfile -t sitl_bins < <(ls -1 "${STATE_BUILD_DIR}"/inav_*_SITL 2>/dev/null | sort -V)
    if [[ ${#sitl_bins[@]} -eq 0 ]]; then
        mapfile -t sitl_bins < <(ls -1 "${REPO_ROOT}"/build_SITL/inav_*_SITL 2>/dev/null | sort -V)
    fi
    if [[ ${#sitl_bins[@]} -eq 0 ]]; then
        echo "No SITL binary found in ${STATE_BUILD_DIR}." >&2
        echo "Build first with: unset LD_LIBRARY_PATH; cmake/docker_build_sitl.sh" >&2
        exit 1
    fi
    SITL_BIN="${sitl_bins[-1]}"
fi

if [[ -z "${EEPROM_PATH}" ]]; then
    EEPROM_PATH="${STATE_EEPROM_PATH}"
fi

mkdir -p "$(dirname "${EEPROM_PATH}")"
mkdir -p "${STATE_LOG_DIR}"

if [[ ! -f "${EEPROM_PATH}" && -f "${LEGACY_STATE_EEPROM_PATH}" ]]; then
    cp -a "${LEGACY_STATE_EEPROM_PATH}" "${EEPROM_PATH}"
fi

if [[ ! -f "${EEPROM_PATH}" && -f "${LEGACY_EEPROM_PATH}" ]]; then
    cp -a "${LEGACY_EEPROM_PATH}" "${EEPROM_PATH}"
fi

if [[ ${RESET_EEPROM} -eq 1 ]]; then
    if [[ -f "${EEPROM_PATH}" ]]; then
        cp -a "${EEPROM_PATH}" "${EEPROM_PATH}.bak.$(date +%Y%m%d_%H%M%S)"
    fi
    rm -f "${EEPROM_PATH}"
fi

WORLD_FILE="${RESOURCE_ROOT}/worlds/${WORLD_NAME}.sdf"
if [[ ! -f "${WORLD_FILE}" ]]; then
    echo "World file not found: ${WORLD_FILE}" >&2
    exit 1
fi

export GZ_SIM_RESOURCE_PATH="${RESOURCE_ROOT}/models:${RESOURCE_ROOT}/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SERVER_CONFIG_PATH="${RESOURCE_ROOT}/server.config"

cleanup() {
    if [[ -n "${GAZEBO_PID:-}" ]] && kill -0 "${GAZEBO_PID}" 2>/dev/null; then
        kill "${GAZEBO_PID}" 2>/dev/null || true
        wait "${GAZEBO_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

GZ_SIM_LOG="${STATE_LOG_DIR}/gz_sim.log"
rm -f "${GZ_SIM_LOG}"
gz sim -r --gui-config "${GUI_CONFIG_FILE}" "${WORLD_FILE}" >"${GZ_SIM_LOG}" 2>&1 &
GAZEBO_PID=$!

for _ in $(seq 1 40); do
    if ! kill -0 "${GAZEBO_PID}" 2>/dev/null; then
        echo "Gazebo exited during startup." >&2
        tail -n 80 "${GZ_SIM_LOG}" >&2 || true
        exit 1
    fi
    sleep 0.25
done

echo "Using SITL binary: ${SITL_BIN}"
echo "Gazebo world: ${WORLD_NAME}, model: ${MODEL_NAME}"
echo "SITL TCP base port: ${TCP_BASE_PORT}"
echo "EEPROM path: ${EEPROM_PATH}"
echo "Gazebo log: ${GZ_SIM_LOG}"

"${SITL_BIN}" \
    --sim=gz \
    --simworld="${WORLD_NAME}" \
    --simmodel="${MODEL_NAME}" \
    --chanmap="${CHANMAP}" \
    --tcpbaseport="${TCP_BASE_PORT}" \
    --path="${EEPROM_PATH}" \
    "${EXTRA_SITL_ARGS[@]}"
