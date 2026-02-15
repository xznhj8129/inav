#!/bin/bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="$REPO_ROOT/src/utils/gazebo/state"
BUILD_DIR="$STATE_DIR/build"
CLEAN_BUILD=0

if [[ "${1:-}" == "clean" || "${1:-}" == "--clean" ]]; then
    CLEAN_BUILD=1
fi

export CCACHE_DISABLE=1
export CCACHE_TEMPDIR=/tmp

if [[ "$CLEAN_BUILD" -eq 1 ]]; then
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
#cmake -DSITL=ON -DWARNINGS_AS_ERRORS=ON -GNinja -B build_SITL ..
cmake -DSITL=ON -DDEBUG=ON -DWARNINGS_AS_ERRORS=ON -GNinja -B "$BUILD_DIR" "$REPO_ROOT"
cmake --build "$BUILD_DIR"
