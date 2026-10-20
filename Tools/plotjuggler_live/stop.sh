#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=Tools/plotjuggler_live/common.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ $# != 0 ]]; then
    if [[ $# == 1 && ( "$1" == --help || "$1" == -h ) ]]; then
        printf 'Usage: %s\n' "$0"
        printf '%s\n' "Stops only this tool's monitoring container."
        exit 0
    fi
    fail "Unexpected argument: $*"
fi
initialize_state
lock_state
select_docker
container=$(container_id)
if [[ -z "$container" ]]; then
    info 'No monitoring container is present. Simulation and other services were left running.'
    exit 0
fi
assert_owned "$container"
if container_running "$container"; then
    docker_cmd stop --time 10 "$container" >/dev/null
fi
docker_cmd logs "$container" > "$PJ_STATE_DIR/logs/container.log" 2>&1 || true
docker_cmd rm "$container" >/dev/null
if [[ -f "$PJ_STATE_DIR/runtime.json" ]]; then
    mv -f "$PJ_STATE_DIR/runtime.json" "$PJ_STATE_DIR/last-runtime.json"
fi
info 'Monitoring stopped. PX4, jMAVSim, QGroundControl and any reused external Agent remain running.'
