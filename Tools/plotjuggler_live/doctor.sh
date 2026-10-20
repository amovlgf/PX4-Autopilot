#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=Tools/plotjuggler_live/common.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [[ $# != 0 ]]; then
    if [[ $# == 1 && ( "$1" == --help || "$1" == -h ) ]]; then
        printf 'Usage: %s\n' "$0"
        printf '%s\n' "Checks the environment and actually receives ROS messages."
        exit 0
    fi
    fail "Unexpected argument: $*"
fi
initialize_state
lock_state
select_docker
info "Checkout: $PJ_PX4_DIR"
info "Image: $PJ_IMAGE; UDP port: $PJ_DDS_PORT; Domain: $ROS_DOMAIN_ID"
px4_pid=$(find_px4)
[[ -n "$px4_pid" ]] || fail 'The selected PX4 executable is not running.'
check_px4_connection_config
[[ -f "$PJ_STATE_DIR/current/install/setup.bash" ]] || fail 'No built ROS workspace; run setup.sh.'
container=$(container_id)
if [[ -n "$container" ]]; then
    assert_owned "$container"
    if container_running "$container"; then
        docker_cmd exec "$container" bash /opt/px4-monitor/container/run.sh diagnose
        exit $?
    fi
    docker_cmd logs --tail 30 "$container" >&2 || true
    fail 'The monitoring container has exited. Review the Agent/attitude/PlotJuggler logs and restart monitoring; GUI health is not established.'
fi
select_agent
[[ "$PJ_START_AGENT" == 0 ]] || fail 'No Agent is listening. Run start.sh --attach (or --no-gui), then doctor.sh.'
common_container_args
docker_cmd run --rm "${PJ_CONTAINER_ARGS[@]}" "$PJ_IMAGE" bash /opt/px4-monitor/container/run.sh diagnose
