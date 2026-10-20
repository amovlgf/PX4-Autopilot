#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=Tools/plotjuggler_live/common.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
rebuild=0
for argument in "$@"; do
    case "$argument" in
        --rebuild-image) rebuild=1 ;;
        --help|-h) printf 'Usage: %s [--rebuild-image]\nBuild the isolated ROS environment and synchronize local PX4 messages.\n' "$0"; exit 0 ;;
        *) fail "Unknown option: $argument" ;;
    esac
done
initialize_state
lock_state
select_docker
[[ -z $(container_id) ]] || { container=$(container_id); assert_owned "$container"; ! container_running "$container" || fail 'Stop monitoring before rebuilding its workspace; the simulation can remain running.'; }
if (( rebuild )) || ! docker_cmd image inspect "$PJ_IMAGE" >/dev/null 2>&1; then
    info "Building $PJ_IMAGE"
    build_args=(--image "$PJ_IMAGE")
    if [[ "${PJ_DOCKER[0]}" == sudo ]]; then
        build_args+=(--sudo)
    fi
    PJ_STATE_DIR="$PJ_STATE_DIR" bash "$PJ_TOOL_DIR/container/build.sh" "${build_args[@]}"
else
    info "Reusing image $PJ_IMAGE (use --rebuild-image to rebuild)."
fi
common_container_args
info 'Synchronizing the current PX4 message definitions and building the ROS workspace.'
docker_cmd run --rm "${PJ_CONTAINER_ARGS[@]}" "$PJ_IMAGE" bash /opt/px4-monitor/container/run.sh prepare
info "Setup complete. Workspace: $PJ_STATE_DIR/current"
