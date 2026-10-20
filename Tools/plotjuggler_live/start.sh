#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=Tools/plotjuggler_live/common.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
mode=attach
selected_mode=
gui=1
for argument in "$@"; do
    case "$argument" in
        --attach|--launch)
            [[ -z "$selected_mode" || "$selected_mode" == "$argument" ]] || fail '--attach and --launch are mutually exclusive.'
            selected_mode=$argument
            mode=${argument#--}
            ;;
        --no-gui) gui=0 ;;
        --help|-h) printf 'Usage: %s [--attach|--launch] [--no-gui]\nAttach is the default. Simulation and QGC remain open when monitoring stops.\n' "$0"; exit 0 ;;
        *) fail "Unknown option: $argument" ;;
    esac
done
initialize_state
lock_state
select_docker
[[ -f "$PJ_STATE_DIR/current/install/setup.bash" ]] || fail "ROS workspace is missing. Run $PJ_TOOL_DIR/setup.sh first."
container=$(container_id)
if [[ -n "$container" ]]; then
    assert_owned "$container"
    if container_running "$container"; then
        px4_pid=$(find_px4)
        [[ -n "$px4_pid" ]] || fail 'The monitoring container exists, but the selected PX4 has stopped. Restart the original simulation, then run doctor.sh.'
        check_px4_connection_config
        info 'Monitoring is already running; checking real message reception again.'
        docker_cmd exec "$container" bash /opt/px4-monitor/container/run.sh probe
        info 'The existing monitoring container was reused. Review probe warnings for unavailable control/PID data.'
        exit 0
    fi
    docker_cmd rm "$container" >/dev/null
fi
px4_pid=$(find_px4)
if [[ "$mode" == launch ]]; then
    launch_simulation "$px4_pid"
    for ((attempt=0; attempt<120; attempt++)); do
        px4_pid=$(find_px4)
        [[ -n "$px4_pid" ]] && break
        sleep 1
    done
    # A newly launched PX4 becomes visible before its startup script finishes.
    if [[ -n "$px4_pid" ]]; then
        for ((attempt=0; attempt<30; attempt++)); do
            status=$(px4_read uxrce_dds_client status 2>&1 || true)
            [[ "$status" == *Running* ]] && break
            sleep 1
        done
    fi
fi
[[ -n "$px4_pid" ]] || fail 'No running PX4 from this checkout. Start the existing jMAVSim session or use --launch.'
check_px4_connection_config
select_agent
common_container_args
if (( gui )); then
    prepare_xauthority
fi
PJ_CONTAINER_ARGS+=(--env "PJ_START_AGENT=$PJ_START_AGENT" --env "PJ_GUI=$gui")
container=
started=0
launch_token="$$-$(date +%s%N)"
PJ_CONTAINER_ARGS+=(--label "$PJ_CONTAINER_LABEL.launch=$launch_token")
cleanup_failed_start() {
    local result=$?
    trap - EXIT
    if (( ! started )) && [[ -z "$container" ]]; then
        # Docker may create the container and then fail before returning its ID.
        # Recover it only if its per-attempt tag proves this exact run created it.
        local candidate candidate_token
        candidate=$(container_id)
        if [[ -n "$candidate" ]]; then
            candidate_token=$(docker_cmd inspect --format "{{index .Config.Labels \"$PJ_CONTAINER_LABEL.launch\"}}" "$candidate" 2>/dev/null || true)
            if [[ "$candidate_token" == "$launch_token" ]]; then
                container=$candidate
            fi
        fi
    fi
    if (( ! started )) && [[ -n "$container" ]]; then
        info "Monitoring startup failed; logs: $PJ_STATE_DIR/logs/container.log"
        docker_cmd logs "$container" > "$PJ_STATE_DIR/logs/container.log" 2>&1 || true
        # ID and owner are both checked; the simulation and an external Agent are untouched.
        assert_owned "$container"
        docker_cmd rm --force "$container" >/dev/null 2>&1 || true
    fi
    exit "$result"
}
trap cleanup_failed_start EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
container=$(docker_cmd run --detach --name "$PJ_CONTAINER_NAME" "${PJ_CONTAINER_ARGS[@]}" "$PJ_IMAGE" bash /opt/px4-monitor/container/run.sh monitor)
write_runtime "$container" "$px4_pid" "$PJ_START_AGENT"
# Readiness is published only after actual BEST_EFFORT reception succeeds.
for ((attempt=0; attempt<PJ_STARTUP_SECONDS+45; attempt++)); do
    if ! container_running "$container"; then
        docker_cmd logs "$container" >&2 || true
        fail 'Monitoring container exited during startup. See the diagnostic output above.'
    fi
    if docker_cmd exec "$container" test -f /tmp/px4-monitor-ready; then
        started=1
        break
    fi
    sleep 1
done
(( started )) || fail 'Timed out waiting for real PX4 ROS messages. Run doctor.sh after checking DDS configuration.'
info "Monitoring attached to PX4 PID $px4_pid, DDS port $PJ_DDS_PORT, Domain $ROS_DOMAIN_ID."
if (( gui )); then
    info 'PlotJuggler is opening. In the ROS 2 streamer, confirm the preselected topics to begin plotting.'
else
    info 'GUI disabled; the Agent and attitude conversion are running for diagnostics.'
fi
info 'stop.sh stops only this monitoring container; PX4, jMAVSim and QGC remain running.'
