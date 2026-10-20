#!/usr/bin/env bash
# Shared host helpers. Sourcing this file does not start or stop anything.
set -euo pipefail

PJ_TOOL_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
PJ_PX4_DIR=${PJ_PX4_DIR:-$(cd "$PJ_TOOL_DIR/../.." && pwd -P)}
PJ_CONFIG=${PJ_CONFIG:-$PJ_TOOL_DIR/config/settings.env}
if [[ -f "$PJ_CONFIG" ]]; then
    # A settings file is trusted local shell configuration, never generated input.
    # shellcheck source=/dev/null
    source "$PJ_CONFIG"
fi
PJ_STATE_DIR=${PJ_STATE_DIR:-$PJ_TOOL_DIR/.state}
PJ_IMAGE=${PJ_IMAGE:-px4-plotjuggler-live:humble-2.4.2}
PJ_DDS_PORT=${PJ_DDS_PORT:-8888}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-auto}
PJ_QGC_DIR=${PJ_QGC_DIR:-/home/amovlgf/QGroundControl}
PJ_QGC_BIN=${PJ_QGC_BIN:-$PJ_QGC_DIR/build/Debug/QGroundControl}
PJ_QGC_RUNNER=${PJ_QGC_RUNNER:-$PJ_QGC_DIR/tools/run-qgroundcontrol.sh}
PJ_BUFFER_SECONDS=${PJ_BUFFER_SECONDS:-60}
PJ_PROBE_SECONDS=${PJ_PROBE_SECONDS:-10}
PJ_STARTUP_SECONDS=${PJ_STARTUP_SECONDS:-30}
PJ_LAYOUT=${PJ_LAYOUT:-/opt/px4-monitor/layouts/live.xml}
PJ_CONTAINER_LABEL=org.px4.plotjuggler-live

info() { printf '[plotjuggler] %s\n' "$*"; }
fail() { printf '[plotjuggler] ERROR: %s\n' "$*" >&2; exit 1; }
require_command() { command -v "$1" >/dev/null 2>&1 || fail "Missing command: $1"; }

initialize_state() {
    require_command python3
    require_command flock
    [[ -d "$PJ_PX4_DIR/msg" ]] || fail "Not a PX4 checkout: $PJ_PX4_DIR"
    if [[ ! "$PJ_DDS_PORT" =~ ^[1-9][0-9]{3,4}$ ]] || (( PJ_DDS_PORT < 1024 || PJ_DDS_PORT > 65535 )); then
        fail 'PJ_DDS_PORT must be 1024..65535'
    fi
    if [[ ! "$ROS_DOMAIN_ID" =~ ^(0|[1-9][0-9]{0,2})$ ]] || (( ROS_DOMAIN_ID > 232 )); then
        fail 'ROS_DOMAIN_ID must be 0..232'
    fi
    if [[ ! "$PJ_BUFFER_SECONDS" =~ ^[1-9][0-9]*$ ]] || (( PJ_BUFFER_SECONDS < 10 )); then
        fail 'PJ_BUFFER_SECONDS must be an integer of at least 10 seconds'
    fi
    [[ "$PJ_PROBE_SECONDS" =~ ^[1-9][0-9]*$ ]] || fail 'PJ_PROBE_SECONDS must be a positive integer'
    [[ "$PJ_STARTUP_SECONDS" =~ ^[1-9][0-9]*$ ]] || fail 'PJ_STARTUP_SECONDS must be a positive integer'
    [[ "$ROS_LOCALHOST_ONLY" == auto || "$ROS_LOCALHOST_ONLY" == 0 || "$ROS_LOCALHOST_ONLY" == 1 ]] || fail 'ROS_LOCALHOST_ONLY must be auto, 0 or 1'
    umask 077
    mkdir -p "$PJ_STATE_DIR/logs" "$PJ_STATE_DIR/home" "$PJ_STATE_DIR/workspaces"
    PJ_STATE_DIR=$(cd "$PJ_STATE_DIR" && pwd -P)
    PJ_PX4_DIR=$(cd "$PJ_PX4_DIR" && pwd -P)
    PJ_OWNER=$(printf '%s\n%s\n%s\n' "$PJ_PX4_DIR" "$PJ_STATE_DIR" "$(id -u)" | sha256sum | cut -d ' ' -f1)
    PJ_CONTAINER_NAME="px4-plotjuggler-$(id -u)-${PJ_OWNER:0:12}"
}

lock_state() {
    exec 9>"$PJ_STATE_DIR/lifecycle.lock"
    flock -w 15 9 || fail 'Another setup/start/stop operation is active; retry after it finishes.'
}

# sg executes a POSIX shell command. Quote every argument, including newlines,
# without eval or Bash-only quoting that /bin/sh could interpret differently.
quote_shell_command() {
    local argument
    printf 'exec'
    for argument in "$@"; do
        printf " '%s'" "${argument//\'/\'\\\'\'}"
    done
}

docker_group_exec() {
    local command
    command=$(quote_shell_command "$@")
    if [[ "$PJ_DOCKER_PRIMARY_GROUP" != docker ]]; then
        command=$(quote_shell_command sg "$PJ_DOCKER_PRIMARY_GROUP" -c "$command")
    fi
    # Only this client subprocess changes groups. Keep the management script's
    # original credentials so it can still verify already-running PX4 processes.
    # No controlling terminal or stdin is available for password requests.
    setsid --wait sg docker -c "$command" </dev/null
}

select_granted_docker_group() {
    local account entry docker_gid memberships
    command -v sg >/dev/null 2>&1 && command -v getent >/dev/null 2>&1 \
        && command -v setsid >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1 || return 1
    account=$(id -un) || return 1
    entry=$(getent group docker) || return 1
    IFS=: read -r _ _ docker_gid _ <<< "$entry"
    [[ "$docker_gid" =~ ^[0-9]+$ ]] || return 1
    # A username argument asks NSS for granted groups, rather than the stale
    # supplementary groups inherited by this desktop session.
    memberships=$(id -G "$account") || return 1
    [[ " $memberships " == *" $docker_gid "* ]] || return 1
    PJ_DOCKER_PRIMARY_GROUP=$(id -gn) || return 1
    docker_group_exec timeout 10 docker info >/dev/null 2>&1 || return 1
    PJ_DOCKER=(docker_group_exec docker)
    info 'Using Docker group membership already granted to this user.'
}

select_docker() {
    require_command docker
    if docker info >/dev/null 2>&1; then
        PJ_DOCKER=(docker)
    elif select_granted_docker_group; then
        :
    elif command -v sudo >/dev/null 2>&1 && sudo -n docker info >/dev/null 2>&1; then
        PJ_DOCKER=(sudo -n docker)
    else
        fail 'Docker daemon is unavailable or access is denied. Start Docker or use an authorized shell. Docker group access is reused only after the system confirms membership; no group configuration was changed. A fresh login session may be needed if local group switching is unavailable.'
    fi
}

docker_cmd() { "${PJ_DOCKER[@]}" "$@"; }

container_id() {
    docker_cmd inspect --format '{{.Id}}' "$PJ_CONTAINER_NAME" 2>/dev/null || true
}

assert_owned() {
    local container=$1 owner
    owner=$(docker_cmd inspect --format "{{index .Config.Labels \"$PJ_CONTAINER_LABEL.owner\"}}" "$container" 2>/dev/null) || fail "Cannot inspect container $container"
    [[ "$owner" == "$PJ_OWNER" ]] || fail "Container $container belongs to another session; refusing to change it."
}

container_running() {
    [[ $(docker_cmd inspect --format '{{.State.Running}}' "$1" 2>/dev/null) == true ]]
}

common_container_args() {
    PJ_CONTAINER_ARGS=(--init --network host --ipc host --user "$(id -u):$(id -g)"
        --label "$PJ_CONTAINER_LABEL.owner=$PJ_OWNER"
        --env "HOME=/work/home" --env "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
        --env "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY/auto/0}" --env "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
        --env "PJ_DDS_PORT=$PJ_DDS_PORT" --env "PJ_BUFFER_SECONDS=$PJ_BUFFER_SECONDS"
        --env "PJ_PROBE_SECONDS=$PJ_PROBE_SECONDS" --env "PJ_STARTUP_SECONDS=$PJ_STARTUP_SECONDS" --env "PJ_LAYOUT=$PJ_LAYOUT"
        --mount "type=bind,src=$PJ_PX4_DIR,dst=/px4,readonly"
        --mount "type=bind,src=$PJ_TOOL_DIR,dst=/opt/px4-monitor,readonly"
        --mount "type=bind,src=$PJ_STATE_DIR,dst=/work"
        --workdir /work)
}

# Exact executable matching avoids touching other checkouts or Java processes.
find_process() {
    python3 - "$1" "${2:-all}" <<'PY'
import os, pathlib, sys
expected = os.path.realpath(sys.argv[1])
main_px4_only = sys.argv[2] == "px4-main"
for entry in pathlib.Path('/proc').iterdir():
    if not entry.name.isdigit():
        continue
    try:
        actual = os.readlink(entry / 'exe')
        if actual.endswith(' (deleted)'):
            actual = actual[:-10]
        if os.path.realpath(actual) == expected:
            if main_px4_only:
                command = (entry / 'cmdline').read_bytes().split(b'\0', 1)[0]
                # PX4's CLI symlinks execute the same binary as the main server.
                # They are short-lived clients, not additional SITL instances.
                if not command or os.path.basename(os.fsdecode(command)).startswith('px4-'):
                    continue
            print(entry.name)
    except FileNotFoundError:
        pass
    except PermissionError:
        # A changed primary GID (for example sg docker) can make an existing
        # same-UID PX4 non-inspectable. Its command line is only a candidate,
        # never enough evidence to assume it is absent and launch a duplicate.
        try:
            command = (entry / 'cmdline').read_bytes().split(b'\0', 1)[0]
        except (FileNotFoundError, PermissionError):
            continue
        if os.fsdecode(command) == expected:
            raise SystemExit('Cannot verify existing process PID ' + entry.name +
                             ' whose command matches ' + expected +
                             ': /proc executable access is denied. Open a fresh login session '
                             'with the normal primary group and Docker supplementary group; '
                             'no duplicate process was launched.')
PY
}

find_px4() {
    local pids
    pids=$(find_process "$PJ_PX4_DIR/build/px4_sitl_default/bin/px4" px4-main) || return
    [[ $(printf '%s\n' "$pids" | sed '/^$/d' | wc -l) -le 1 ]] || fail 'Several PX4 instances use this executable; select a single instance before starting this monitor.'
    printf '%s' "$pids"
}

px4_read() {
    timeout 5 "$PJ_PX4_DIR/build/px4_sitl_default/bin/px4-$1" --instance 0 "${@:2}"
}

check_px4_connection_config() {
    local result domain
    local running_pid
    running_pid=$(find_px4)
    if [[ -n "$running_pid" ]] && [[ $(readlink "/proc/$running_pid/exe") == *' (deleted)' ]]; then
        info 'The running PX4 predates the latest build. Existing exported topics remain available; newly added DDS exports require a planned restart. No restart was performed.'
    fi
    result=$(px4_read uxrce_dds_client status 2>&1) || fail "Cannot read PX4 DDS client status: $result"
    printf '%s\n' "$result"
    [[ "$result" == *'Running'* ]] || fail 'PX4 uXRCE-DDS client is not running. Check the existing PX4 startup configuration.'
    [[ "$result" =~ Agent\ port:[[:space:]]*$PJ_DDS_PORT([[:space:]]|$) ]] || fail "PX4 client is not configured for UDP port $PJ_DDS_PORT; align local settings without changing the flight parameters."
    [[ "$result" =~ Agent\ IP:[[:space:]]*127\.0\.0\.1([[:space:]]|$) ]] || fail 'This host-network setup expects the existing PX4 client to use Agent IP 127.0.0.1.'
    local client_localhost=0
    if [[ "$result" =~ Localhost\ only:[[:space:]]*yes ]]; then
        client_localhost=1
    fi
    if [[ "$ROS_LOCALHOST_ONLY" == auto ]]; then
        ROS_LOCALHOST_ONLY=$client_localhost
    elif [[ "$ROS_LOCALHOST_ONLY" != "$client_localhost" ]]; then
        fail "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY differs from the PX4 participant setting ($client_localhost). Use auto to follow the existing client."
    fi
    domain=$(px4_read param show UXRCE_DDS_DOM_ID 2>&1) || fail 'Cannot read the PX4 DDS Domain ID.'
    [[ "$domain" =~ UXRCE_DDS_DOM_ID[^:]*:[[:space:]]*$ROS_DOMAIN_ID([[:space:]]|$) ]] || fail "PX4 Domain ID differs from ROS_DOMAIN_ID=$ROS_DOMAIN_ID. Set the local monitor configuration to the existing PX4 value."
}

# Read the version of the library actually loaded by the listening Agent.
# Agent -v means verbosity, so it must never be used as a version query.
verify_agent_version() {
    python3 - "$1" <<'PY_AGENT'
import pathlib, re, sys
sockets = sys.argv[1]
pids = sorted(set(re.findall(r'"MicroXRCEAgent",pid=(\d+)', sockets)))
if len(pids) != 1:
    raise SystemExit('Cannot identify exactly one external DDS Agent listener')
process = pathlib.Path('/proc') / pids[0]
versions = set()
try:
    versions.update(re.findall(r'libmicroxrcedds_agent\.so\.(\d+\.\d+(?:\.\d+)?)',
                               (process / 'maps').read_text()))
except OSError:
    pass
if not versions:
    manifest = process / 'root/opt/micro-xrce-dds-agent/share/px4-plotjuggler-live/agent-versions.tsv'
    try:
        versions.update(re.findall(r'^Micro-XRCE-DDS-Agent\s+v?(\d+\.\d+\.\d+)',
                                   manifest.read_text(), re.MULTILINE))
    except OSError:
        pass
if not versions:
    raise SystemExit('Cannot verify the external Agent version from its loaded library or version manifest; '
                     'leave it untouched and use a confirmed Agent v2 installation')
if any(version.split('.')[0] != '2' for version in versions):
    raise SystemExit('Incompatible external Agent version: ' + ', '.join(sorted(versions)) +
                     '; this PX4 client requires Agent v2')
print('External Micro XRCE-DDS Agent PID ' + pids[0] + ', verified version ' + ', '.join(sorted(versions)))
PY_AGENT
}

# An occupied port is only reused when the kernel reports the Agent executable.
select_agent() {
    local sockets
    require_command ss
    sockets=$(ss -H -lunp "sport = :$PJ_DDS_PORT")
    if [[ -z "$sockets" ]]; then
        PJ_START_AGENT=1
    elif [[ "$sockets" == *'"MicroXRCEAgent"'* ]]; then
        verify_agent_version "$sockets" || fail 'External Agent compatibility could not be established; no process was changed.'
        PJ_START_AGENT=0
        info "Reusing the existing MicroXRCEAgent on UDP $PJ_DDS_PORT; it will not be stopped."
    else
        fail "UDP $PJ_DDS_PORT is occupied by an unknown process. Inspect it with ss -lunp; no process was stopped."
    fi
    export PJ_START_AGENT
}

write_runtime() {
    python3 - "$PJ_STATE_DIR/runtime.json" "$1" "$PJ_OWNER" "${2:-}" "${3:-}" <<'PY'
import datetime, json, os, pathlib, sys
path, cid, owner, px4_pid, agent_owned = sys.argv[1:]
start_ticks = None
if px4_pid:
    try:
        start_ticks = pathlib.Path('/proc', px4_pid, 'stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        pass
value = dict(container_id=cid, owner=owner, recorded_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
             px4_pid=px4_pid, px4_start_ticks=start_ticks, px4_owned=False, agent_owned=agent_owned == '1')
temporary = pathlib.Path(path + '.tmp')
temporary.write_text(json.dumps(value, indent=2) + '\n')
os.replace(temporary, path)
PY
}

prepare_xauthority() {
    [[ -n "${DISPLAY:-}" ]] || fail 'DISPLAY is missing. Run from the desktop session, or use --no-gui for diagnostics.'
    require_command xauth
    local source_auth=${XAUTHORITY:-${HOME}/.Xauthority} records
    [[ -r "$source_auth" ]] || fail "Cannot read X11 authorization file $source_auth. Set XAUTHORITY to the desktop session cookie file."
    records=$(xauth -f "$source_auth" nlist "$DISPLAY" 2>/dev/null)
    [[ -n "$records" ]] || fail "No X11 authorization cookie for DISPLAY=$DISPLAY in $source_auth"
    local temporary="$PJ_STATE_DIR/xauthority.tmp"
    : > "$temporary"
    chmod 600 "$temporary"
    printf '%s\n' "$records" | sed 's/^..../ffff/' | xauth -f "$temporary" nmerge -
    mv -f "$temporary" "$PJ_STATE_DIR/xauthority"
    PJ_CONTAINER_ARGS+=(--env "DISPLAY=$DISPLAY" --env "XAUTHORITY=/tmp/monitor.xauthority"
        --env "QT_X11_NO_MITSHM=1" --env "QT_QPA_PLATFORM=xcb"
        --mount "type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly"
        --mount "type=bind,src=$PJ_STATE_DIR/xauthority,dst=/tmp/monitor.xauthority,readonly")
}

open_terminal() {
    local title=$1 command=$2
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$title" -- bash -lc "$command" 9>&-
    elif command -v konsole >/dev/null 2>&1; then
        konsole --new-tab -p "tabtitle=$title" -e bash -lc "$command" 9>&- &
    elif command -v xterm >/dev/null 2>&1; then
        xterm -T "$title" -e bash -lc "$command" 9>&- &
    else
        fail 'A GUI terminal is required for --launch (gnome-terminal, konsole, or xterm).'
    fi
}

launch_simulation() {
    local px4_pid=$1 command quoted_px4 quoted_qgc quoted_binary quoted_runner qgc_pids
    [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || fail '--launch requires a desktop display.'
    [[ -x "$PJ_QGC_BIN" && -x "$PJ_QGC_RUNNER" ]] || fail "QGC Debug binary or its wrapper is missing: $PJ_QGC_BIN / $PJ_QGC_RUNNER"
    if [[ -z "$px4_pid" ]]; then
        printf -v quoted_px4 '%q' "$PJ_PX4_DIR"
        command="unset HEADLESS; cd $quoted_px4 && make px4_sitl_default jmavsim; result=\$?; printf '\\nPX4 exited (%s). Press Enter to close.\\n' \"\$result\"; read -r _; exit \"\$result\""
        info "Launching the existing jMAVSim command on branch $(git -C "$PJ_PX4_DIR" branch --show-current)"
        open_terminal 'PX4 jMAVSim Console' "$command"
    else
        info "PX4 PID $px4_pid is already running; attaching."
    fi
    qgc_pids=$(find_process "$PJ_QGC_BIN") || return
    if [[ -z "$qgc_pids" ]]; then
        printf -v quoted_qgc '%q' "$PJ_QGC_DIR"
        printf -v quoted_binary '%q' "$PJ_QGC_BIN"
        printf -v quoted_runner '%q' "$PJ_QGC_RUNNER"
        open_terminal QGroundControl "cd $quoted_qgc && QGC_BINARY=$quoted_binary $quoted_runner"
    else
        info 'Reusing the running QGroundControl.'
    fi
}
