#!/usr/bin/env bash
# Runs inside the pinned ROS container, as the host user's UID/GID.
set -euo pipefail
umask 077
mkdir -p /work/logs /work/home
export HOME=/work/home
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

load_ros() {
    set +u
    # shellcheck disable=SC1090,SC1091
    source /opt/ros/humble/setup.bash
    if [[ -f /work/current/install/setup.bash ]]; then
        # shellcheck disable=SC1090,SC1091
        source /work/current/install/setup.bash
    fi
    set -u
}

check_components() {
    python3 -c 'import rclpy, px4_msgs.msg; print("ROS 2 and locally synchronized px4_msgs: available")' || return
    command -v MicroXRCEAgent >/dev/null || { echo "Micro XRCE-DDS Agent executable is missing." >&2; return 1; }
    local prefix
    prefix=$(ros2 pkg prefix plotjuggler) || return
    [[ -x "$prefix/lib/plotjuggler/plotjuggler" ]] || { echo 'PlotJuggler executable is missing.' >&2; return 1; }
    [[ -n $(find /opt/ros/humble/lib -type f -name '*DataStreamROS2*.so' -print -quit) ]] || {
        echo 'PlotJuggler ROS 2 subscriber plugin is missing. Rebuild the monitoring image.' >&2
        return 1
    }
    printf 'Micro XRCE-DDS Agent, PlotJuggler and ROS 2 subscriber plugin: available\n'
}

probe() {
    check_components || return
    python3 /opt/px4-monitor/python/probe.py --manifest /work/current/manifest.json \
        --duration "${PJ_PROBE_SECONDS:-10}" --require-core --json-output /work/logs/probe.json || return
    python3 - <<'PY_REPORT'
import json
from pathlib import Path
report = json.loads(Path('/work/logs/probe.json').read_text())
print('Core telemetry reception passed; additional topic/validity warnings: ' + str(len(report['warnings'])))
print('Complete monitoring data: ' + ('available' if report['monitoring_complete'] else 'INCOMPLETE'))
if report['missing_monitoring_topics']:
    print('Unavailable or invalid monitoring topics: ' + ', '.join(item['topic'] for item in report['missing_monitoring_topics']))
print('Data reception does not establish visible PlotJuggler curves; GUI confirmation remains separate.')
PY_REPORT
}

gui_status() {
    local prefix
    if [[ "${PJ_GUI:-0}" == 1 ]]; then
        prefix=$(ros2 pkg prefix plotjuggler) || return
    else
        prefix=
    fi
    timeout 5 python3 - "$prefix/lib/plotjuggler/plotjuggler" <<'PY_GUI'
import ctypes
import os
from pathlib import Path
import sys

if os.environ.get('PJ_GUI', '0') != '1':
    print('GUI: disabled for this session; no display or PlotJuggler process is required.')
    raise SystemExit(0)

display = os.environ.get('DISPLAY')
if not display:
    raise SystemExit('GUI: enabled but DISPLAY is missing; reconnect from the desktop session.')
try:
    x11 = ctypes.CDLL('libX11.so.6')
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.restype = ctypes.c_int
    connection = x11.XOpenDisplay(display.encode())
    if not connection:
        raise SystemExit('GUI: cannot connect to DISPLAY=' + display +
                         '; check the desktop session, X11 socket and Xauthority cookie.')
    x11.XCloseDisplay(connection)
except OSError as error:
    raise SystemExit('GUI: X11 client library could not be loaded: ' + str(error))

expected = os.path.realpath(sys.argv[1])
pids = []
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():
        continue
    try:
        if os.path.realpath(os.readlink(entry / 'exe')) == expected:
            pids.append(entry.name)
    except (FileNotFoundError, PermissionError):
        continue
if not pids:
    raise SystemExit('GUI: X11 connection works, but the PlotJuggler application process is absent. '
                     'Inspect /work/logs/plotjuggler.log; data reception alone does not establish GUI health.')
print('GUI: X11 connection works; PlotJuggler process PID(s): ' + ', '.join(pids))
print('GUI: window rendering, topic selection and visible updating curves still require visual confirmation.')
PY_GUI
}

case "${1:-}" in
prepare)
    # Source only the base ROS environment while building a new workspace.
    set +u
    # shellcheck disable=SC1090,SC1091
    source /opt/ros/humble/setup.bash
    set -u
    synchronization=$(python3 /opt/px4-monitor/python/sync_messages.py --px4-root /px4 --work-root /work)
    workspace=$(printf '%s' "$synchronization" | python3 -c 'import json,sys; print(json.load(sys.stdin)["workspace"])')
    [[ "$workspace" == /work/workspaces/* && -f "$workspace/manifest.json" ]] || {
        echo "Message synchronization returned an invalid workspace: $workspace" >&2
        exit 1
    }
    if [[ ! -f "$workspace/.build-complete" || ! -f "$workspace/install/setup.bash" ]]; then
        cd "$workspace"
        CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --base-paths src --merge-install --event-handlers console_direct+
        touch "$workspace/.build-complete"
    else
        printf 'Reusing the already built message workspace: %s\n' "$workspace"
    fi
    set +u
    # shellcheck disable=SC1090,SC1091
    source "$workspace/install/setup.bash"
    set -u
    check_components
    # A relative link works both on the host and at the /work bind mount.
    python3 - "$workspace" <<'PY'
import os, pathlib, sys
workspace = pathlib.Path(sys.argv[1]).resolve()
work = pathlib.Path('/work').resolve()
relative = workspace.relative_to(work)
current = work / 'current'
if current.exists() and not current.is_symlink():
    raise SystemExit('/work/current is an existing non-symlink; refusing to replace it')
temporary = work / ('.current-' + str(os.getpid()))
temporary.symlink_to(relative, target_is_directory=True)
os.replace(temporary, current)
PY
    ;;
probe)
    load_ros
    probe
    ;;
diagnose)
    load_ros
    telemetry_result=0
    gui_result=0
    probe || telemetry_result=$?
    gui_status || gui_result=$?
    if (( telemetry_result != 0 || gui_result != 0 )); then
        exit 1
    fi
    ;;
monitor)
    load_ros
    check_components
    children=()
    cleanup() {
        local result=$?
        trap - EXIT INT TERM
        rm -f /tmp/px4-monitor-ready
        for child in "${children[@]}"; do
            kill -TERM "$child" 2>/dev/null || true
        done
        # Docker's init reaps descendants; Docker stop supplies a bounded fallback.
        for child in "${children[@]}"; do
            wait "$child" 2>/dev/null || true
        done
        exit "$result"
    }
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    if [[ "${PJ_START_AGENT:-1}" == 1 ]]; then
        MicroXRCEAgent udp4 -p "${PJ_DDS_PORT:-8888}" > /work/logs/agent.log 2>&1 &
        children+=("$!")
    fi
    python3 /opt/px4-monitor/python/attitude_node.py --manifest /work/current/manifest.json > /work/logs/attitude.log 2>&1 &
    children+=("$!")
    # Initial PX4 time synchronization needs 500 samples before topics exist.
    PJ_PROBE_SECONDS="${PJ_STARTUP_SECONDS:-30}" probe
    if [[ "${PJ_GUI:-1}" == 1 ]]; then
        # In PlotJuggler 3.17.2 the CLI value is the spin-box maximum, and
        # reaching that maximum means infinite storage. Its saved setting also
        # overrides the layout during startup. Set only our isolated preference.
        python3 - "${PJ_BUFFER_SECONDS:-60}" <<'PY_BUFFER'
import os
from pathlib import Path
import sys
path = Path('/work/home/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini')
path.parent.mkdir(parents=True, exist_ok=True)
lines = path.read_text().splitlines(keepends=True) if path.exists() else []
key = 'MainWindow.streamingBufferValue='
setting = key + str(int(sys.argv[1])) + '\n'
general = False
replaced = False
for index, line in enumerate(lines):
    if line.strip().startswith('['):
        general = line.strip() == '[General]'
    elif general and line.startswith(key):
        lines[index] = setting
        replaced = True
if not replaced:
    for index, line in enumerate(lines):
        if line.strip() == '[General]':
            lines.insert(index + 1, setting)
            break
    else:
        lines = ['[General]\n', setting, '\n'] + lines
temporary = path.with_suffix('.ini.tmp')
temporary.write_text(''.join(lines))
os.replace(temporary, path)
PY_BUFFER
        buffer_upper=$(( ${PJ_BUFFER_SECONDS:-60} + 1 ))
        plotjuggler_args=(--disable_opengl --buffer_size "$buffer_upper")
        selected_layout=
        if [[ -f /work/current/layouts/live.xml ]]; then
            selected_layout=/work/current/layouts/live.xml
        elif [[ -f "${PJ_LAYOUT:-/opt/px4-monitor/layouts/live.xml}" ]]; then
            selected_layout=${PJ_LAYOUT:-/opt/px4-monitor/layouts/live.xml}
        fi
        if [[ -n "$selected_layout" ]]; then
            python3 /opt/px4-monitor/python/validate_layout.py \
                --manifest /work/current/manifest.json --layout "$selected_layout"
            plotjuggler_args+=(--layout "$selected_layout")
        fi
        ros2 run plotjuggler plotjuggler "${plotjuggler_args[@]}" > /work/logs/plotjuggler.log 2>&1 &
        children+=("$!")
        sleep 1
    fi
    for child in "${children[@]}"; do
        kill -0 "$child" 2>/dev/null || { echo "Monitoring component exited; inspect /work/logs." >&2; exit 1; }
    done
    touch /tmp/px4-monitor-ready
    wait -n "${children[@]}"
    ;;
*)
    printf 'Usage: %s {prepare|monitor|probe|diagnose}\n' "$0" >&2
    exit 2
    ;;
esac
