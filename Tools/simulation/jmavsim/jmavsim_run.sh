#!/usr/bin/env bash

set -euo pipefail

# The normal CMake launch sets JMAVSIM_BUILD_DIR explicitly. The fallback
# supports invoking the rootfs symlink from its own PX4 build directory.
build_dir=${JMAVSIM_BUILD_DIR:-"$(cd .. && pwd)/jmavsim"}
sensor_config=${JMAVSIM_SENSOR_CONFIG:-"$build_dir/config/position_source_test.properties"}
java_command=${JMAVSIM_JAVA:-java}
port=4560
baudrate=921600
device=
ip=127.0.0.1
protocol=tcp
extra_args=()

while getopts "b:d:ui:p:qsr:loat" opt; do
	case "$opt" in
		b) baudrate=$OPTARG ;;
		d) device=$OPTARG ;;
		u) protocol=udp ;;
		i) ip=$OPTARG ;;
		p) port=$OPTARG ;;
		q) extra_args+=(-qgc) ;;
		s) extra_args+=(-sdk) ;;
		r) extra_args+=(-r "$OPTARG") ;;
		l) extra_args+=(-lockstep) ;;
		o) extra_args+=(-disponly) ;;
		a) extra_args+=(-fw) ;;
		t) extra_args+=(-ts) ;;
		*) exit 1 ;;
	esac
done

if [[ -n "$device" ]]; then
	connection=(-serial "$device" "$baudrate")
elif [[ "$protocol" == tcp ]]; then
	connection=(-tcp "$ip:$port")
else
	connection=(-udp "$port")
fi

if [[ ${HEADLESS:-0} == 1 ]]; then
	extra_args+=(-no-gui)
fi

if [[ ! -f "$build_dir/out/production/jmavsim_run.jar" || ! -f "$sensor_config" ]]; then
	echo "jMAVSim build or sensor configuration missing; build the build_jmavsim target first." >&2
	exit 1
fi

cd "$build_dir/out/production"
exec "$java_command" --add-exports java.base/java.lang=ALL-UNNAMED \
	--add-exports java.desktop/sun.awt=ALL-UNNAMED \
	--add-exports java.desktop/sun.java2d=ALL-UNNAMED \
	-XX:GCTimeRatio=20 -Djava.ext.dirs= "-Djmavsim.sensorConfig=$sensor_config" \
	-jar jmavsim_run.jar "${connection[@]}" "${extra_args[@]}"
