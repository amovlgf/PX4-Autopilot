#!/usr/bin/env bash
set -euo pipefail

agent_commit=57d086216d01ec43121845d385894a25987f8a2c
fastdds_commit=092848725b8425e4f05a8ccf7b3b8d513fabf733
agent_prefix=/opt/micro-xrce-dds-agent
agent_source=/tmp/Micro-XRCE-DDS-Agent

git clone --depth 1 --branch v2.4.2 \
    https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$agent_source"
test "$(git -C "$agent_source" rev-parse HEAD)" = "$agent_commit"

# Upstream 2.4.2 follows the moving 2.12.x branch. Pin its compatible dependency
# to the verified v2.12.2 commit, leaving all flight-controller sources untouched.
python3 - "$agent_source/CMakeLists.txt" "$fastdds_commit" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()
original = 'set(_fastdds_tag 2.12.x)'
if text.count(original) != 1:
    raise SystemExit('Agent source changed: expected exactly one Fast DDS version setting')
path.write_text(text.replace(original, f'set(_fastdds_tag {sys.argv[2]})'))
PY

export CMAKE_BUILD_PARALLEL_LEVEL=2
cmake -S "$agent_source" -B "$agent_source/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$agent_prefix" \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DUAGENT_BUILD_TESTS=OFF \
    -DUAGENT_BUILD_USAGE_EXAMPLES=OFF \
    -DUAGENT_P2P_PROFILE=OFF
cmake --build "$agent_source/build" --parallel 2
cmake --install "$agent_source/build"

mkdir -p "$agent_prefix/share/px4-plotjuggler-live"
{
    printf 'Micro-XRCE-DDS-Agent\tv2.4.2\t%s\n' "$agent_commit"
    printf 'Fast-DDS\tv2.12.2\t%s\n' "$fastdds_commit"
    printf 'Fast-CDR\tv1.1.1\nspdlog\tv1.9.2\nfoonathan_memory\tv0.7-3\n'
} > "$agent_prefix/share/px4-plotjuggler-live/agent-versions.tsv"
