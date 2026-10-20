#!/usr/bin/env bash
set -euo pipefail

container_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
monitor_dir=$(dirname -- "$container_dir")
state_dir=${PJ_STATE_DIR:-"$monitor_dir/.state"}
image_name=px4-plotjuggler-live:humble-2.4.2
base_image=px4-monitor-ubuntu:22.04.5
proxy_mode=${PJ_BUILD_PROXY:-none}
ubuntu_rootfs_sha256=242cd8898b33ea806ef5f13b1076ed7c76f9f989d18384452f7166692438ff1a
use_sudo=false

usage() {
    cat <<'EOF'
Usage: container/build.sh [--sudo] [--image NAME] [--base-image NAME]
                          [--no-proxy | --use-proxy]

Build the Ubuntu 22.04 / ROS Humble monitor image. Project sources are mounted
at runtime. The default base is a SHA256-verified official Ubuntu-base rootfs,
imported locally; Docker Hub access is not required. --base-image can select an
existing locally loaded Ubuntu 22.04 image, or a registry image to pull.

--no-proxy (default, PJ_BUILD_PROXY=none) clears proxy build arguments only.
--use-proxy (PJ_BUILD_PROXY=inherit) forwards the current proxy variables.
No host shell, system proxy, Docker daemon or group configuration is changed.
--sudo uses terminal authentication when needed; passwords are never read here.
EOF
}

while (($#)); do
    case "$1" in
        --sudo) use_sudo=true; shift ;;
        --no-proxy) proxy_mode=none; shift ;;
        --use-proxy) proxy_mode=inherit; shift ;;
        --image|--base-image)
            (($# >= 2)) || { printf '%s requires a value\n' "$1" >&2; exit 2; }
            if [[ $1 == --image ]]; then image_name=$2; else base_image=$2; fi
            shift 2 ;;
        --help|-h) usage; exit 0 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
done
[[ $proxy_mode == none || $proxy_mode == inherit ]] || {
    printf '%s\n' 'PJ_BUILD_PROXY must be none or inherit' >&2; exit 2;
}

if ! command -v docker >/dev/null 2>&1; then
    printf '%s\n' 'Docker is not installed. In a local terminal run:' \
        '  sudo apt-get update' '  sudo apt-get install docker.io' >&2
    exit 1
fi

docker_cmd=(docker)
if "$use_sudo"; then
    if [[ -t 0 && -t 2 ]]; then
        sudo -v
    elif ! sudo -n true 2>/dev/null; then
        printf '%s\n' 'Docker needs administrator access. Run container/build.sh --sudo in your local terminal.' >&2
        exit 1
    fi
    docker_cmd=(sudo -n "--preserve-env=http_proxy,https_proxy,HTTP_PROXY,HTTPS_PROXY,ALL_PROXY,all_proxy,NO_PROXY,no_proxy" docker)
else
    # Reuse the same already-granted group fallback as setup/start/doctor.
    # shellcheck source=Tools/plotjuggler_live/common.sh
    source "$monitor_dir/common.sh"
    select_docker
    docker_cmd=("${PJ_DOCKER[@]}")
fi
if ! "${docker_cmd[@]}" info >/dev/null 2>&1; then
    printf '%s\n' 'Cannot access the Docker daemon. Check that it is running.' \
        'If the socket requires administrator access, run container/build.sh --sudo in your local terminal.' >&2
    exit 1
fi

bootstrap_base() {
    local checksum=$ubuntu_rootfs_sha256
    local filename=ubuntu-base-22.04.5-base-amd64.tar.gz
    local url=https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release
    local cache="$state_dir/ubuntu-base" actual=''
    local curl_args=(--fail --silent --show-error --location --retry 1 --connect-timeout 15 --max-time 180)
    [[ $(uname -m) == x86_64 ]] || {
        printf '%s\n' 'The pinned rootfs is amd64. Supply an Ubuntu 22.04 image using --base-image on other architectures.' >&2
        return 1
    }
    mkdir -p "$cache"
    if [[ -f $cache/$filename ]]; then actual=$(sha256sum "$cache/$filename"); fi
    if [[ ${actual%% *} != "$checksum" ]]; then
        command -v curl >/dev/null || { printf '%s\n' 'curl is required to obtain the official Ubuntu base.' >&2; return 1; }
        if [[ $proxy_mode == none ]]; then curl_args+=(--noproxy '*'); fi
        curl "${curl_args[@]}" "$url/$filename" --output "$cache/$filename.partial" || return $?
        actual=$(sha256sum "$cache/$filename.partial")
        [[ ${actual%% *} == "$checksum" ]] || { printf '%s\n' 'Ubuntu rootfs SHA256 mismatch; refusing import.' >&2; return 1; }
        mv -- "$cache/$filename.partial" "$cache/$filename"
    fi
    "${docker_cmd[@]}" import --change 'CMD ["bash"]' \
        --change 'LABEL org.opencontainers.image.version="22.04.5"' \
        --change "LABEL org.opencontainers.image.revision=\"sha256:$checksum\"" \
        "$cache/$filename" "$base_image" >&2
}

if ! "${docker_cmd[@]}" image inspect "$base_image" >/dev/null 2>&1; then
    if [[ $base_image == px4-monitor-ubuntu:22.04.5 ]]; then
        bootstrap_base
    else
        "${docker_cmd[@]}" pull "$base_image"
    fi
fi
# Imported archives often have no RepoDigests. Their local tag is reused without
# a registry request; the immutable local image ID is recorded below.
base_digest=$("${docker_cmd[@]}" image inspect --format '{{if .RepoDigests}}{{index .RepoDigests 0}}{{end}}' "$base_image")
agent_base=${base_digest:-$base_image}
ros_base=$agent_base
build_args=(--network host --build-arg "AGENT_BUILD_IMAGE=$agent_base"
    --build-arg "ROS_BASE_IMAGE=$ros_base")
for proxy_name in http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy NO_PROXY no_proxy; do
    if [[ $proxy_mode == none ]]; then
        build_args+=(--build-arg "$proxy_name=")
    elif [[ -n ${!proxy_name:-} ]]; then
        # Passing the name avoids printing credentials in command arguments.
        build_args+=(--build-arg "$proxy_name")
    fi
done
"${docker_cmd[@]}" build "${build_args[@]}" --tag "$image_name" "$container_dir"

mkdir -p "$state_dir"
manifest_tmp=$(mktemp "$state_dir/image-build.XXXXXX")
trap 'rm -f -- "$manifest_tmp"' EXIT
{
    printf 'image\t%s\n' "$image_name"
    printf 'image_id\t%s\n' "$("${docker_cmd[@]}" image inspect --format '{{.Id}}' "$image_name")"
    printf 'agent_base\t%s\nros_base\t%s\n' "$agent_base" "$ros_base"
    printf 'base_image_id\t%s\n' "$("${docker_cmd[@]}" image inspect --format '{{.Id}}' "$base_image")"
    printf 'build_proxy_mode\t%s\n' "$proxy_mode"
    if [[ $base_image == px4-monitor-ubuntu:22.04.5 ]]; then
        printf 'ubuntu_rootfs_sha256\t%s\n' "$ubuntu_rootfs_sha256"
    fi
    printf 'ros_key_sha256\t%s\n' "$(sha256sum "$container_dir/ros-archive-keyring.gpg" | cut -d ' ' -f 1)"
    "${docker_cmd[@]}" run --rm --network none --user "$(id -u):$(id -g)" \
        "$image_name" bash -c \
        'cat /opt/micro-xrce-dds-agent/share/px4-plotjuggler-live/agent-versions.tsv; cat /usr/local/share/px4-plotjuggler-live/packages.tsv'
} > "$manifest_tmp"
mv -- "$manifest_tmp" "$state_dir/image-build.tsv"
printf 'Built %s\nVersion manifest: %s\n' "$image_name" "$state_dir/image-build.tsv"
