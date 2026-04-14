#!/usr/bin/env bash
# 编译前先执行仓库根目录的 `make distclean`，再编译指定配置。
# Usage（在仓库根目录）:
#   ./boards/amovlab/icf6/fresh_build.sh amovlab_icf6_test -j8
#   ./boards/amovlab/icf6/fresh_build.sh amovlab_icf6_default -j8
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if [[ $# -lt 1 ]]; then
	echo "用法: $0 <make 配置目标> [传给 make 的额外参数...]" >&2
	echo "示例: $0 amovlab_icf6_test -j8" >&2
	exit 1
fi

TARGET="$1"
shift

echo "执行 make distclean（仓库根目录）…"
make -C "${REPO_ROOT}" distclean

echo "开始编译: make ${TARGET} $*"
make -C "${REPO_ROOT}" "${TARGET}" "$@"
