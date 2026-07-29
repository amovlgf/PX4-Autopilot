#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/../../.." && pwd)"
output_dir="${script_dir}/generated/matrix"
log_list="${script_dir}/matrix_logs.txt"
baseline_output_dir="${script_dir}/generated/baseline"
baseline_log_list="${script_dir}/baseline_logs.txt"
log_paths=()
baseline_log_paths=()
with_baseline=false

if [[ "${1:-}" == "--with-baseline" ]]; then
	with_baseline=true

elif [[ $# -gt 0 ]]; then
	echo "usage: $0 [--with-baseline]" >&2
	exit 2
fi

while IFS= read -r entry; do
	if [[ -n "${entry}" && "${entry}" != \#* ]]; then
		log_paths+=("${repo_dir}/${entry}")
	fi
done < "${log_list}"

if [[ ${#log_paths[@]} -eq 0 ]]; then
	echo "no matrix ULogs listed in ${log_list}" >&2
	exit 1
fi

MPLCONFIGDIR="${TMPDIR:-/tmp}/px4-single-motor-matplotlib" \
	python3 "${script_dir}/extract_results.py" \
		--output-dir "${output_dir}" \
		--label "primary-24-case-matrix" \
		"${log_paths[@]}"

plot_args=(
	--results "${output_dir}/results.json"
	--output-dir "${script_dir}/generated/plots"
)

if ${with_baseline}; then
	while IFS= read -r entry; do
		if [[ -n "${entry}" && "${entry}" != \#* ]]; then
			baseline_log_paths+=("${repo_dir}/${entry}")
		fi
	done < "${baseline_log_list}"

	if [[ ${#baseline_log_paths[@]} -eq 0 ]]; then
		echo "no baseline ULogs listed in ${baseline_log_list}" >&2
		exit 1
	fi

	MPLCONFIGDIR="${TMPDIR:-/tmp}/px4-single-motor-matplotlib" \
		python3 "${script_dir}/extract_results.py" \
			--output-dir "${baseline_output_dir}" \
			--label "pre-fix-baseline" \
			"${baseline_log_paths[@]}"
	plot_args+=(--baseline-results "${baseline_output_dir}/results.json")
fi

MPLCONFIGDIR="${TMPDIR:-/tmp}/px4-single-motor-matplotlib" \
	python3 "${script_dir}/generate_plots.py" "${plot_args[@]}"
