#!/bin/bash
# compare_baseline.sh <tools dir> <plugin> <runs dir> <baseline dir> <exact> <run>...
#
# Compares each checked run with its summary in the baseline version; with <exact> ON a byte that moved
# fails too, the check for a change that must not move one, such as a refactoring.

set -euo pipefail

if [[ $# -lt 6 ]]; then
    echo "usage: $0 <tools dir> <plugin> <runs dir> <baseline dir> <exact> <run>..." >&2
    exit 2
fi

tools=$1
plugin=$2
runs=$3
baseline=$4
exact=$5
shift 5

exact_flag=()

if [[ "${exact}" == ON ]]; then
    exact_flag=(--exact)
fi

for run in "$@"; do
    python3 "${tools}/baseline.py" compare "${runs}/${run}" "${baseline}/${run}" --plugin "${plugin}" "${exact_flag[@]}"
done
