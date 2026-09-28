#!/bin/bash
# record_baseline.sh <tools dir> <plugin> <runs dir> <baseline dir> <run>...
#
# Records a new baseline version from the checked runs. baseline.py refuses to overwrite a version, so
# an older one is never lost: a change of behavior is a new version.

set -euo pipefail

if [[ $# -lt 5 ]]; then
    echo "usage: $0 <tools dir> <plugin> <runs dir> <baseline dir> <run>..." >&2
    exit 2
fi

tools=$1
plugin=$2
runs=$3
baseline=$4
shift 4

for run in "$@"; do
    python3 "${tools}/baseline.py" record "${runs}/${run}" "${baseline}/${run}" --plugin "${plugin}"
done
