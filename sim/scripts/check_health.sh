#!/bin/bash
# check_health.sh <tools dir> <runs dir> <run>...
#
# The checked runs' health: no warning, collision or non-finite sample, and no unbound port, watchdog
# expiry, emergency stop or dropped byte.

set -euo pipefail

if [[ $# -lt 3 ]]; then
    echo "usage: $0 <tools dir> <runs dir> <run>..." >&2
    exit 2
fi

tools=$1
runs=$2
shift 2

paths=()

for run in "$@"; do
    paths+=("${runs}/${run}")
done

python3 "${tools}/check_run.py" --expect unbound_ports=0 --expect watchdog_expiries=0 --expect emergency_stops=0 \
    --expect serial_dropped_bytes=0 "${paths[@]}"
