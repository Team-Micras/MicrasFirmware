#!/bin/bash
# contest.sh <simulator> <scenario> <out dir> <tools dir> <maze>...
#
# The whole contest in every maze at once, exploring and then the fastest run, and its health. A row
# every 10 ms is plenty for the health, and the events are still found every tick.

set -euo pipefail

if [[ $# -lt 5 ]]; then
    echo "usage: $0 <simulator> <scenario> <out dir> <tools dir> <maze>..." >&2
    exit 2
fi

simulator=$1
scenario=$2
out=$3
tools=$4
shift 4

runs=()

for maze in "$@"; do
    mkdir -p "${out}/${maze}"
    "${simulator}" --scenario "${scenario}" --maze "${maze}" --out "${out}/${maze}" --record-every 80 \
        > "${out}/${maze}/stdout.txt" &
    runs+=("${out}/${maze}")
done

wait

python3 "${tools}/check_run.py" --expect-state IDLE --expect unbound_ports=0 --expect watchdog_expiries=0 \
    --expect emergency_stops=0 "${runs[@]}"
