#!/bin/bash
# watch.sh <simulator> <scenarios dir> <runs dir>
#
# An exploration in a window. Space pauses, right steps, tab cycles cameras, esc quits -> <runs dir>/watch

set -euo pipefail

if [[ $# -ne 3 ]]; then
    echo "usage: $0 <simulator> <scenarios dir> <runs dir>" >&2
    exit 2
fi

mkdir -p "$3"
"$1" --scenario "$2/explore.toml" --out "$3/watch" --viewer
