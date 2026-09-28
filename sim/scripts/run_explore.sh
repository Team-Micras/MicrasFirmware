#!/bin/bash
# run_explore.sh <simulator> <scenarios dir> <runs dir> <seconds>
#
# The first <seconds> of an exploration started by the button, keeping the flash it saves: the search
# to the goal, where the firmware saves the map, and the start of the way back -> <runs dir>/explore.
# The run starts from the runs directory with a copy of its scenario, so the paths meta.json and a
# baseline summary record are the same on every machine.

set -euo pipefail

if [[ $# -ne 4 ]]; then
    echo "usage: $0 <simulator> <scenarios dir> <runs dir> <seconds>" >&2
    exit 2
fi

mkdir -p "$3/explore"
rm -f "$3/explore/flash.bin"
cp "$2/explore.toml" "$3/explore.toml"
cd "$3"
"$1" --scenario explore.toml --seconds "$4" --out explore --flash explore/flash.bin
