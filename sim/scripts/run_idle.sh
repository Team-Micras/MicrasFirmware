#!/bin/bash
# run_idle.sh <simulator> <scenarios dir> <runs dir>
#
# The robot left alone for 4 s -> <runs dir>/idle. The run starts from the runs directory with a copy of
# its scenario, so the paths meta.json and a baseline summary record are the same on every machine.

set -euo pipefail

if [[ $# -ne 3 ]]; then
    echo "usage: $0 <simulator> <scenarios dir> <runs dir>" >&2
    exit 2
fi

mkdir -p "$3"
cp "$2/idle.toml" "$3/idle.toml"
cd "$3"
"$1" --scenario idle.toml --out idle
