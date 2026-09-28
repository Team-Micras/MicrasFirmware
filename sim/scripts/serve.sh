#!/bin/bash
# serve.sh <simulator> <scenarios dir> <runs dir>
#
# An exploration started over the link, with the bridge open so micras-monitor can connect on
# ws://localhost:8080 -> <runs dir>/serve

set -euo pipefail

if [[ $# -ne 3 ]]; then
    echo "usage: $0 <simulator> <scenarios dir> <runs dir>" >&2
    exit 2
fi

mkdir -p "$3"
"$1" --scenario "$2/explore_link.toml" --out "$3/serve" --monitor
