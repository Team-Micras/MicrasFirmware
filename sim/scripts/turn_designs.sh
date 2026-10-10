#!/bin/bash
# turn_designs.sh <turn designer> <header>
#
# Designs the turns of two bends into the header the build generates, replacing it only once the
# designer succeeded, so a failed search leaves the header as it was.

set -euo pipefail

if [[ $# -ne 2 ]]; then
    echo "usage: $0 <turn designer> <header>" >&2
    exit 2
fi

"$1" > "$2.new"
mv "$2.new" "$2"
