#!/bin/bash
# check_flash.sh <simulator> <scenarios dir> <runs dir> <tools dir>
#
# The checked exploration saves its map when it reaches the goal, which leaves a programmed image in its
# flash file (run_explore.sh), and a new run boots from that image to IDLE.

set -euo pipefail

if [[ $# -ne 4 ]]; then
    echo "usage: $0 <simulator> <scenarios dir> <runs dir> <tools dir>" >&2
    exit 2
fi

simulator=$1
scenarios=$2
runs=$3
tools=$4

python3 -c 'import sys; image = open(sys.argv[1], "rb").read(); sys.exit(0 if any(byte != 0xFF for byte in image) else "the flash image is blank")' \
    "${runs}/explore/flash.bin"
cp "${runs}/explore/flash.bin" "${runs}/booted.bin"
"${simulator}" --scenario "${scenarios}/idle.toml" --out "${runs}/booted" --flash "${runs}/booted.bin" > /dev/null
python3 "${tools}/check_run.py" --expect-state IDLE "${runs}/booted"
