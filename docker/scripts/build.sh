#!/bin/bash

LINT=$1

pkill -f Xvfb
rm /tmp/.X10-lock
Xvfb :10 -ac > /dev/null & 
XVFB_PID=$!
export DISPLAY=:10

trap "kill $XVFB_PID" EXIT

cd /MicrasFirmware
git submodule update --init --recursive || exit 1
mkdir -p /MicrasFirmware/build || exit 1
cd /MicrasFirmware/build || exit 1
cmake .. -DLINTER_MODE=$LINT || exit 1

echo "Compiling main..." | sed 's/.*/\x1b[34m&\x1b[0m/'

make -j

if [ $? -eq 0 ]; then
  echo "Compilation successful." | sed 's/.*/\x1b[32m&\x1b[0m/'
else
  echo "Compilation failed." | sed 's/.*/\x1b[31m&\x1b[0m/'
  exit 1
fi

echo "Compiling tests..." | sed 's/.*/\x1b[34m&\x1b[0m/'

make test_all -j

if [ $? -eq 0 ]; then
  echo "Tests compiled successfully." | sed 's/.*/\x1b[32m&\x1b[0m/'
else
  echo "Tests compilation failed." | sed 's/.*/\x1b[31m&\x1b[0m/'
  exit 1
fi
