Xvfb :10 -ac > /dev/null &
export DISPLAY=:10
java -jar $CUBE_PATH/STM32CubeMX -q /MicrasFirmware/docker/scripts/.cube
pkill -f Xvfb
