# run fromm project root `bash docker/scripts/cube.sh`
Xvfb :10 -ac > /dev/null &
export DISPLAY=:10
$CUBE_PATH/jre/bin/java -jar $CUBE_PATH/STM32CubeMX -q /MicrasFirmware/docker/scripts/.cube
pkill -f Xvfb
