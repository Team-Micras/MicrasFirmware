##################################
# Base image for Micras Firmware #
##################################
FROM thunderatz/stm32cubemx:6.13.0-g4 AS micras

RUN apt-get update -y > /dev/null && \
    apt-get upgrade -y > /dev/null && \
    apt-get install -y \
    gcc-arm-none-eabi \
    make \
    cmake \
    clang-tidy \
    clang-format  > /dev/null\
    && apt-get clean > /dev/null

COPY ./cube/micras.ioc /MicrasFirmware/cube/micras.ioc
WORKDIR /MicrasFirmware

ENV DISPLAY=:10

RUN Xvfb :10 -ac & \
    echo "config load /MicrasFirmware/cube/micras.ioc\nproject generate\nexit\n" > .cube && \
    $CUBE_PATH/STM32CubeMX -q /MicrasFirmware/.cube && \
    rm .cube && \
    pkill -f Xvfb

RUN rm /tmp/.X10-lock

RUN echo "trap 'chown -R ubuntu /MicrasFirmware' EXIT" >> "/root/.bashrc"

###################################
# Build image for Micras Firmware #
###################################
FROM micras AS build

COPY . /MicrasFirmware

WORKDIR /MicrasFirmware
