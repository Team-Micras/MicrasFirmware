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

COPY . /MicrasFirmware
WORKDIR /MicrasFirmware

RUN echo "trap 'chown -R ubuntu /MicrasFirmware' EXIT" >> "/root/.bashrc"

###################################
# Build image for Micras Firmware #
###################################
FROM micras AS build

RUN Xvfb :10 -ac >/dev/null 2>&1 & \
    export DISPLAY=:10 && \
    mkdir /MicrasFirmware/build && \
    cd /MicrasFirmware/build && \
    cmake .. -DBUILD_TYPE=Release && \
    pkill -f Xvfb

RUN git submodule update --init --recursive
