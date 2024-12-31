##################################
# Base image for Micras Firmware #
##################################
FROM thunderatz/stm32cubemx:6.13.0-g4 AS micras

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y \
    gcc-arm-none-eabi \
    make \
    cmake \
    clang-tidy \
    clang-format \
    && apt-get clean

COPY . /MicrasFirmware
WORKDIR /MicrasFirmware

RUN echo "trap 'chown -R ubuntu /MicrasFirmware' EXIT" >> "/root/.bashrc"

###################################
# Build image for Micras Firmware #
###################################
FROM micras AS build

RUN Xvfb :10 -ac > /dev/null & \
    export DISPLAY=:10 && \
    mkdir /MicrasFirmware/build && \
    cd /MicrasFirmware/build && \
    cmake .. -DBUILD_TYPE=Release && \
    pkill -f Xvfb

RUN git submodule update --init --recursive
