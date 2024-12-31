##################################
# Base image for Micras Firmware #
##################################
FROM thunderatz/stm32cubemx:6.13.0-g4 AS base

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y \
    gcc-arm-none-eabi \
    make \
    cmake \
    && apt-get clean

COPY . /MicrasFirmware
WORKDIR /MicrasFirmware

RUN echo "trap 'chown -R ubuntu /MicrasFirmware' EXIT" >> "/root/.bashrc"

###################################
# Build image for Micras Firmware #
###################################
FROM base AS build

RUN Xvfb :10 -ac > /dev/null & \
    export DISPLAY=:10 && \
    mkdir /MicrasFirmware/build && \
    cd /MicrasFirmware/build && \
    cmake .. -DBUILD_TYPE=Release && \
    pkill -f Xvfb

RUN git submodule update --init --recursive

##################################
# Lint image for Micras Firmware #
##################################
FROM build AS lint

RUN apt-get install -y clang-tidy

####################################
# Format image for Micras Firmware #
####################################
FROM base AS format

RUN apt-get install -y clang-format
