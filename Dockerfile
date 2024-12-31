##################################
# Base image for Micras Firmware #
##################################
FROM ubuntu:24.04 AS base

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y \
    gcc-arm-none-eabi \
    build-essential \
    cmake \
    unzip \
    git \
    xvfb \
    curl \
    openjdk-21-jre \
    libgbm1 \
    && apt-get clean

RUN mkdir st && cd st && \
    curl -O https://sw-center.st.com/packs/resource/library/stm32cube_mx_v6130-lin.zip && \
    unzip stm32cube_mx_v6130-lin.zip && \
    unzip JavaJre.zip && \
    mv MX /root/STM32CubeMX && \
    mv jre /root/STM32CubeMX && \
    cd .. && rm -rf st

ENV CUBE_PATH="/root/STM32CubeMX"

RUN echo "exit" > /root/cube-init && \
    Xvfb :10 -ac > /dev/null & \
    export DISPLAY=:10 && \
    $CUBE_PATH/STM32CubeMX -q /root/cube-init && \
    rm /root/cube-init && \
    pkill -f Xvfb && \
    rm /tmp/.X10-lock

WORKDIR /root

RUN mkdir STM32Cube && cd STM32Cube && mkdir Repository && cd Repository && \
    git clone -b v1.6.1 https://github.com/STMicroelectronics/STM32CubeG4.git && cd STM32CubeG4 && \
    git submodule update --init --recursive

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
