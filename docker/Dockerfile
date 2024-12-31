FROM ubuntu:24.04 AS base

RUN apt update -y && \
    apt upgrade -y && \
    apt install -y \
    gcc-arm-none-eabi \
    build-essential \
    cmake \
    unzip \
    git \
    xvfb \
    curl \
    openjdk-21-jre \
    libgbm1 \
    && apt clean

RUN mkdir st && cd st && \
    curl -O https://sw-center.st.com/packs/resource/library/stm32cube_mx_v6130-lin.zip && \
    unzip stm32cube_mx_v6130-lin.zip && \
    unzip JavaJre.zip && \
    mv MX /root/STM32CubeMX && \
    mv jre /root/STM32CubeMX && \
    cd .. && rm -rf st

WORKDIR /root

RUN mkdir STM32Cube && cd STM32Cube && mkdir Repository && cd Repository && \
    git clone -b v1.6.1 https://github.com/STMicroelectronics/STM32CubeG4.git && cd STM32CubeG4 && \
    git submodule update --init --recursive

ENV CUBE_PATH="/root/STM32CubeMX"

RUN echo "exit" > /root/cube-init && \
    Xvfb :10 -ac > /dev/null & \
    export DISPLAY=:10 && \
    $CUBE_PATH/STM32CubeMX -q /root/cube-init && \
    pkill -f Xvfb

RUN echo "trap 'chown -R ubuntu /MicrasFirmware' EXIT" >> "/root/.bashrc"

FROM base AS build

WORKDIR /MicrasFirmware
COPY . /MicrasFirmware

RUN source /MicrasFirmware/docker/scripts/build.sh

FROM build AS format

RUN apt install -y clang-format

RUN source /MicrasFirmware/docker/scripts/format.sh

FROM build AS lint

RUN apt install -y clang-tidy

RUN source /MicrasFirmware/docker/scripts/build.sh ON

FROM base AS dev

WORKDIR /MicrasFirmware
