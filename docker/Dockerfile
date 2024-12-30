FROM ubuntu:24.04 AS base

RUN apt update -y
RUN apt upgrade -y

RUN apt install -y \
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

WORKDIR /root

RUN mkdir STM32Cube && cd STM32Cube && mkdir Repository && cd Repository && \
    git clone -b v1.6.1 https://github.com/STMicroelectronics/STM32CubeG4.git && cd STM32CubeG4 && \
    git submodule update --init --recursive

RUN mkdir st && cd st && \
    curl -O https://sw-center.st.com/packs/resource/library/stm32cube_mx_v6130-lin.zip && \
    unzip stm32cube_mx_v6130-lin.zip && \
    mv MX /root/STM32CubeMX && \
    cd .. && rm -rf st

RUN mkdir /MicrasFirmware

WORKDIR /MicrasFirmware
