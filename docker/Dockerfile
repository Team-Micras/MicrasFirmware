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
    libnss3 \
    libgbm1 \
    libssl3t64 \
    && apt clean

WORKDIR /usr/share

RUN git clone https://github.com/STMicroelectronics/STM32CubeG4 && cd STM32CubeG4 && \
    git submodule update --init --recursive

WORKDIR /

RUN mkdir st && cd st && \
    curl -O https://sw-center.st.com/packs/resource/library/stm32cube_mx_v6110-lin.zip && \
    unzip stm32cube_mx_v6110-lin.zip && \
    unzip JavaJre.zip && \
    mv MX /opt/stm32cubemx && \
    mv jre /opt/stm32cubemx && \
    cd .. && rm -rf st

RUN mkdir MicrasFirmware

WORKDIR /MicrasFirmware
