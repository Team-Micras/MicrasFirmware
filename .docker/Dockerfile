# syntax=docker/dockerfile:1

ARG CUBEMX_IMAGE=thunderatz/stm32cubemx:6.18.1-h7

ARG TOOLCHAIN_PACKAGES="\
    clang-format \
    clang-tidy \
    cmake \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    make"

##########################################
# Cube files generation with STM32CubeMX #
##########################################
FROM ${CUBEMX_IMAGE} AS generator

ARG PROJECT_NAME=micras_v1

WORKDIR /project
COPY cube/*.ioc cube/

# Without network, CubeMX skips its online update check, which can hang the project generation
RUN --network=none <<EOT
    set -e
    printf 'config load /project/cube/%s.ioc\nproject generate\nexit\n' "${PROJECT_NAME}" > /tmp/cube_script
    xvfb-run --auto-servernum "${CUBE_PATH}/STM32CubeMX" -q /tmp/cube_script
    rm /tmp/cube_script
    test -f cube/cmake/stm32cubemx/CMakeLists.txt || { echo "STM32CubeMX failed to generate the project" >&2; exit 1; }
EOT

# Only the generated files, can be replaced by already generated ones with --build-context generated=<dir>
FROM scratch AS generated

COPY --from=generator /project/cube /

###################################
# Build image for Micras Firmware #
###################################
FROM ubuntu:24.04 AS build

ARG TOOLCHAIN_PACKAGES

# The ARM libraries come prebuilt for every ARM core, only the Cortex-M4F and Cortex-M7 double precision ones are kept
RUN apt-get update && \
    apt-get install -y --no-install-recommends ${TOOLCHAIN_PACKAGES} && \
    rm -rf /var/lib/apt/lists/* && \
    find /usr/lib/arm-none-eabi/lib/thumb /usr/lib/gcc/arm-none-eabi/*/thumb -mindepth 1 -maxdepth 1 \
        -not -name "v7e-m+fp" -not -name "v7e-m+dp" -exec rm -rf {} + && \
    rm -rf /usr/lib/arm-none-eabi/lib/arm /usr/lib/gcc/arm-none-eabi/*/arm

WORKDIR /project

COPY . .
COPY --from=generated / cube/

RUN cmake -B build -DBUILD_TYPE=Release

###############################################
# Development image with toolchain and CubeMX #
###############################################
FROM ${CUBEMX_IMAGE} AS dev

ARG TOOLCHAIN_PACKAGES

RUN apt-get update && \
    apt-get install -y --no-install-recommends ${TOOLCHAIN_PACKAGES} && \
    rm -rf /var/lib/apt/lists/*

ENV CUBE_CMD=${CUBE_PATH}/STM32CubeMX

WORKDIR /project

RUN echo "trap 'chown -R ubuntu /project' EXIT" >> /root/.bashrc
