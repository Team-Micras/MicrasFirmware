# syntax=docker/dockerfile:1

##################################
# Base image for Micras Firmware #
##################################
FROM thunderatz/stm32cubemx:6.18.1-h7 AS base

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        clang-format \
        clang-tidy \
        cmake \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib \
        make && \
    rm -rf /var/lib/apt/lists/*

ENV CUBE_CMD=${CUBE_PATH}/STM32CubeMX

WORKDIR /project
COPY cube/*.ioc cube/

ARG PROJECT_NAME=micras_v1

# Without network, CubeMX skips its online update check, which can hang the project generation
RUN --network=none <<EOT
    set -e
    printf 'config load /project/cube/%s.ioc\nproject generate\nexit\n' "${PROJECT_NAME}" > /tmp/cube_script
    xvfb-run --auto-servernum "${CUBE_CMD}" -q /tmp/cube_script
    rm /tmp/cube_script
    test -f cube/cmake/stm32cubemx/CMakeLists.txt || { echo "STM32CubeMX failed to generate the project" >&2; exit 1; }
EOT

RUN echo "trap 'chown -R ubuntu /project' EXIT" >> /root/.bashrc

###################################
# Build image for Micras Firmware #
###################################
FROM base AS build

COPY . .

RUN cmake -B build -DBUILD_TYPE=Release
