##################################
# Base image for Micras Firmware #
##################################
FROM thunderatz/stm32cubemx:6.13.0-g4 AS base

RUN apt-get update -y > /dev/null && \
    apt-get upgrade -y > /dev/null && \
    apt-get install -y \
    gcc-arm-none-eabi \
    make \
    cmake \
    clang-tidy \
    clang-format  > /dev/null\
    && apt-get clean > /dev/null

WORKDIR /project
COPY ./cube/*.ioc /project/cube/

ENV CUBE_CMD=${CUBE_PATH}/STM32CubeMX

ENV DISPLAY=:10
ARG PROJECT_NAME=micras

RUN Xvfb :10 -ac & \
    echo "config load /project/cube/${PROJECT_NAME}.ioc\nproject generate\nexit\n" > .cube && \
    $CUBE_CMD -q /project/.cube && \
    rm .cube && \
    pkill -f Xvfb

RUN rm /tmp/.X10-lock

RUN echo "trap 'chown -R ubuntu /project' EXIT" >> "/root/.bashrc"

###################################
# Build image for Micras Firmware #
###################################
FROM base AS build

COPY . /project

RUN mkdir -p build && \
    cmake -B build -DCMAKE_BUILD_TYPE=Release
