# syntax=docker/dockerfile:1.27

ARG CUBEMX_IMAGE=thunderatz/stm32cubemx:6.18.1-h7

# The clang tooling is pinned to a major version so that a distribution update cannot move the
# formatting rules or the enabled check set under the project
ARG CLANG_VERSION=22

ARG TOOLCHAIN_PACKAGES="\
    ca-certificates \
    cmake \
    doxygen \
    gcc-arm-none-eabi \
    graphviz \
    git \
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
FROM ubuntu:26.04 AS build

ARG TOOLCHAIN_PACKAGES
ARG CLANG_VERSION

# The apt caches are kept in BuildKit cache mounts instead of the image layers, so the hook
# that deletes the downloaded packages after every install has to go
RUN rm -f /etc/apt/apt.conf.d/docker-clean

# The ARM libraries come prebuilt for every ARM core, only the Cortex-M4F and Cortex-M7 double precision ones are kept
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends ${TOOLCHAIN_PACKAGES} \
        "clang-format-${CLANG_VERSION}" "clang-tidy-${CLANG_VERSION}" && \
    for tool in clang-format clang-tidy run-clang-tidy; do \
        ln -s "/usr/bin/${tool}-${CLANG_VERSION}" "/usr/local/bin/${tool}"; \
    done && \
    find /usr/lib/arm-none-eabi/lib/thumb /usr/lib/gcc/arm-none-eabi/*/thumb -mindepth 1 -maxdepth 1 \
        -not -name "v7e-m+fp" -not -name "v7e-m+dp" -exec rm -rf {} + && \
    rm -rf /usr/lib/arm-none-eabi/lib/arm /usr/lib/gcc/arm-none-eabi/*/arm

WORKDIR /project

COPY . .
COPY --from=generated / cube/

RUN cmake -B build -DBUILD_TYPE=Release -DMICRAS_WERROR=ON

###############################################
# Development image with toolchain and CubeMX #
###############################################
FROM ${CUBEMX_IMAGE} AS dev

ARG TOOLCHAIN_PACKAGES
ARG CLANG_VERSION

RUN rm -f /etc/apt/apt.conf.d/docker-clean

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends ${TOOLCHAIN_PACKAGES} \
        "clang-format-${CLANG_VERSION}" "clang-tidy-${CLANG_VERSION}" && \
    for tool in clang-format clang-tidy run-clang-tidy; do \
        ln -s "/usr/bin/${tool}-${CLANG_VERSION}" "/usr/local/bin/${tool}"; \
    done

ENV CUBE_CMD=${CUBE_PATH}/STM32CubeMX

WORKDIR /project

RUN echo "trap 'chown -R ubuntu /project' EXIT" >> /root/.bashrc
