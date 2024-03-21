/**
 * @file imu.cpp
 *
 * @brief Proxy Imu class source
 *
 * @date 03/2024
 */

#include <lsm6dsv_reg.h>

#include "proxy/imu.hpp"

namespace proxy {
Imu::Imu(const Config& imu_config) : spi{imu_config.spi} {
}
}  // namespace proxy
