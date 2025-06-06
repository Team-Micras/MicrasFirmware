/**
 * @file
 */

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>

#include "micras/nav/state.hpp"

namespace micras::nav {
GridPose Pose::to_grid(float cell_size) const {
    return {GridPoint::from_vector(this->position, cell_size), angle_to_grid(this->orientation)};
};

core::Vector Pose::to_cell(float cell_size) const {
    const core::Vector cell_position = this->position % cell_size;

    float cell_advance = 0.0F;
    float cell_alignment = 0.0F;

    switch (angle_to_grid(this->orientation)) {
        case Side::RIGHT:
            cell_advance = cell_position.x;
            cell_alignment = cell_size / 2.0F - cell_position.y;
            break;
        case Side::UP:
            cell_advance = cell_position.y;
            cell_alignment = cell_position.x - cell_size / 2.0F;
            break;
        case Side::LEFT:
            cell_advance = cell_size - cell_position.x;
            cell_alignment = cell_position.y - cell_size / 2.0F;
            break;
        case Side::DOWN:
            cell_advance = cell_size - cell_position.y;
            cell_alignment = cell_size / 2.0F - cell_position.x;
            break;
    }

    return {cell_alignment, cell_advance};
}

std::vector<uint8_t> State::serialize() const {
    std::vector<uint8_t> serial_data;
    serial_data.reserve(20);

    auto x_bytes = std::bit_cast<std::array<uint8_t, 4>>(pose.position.x);
    serial_data.insert(serial_data.end(), x_bytes.begin(), x_bytes.end());

    auto y_bytes = std::bit_cast<std::array<uint8_t, 4>>(pose.position.y);
    serial_data.insert(serial_data.end(), y_bytes.begin(), y_bytes.end());

    auto orientation_bytes = std::bit_cast<std::array<uint8_t, 4>>(pose.orientation);
    serial_data.insert(serial_data.end(), orientation_bytes.begin(), orientation_bytes.end());

    auto linear_bytes = std::bit_cast<std::array<uint8_t, 4>>(velocity.linear);
    serial_data.insert(serial_data.end(), linear_bytes.begin(), linear_bytes.end());

    auto angular_bytes = std::bit_cast<std::array<uint8_t, 4>>(velocity.angular);
    serial_data.insert(serial_data.end(), angular_bytes.begin(), angular_bytes.end());

    return serial_data;
}

void State::deserialize(const uint8_t* serial_data, uint16_t size) {
    if (size != 20) {
        return;
    }
    std::array<uint8_t, 4> x_bytes;
    std::copy(serial_data, serial_data + 4, x_bytes.begin());
    pose.position.x = std::bit_cast<float>(x_bytes);

    std::array<uint8_t, 4> y_bytes;
    std::copy(serial_data + 4, serial_data + 8, y_bytes.begin());
    pose.position.y = std::bit_cast<float>(y_bytes);

    std::array<uint8_t, 4> orientation_bytes;
    std::copy(serial_data + 8, serial_data + 12, orientation_bytes.begin());
    pose.orientation = std::bit_cast<float>(orientation_bytes);

    std::array<uint8_t, 4> linear_bytes;
    std::copy(serial_data + 12, serial_data + 16, linear_bytes.begin());
    velocity.linear = std::bit_cast<float>(linear_bytes);

    std::array<uint8_t, 4> angular_bytes;
    std::copy(serial_data + 16, serial_data + 20, angular_bytes.begin());
    velocity.angular = std::bit_cast<float>(angular_bytes);
}

RelativePose::RelativePose(const Pose& absolute_pose) : absolute_pose{&absolute_pose} { }

Pose RelativePose::get() const {
    return {
        this->absolute_pose->position - this->reference_pose.position,
        this->absolute_pose->orientation - this->reference_pose.orientation
    };
}

void RelativePose::reset_reference() {
    this->reference_pose = *(this->absolute_pose);
}
}  // namespace micras::nav
