/**
 * @file
 */

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

    serial_data.push_back(static_cast<uint8_t>(pose.position.x));
    serial_data.push_back(static_cast<uint8_t>(pose.position.y));

    serial_data.push_back(static_cast<uint8_t>(pose.orientation));

    serial_data.push_back(static_cast<uint8_t>(velocity.linear));
    serial_data.push_back(static_cast<uint8_t>(velocity.angular));

    return serial_data;
}

void State::deserialize(const uint8_t* serial_data, uint16_t size) {
    if (size != 6) {
        return;
    }

    pose.position.x = static_cast<float>(serial_data[0]);
    pose.position.y = static_cast<float>(serial_data[1]);

    pose.orientation = static_cast<float>(serial_data[2]);

    velocity.linear = static_cast<float>(serial_data[3]);
    velocity.angular = static_cast<float>(serial_data[4]);
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
