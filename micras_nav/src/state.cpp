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
