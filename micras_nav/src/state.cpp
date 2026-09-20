/**
 * @file
 */

#include <cmath>

#include "micras/core/utils.hpp"
#include "micras/core/vector.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
GridPose Pose::to_grid(float cell_size) const {
    return {
        .position = GridPoint::from_vector(this->position, cell_size), .orientation = angle_to_grid(this->orientation)
    };
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

    return {.x = cell_alignment, .y = cell_advance};
}

Pose Pose::compose(const Pose& local) const {
    const float cosine = std::cos(this->orientation);
    const float sine = std::sin(this->orientation);

    return {
        .position =
            {.x = this->position.x + cosine * local.position.x - sine * local.position.y,
             .y = this->position.y + sine * local.position.x + cosine * local.position.y},
        .orientation = this->orientation + local.orientation,
    };
}

Pose Pose::relative(const Pose& other) const {
    const float cosine = std::cos(this->orientation);
    const float sine = std::sin(this->orientation);

    const core::Vector offset = other.position - this->position;

    return {
        .position = {.x = cosine * offset.x + sine * offset.y, .y = -sine * offset.x + cosine * offset.y},
        .orientation = core::assert_angle(other.orientation - this->orientation),
    };
}

RelativePose::RelativePose(const Pose& absolute_pose) : absolute_pose{&absolute_pose} { }

Pose RelativePose::get() const {
    return {
        .position = this->absolute_pose->position - this->reference_pose.position,
        .orientation = this->absolute_pose->orientation - this->reference_pose.orientation
    };
}

void RelativePose::reset_reference() {
    this->reference_pose = *(this->absolute_pose);
}
}  // namespace micras::nav
