/**
 * @file
 */

#include <cmath>

#include "micras/core/utils.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
GridPose Pose::to_grid(float cell_size) const {
    return {
        .position = GridPoint::from_vector(this->position, cell_size), .orientation = angle_to_grid(this->orientation)
    };
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
}  // namespace micras::nav
