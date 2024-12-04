/**
 * @file
 */

#ifndef MICRAS_NAV_MAPPING_CPP
#define MICRAS_NAV_MAPPING_CPP

#include <cmath>

#include "micras/nav/mapping.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
Mapping<width, height>::Mapping(const proxy::WallSensors<4>& wall_sensors, Mapping::Config config) :
    wall_sensors{wall_sensors},
    maze{config.start, config.goal},
    wall_thickness{config.wall_thickness},
    cell_size{config.cell_size},
    alignment_threshold{config.alignment_threshold},
    front_sensor_pose{config.front_sensor_pose},
    side_sensor_pose{config.side_sensor_pose},
    front_sensors_region_division{cell_size - wall_thickness / 2.0F - front_sensor_pose.position.y},
    side_sensors_region_division{
        cell_size + wall_thickness / 2 - side_sensor_pose.position.y +
        (side_sensor_pose.position.x + (wall_thickness - cell_size) / 2.0F) * std::tan(side_sensor_pose.orientation)
    },
    front_distance_alignment_tolerance{config.front_distance_alignment_tolerance},
    side_distance_alignment_tolerance{config.side_distance_alignment_tolerance},
    front_orientation_alignment_tolerance{config.front_orientation_alignment_tolerance},
    side_orientation_alignment_tolerance{config.side_orientation_alignment_tolerance},
    front_distance_reading{config.front_distance_reading},
    front_orientation_reading{config.front_orientation_reading},
    side_distance_reading{config.side_distance_reading} { }

template <uint8_t width, uint8_t height>
void Mapping<width, height>::update(const Pose& pose) {
    float reliability = 50.0F * (std::cos(4 * pose.orientation) + 1);

    if (reliability < 60.0F) {
        return;
    }

    Information information{};
    nav::Point  cell_position = pose.to_cell(cell_size);

    if (cell_position.y < this->front_sensors_region_division) {
        if (this->wall_sensors.get_observation(Sensor::FRONT_LEFT) == core::Observation::WALL and
            this->wall_sensors.get_observation(Sensor::FRONT_RIGHT) == core::Observation::WALL) {
            information.front = core::Observation::WALL;
        } else if (this->wall_sensors.get_observation(Sensor::FRONT_LEFT) == core::Observation::FREE_SPACE and
                   this->wall_sensors.get_observation(Sensor::FRONT_RIGHT) == core::Observation::FREE_SPACE) {
            information.front = core::Observation::FREE_SPACE;
        }
    }

    if (std::abs(cell_position.x) < this->alignment_threshold * this->cell_size and
        information.front != core::Observation::WALL and cell_position.y > this->side_sensors_region_division) {
        information.front_left = this->wall_sensors.get_observation(Sensor::LEFT);
        information.front_right = this->wall_sensors.get_observation(Sensor::RIGHT);
    }

    this->maze.update(pose.to_grid(this->cell_size), information);
}

template <uint8_t width, uint8_t height>
Mapping<width, height>::Action Mapping<width, height>::get_action(const Pose& pose) const {
    GridPose current_grid_goal = this->maze.get_current_goal(pose.position.to_grid(this->cell_size));
    Point    current_goal = Point::from_grid(current_grid_goal.position, this->cell_size);
    Point    current_goal_rotated = current_goal.rotate(current_grid_goal.orientation);

    if (std::abs(core::assert_angle(pose.orientation - pose.position.angle_between(current_goal))) >
        std::numbers::pi_v<float> / 8.0F) {
        return {Action::Type::LOOK_AT, current_goal_rotated, current_grid_goal.orientation};
    }

    return {Action::Type::GO_TO, current_goal_rotated, current_grid_goal.orientation};
}

template <uint8_t width, uint8_t height>
Pose Mapping<width, height>::correct_pose(const Pose& pose, core::FollowWallType follow_wall_type) const {
    float reliability = 50.0F * (std::cos(4 * pose.orientation) + 1);

    if (reliability < 50.0F) {
        return pose;
    }

    Point cell_position = pose.to_cell(this->cell_size);
    Point pose_correction{};
    Pose  corrected_pose = pose;
    Side  direction = angle_to_grid(pose.orientation);

    switch (follow_wall_type) {
        case core::FollowWallType::FRONT:
            if (this->is_distance_front_aligned()) {
                pose_correction.y = cell_position.y - this->cell_size / 2.0F;
            }

            if (this->is_orientation_front_aligned()) {
                corrected_pose.orientation =
                    core::assert_angle(static_cast<uint8_t>(direction) * std::numbers::pi_v<float> / 2.0F);
            }
            break;
        case core::FollowWallType::PARALLEL:
            if (this->is_distance_side_aligned()) {
                pose_correction.x = cell_position.x;
                corrected_pose.orientation =
                    core::assert_angle(static_cast<uint8_t>(direction) * std::numbers::pi_v<float> / 2.0F);
            } else if (this->is_orientation_side_aligned()) {
                corrected_pose.orientation =
                    core::assert_angle(static_cast<uint8_t>(direction) * std::numbers::pi_v<float> / 2.0F);
            }
            break;
        default:
            return pose;
    }

    pose_correction = pose_correction.rotate(static_cast<Side>((6 - direction) % 4));
    corrected_pose.position = corrected_pose.position - pose_correction;

    return corrected_pose;
}

template <uint8_t width, uint8_t height>
void Mapping<width, height>::calibrate_front() {
    this->front_distance_reading[0] = this->wall_sensors.get_reading(0);
    front_orientation_reading[0] = this->wall_sensors.get_reading(1);
    front_orientation_reading[1] = this->wall_sensors.get_reading(2);
    this->front_distance_reading[1] = this->wall_sensors.get_reading(3);
}

template <uint8_t width, uint8_t height>
void Mapping<width, height>::calibrate_side() {
    this->side_distance_reading[0] = this->wall_sensors.get_reading(1);
    this->side_distance_reading[1] = this->wall_sensors.get_reading(2);
}

template <uint8_t width, uint8_t height>
bool Mapping<width, height>::is_distance_front_aligned() const {
    return core::is_near(
               this->wall_sensors.get_reading(0), this->front_distance_reading[0],
               this->front_distance_alignment_tolerance
           ) and
           core::is_near(
               this->wall_sensors.get_reading(3), this->front_distance_reading[1],
               this->front_distance_alignment_tolerance
           );
}

template <uint8_t width, uint8_t height>
bool Mapping<width, height>::is_orientation_front_aligned() const {
    return std::abs(
               (this->wall_sensors.get_reading(1) - this->front_orientation_reading[0]) +
               (this->wall_sensors.get_reading(2) - this->front_orientation_reading[1])
           ) <= this->front_orientation_alignment_tolerance;
}

template <uint8_t width, uint8_t height>
bool Mapping<width, height>::is_distance_side_aligned() const {
    return core::is_near(
               this->wall_sensors.get_reading(1), this->side_distance_reading[0],
               this->side_distance_alignment_tolerance
           ) and
           core::is_near(
               this->wall_sensors.get_reading(2), this->side_distance_reading[1],
               this->side_distance_alignment_tolerance
           );
}

template <uint8_t width, uint8_t height>
bool Mapping<width, height>::is_orientation_side_aligned() const {
    return std::abs(
               (this->wall_sensors.get_reading(1) - this->side_distance_reading[0]) +
               (this->wall_sensors.get_reading(2) - this->side_distance_reading[1])
           ) <= this->side_orientation_alignment_tolerance;
}

template <uint8_t width, uint8_t height>
core::FollowWallType Mapping<width, height>::get_follow_wall_type(const Pose& pose) const {
    nav::Point cell_position = pose.to_cell(cell_size);

    if (cell_position.y < this->side_sensors_region_division - this->wall_thickness / 2.0F) {
        return this->maze.get_follow_wall_type(pose.to_grid(this->cell_size), false);
    }

    return this->maze.get_follow_wall_type(pose.to_grid(this->cell_size), true);
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAPPING_CPP
