/**
 * @file mapping.cpp
 *
 * @brief Nav Mapping class implementation
 *
 * @date 10/2024
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
        cell_size - wall_thickness / 2.0F - side_sensor_pose.position.y +
        (side_sensor_pose.position.x + (wall_thickness - cell_size) / 2.0F) * std::tan(side_sensor_pose.orientation)
    },
    front_alignment_tolerance{config.front_alignment_tolerance},
    side_alignment_tolerance{config.side_alignment_tolerance},
    can_follow_wall_tolerance{config.can_follow_wall_tolerance},
    front_alignment_measure{config.front_alignment_measure},
    side_alignment_measure{config.side_alignment_measure} { }

template <uint8_t width, uint8_t height>
void Mapping<width, height>::update(const Pose& pose) {
    float reliability = 50.0F * (std::cos(4 * pose.orientation) + 1);

    if (reliability < 50.0F) {
        return;
    }

    Point cell_position = pose.position % this->cell_size;
    Side  side = angle_to_grid(pose.orientation);

    float cell_advance = 0.0F;
    float cell_alignment = 0.0F;

    switch (side) {
        case Side::RIGHT:
            cell_advance = cell_position.x;
            cell_alignment = std::abs(this->cell_size / 2.0F - cell_position.y);
            break;
        case Side::UP:
            cell_advance = cell_position.y;
            cell_alignment = std::abs(this->cell_size / 2.0F - cell_position.x);
            break;
        case Side::LEFT:
            cell_advance = this->cell_size - cell_position.x;
            cell_alignment = std::abs(this->cell_size / 2.0F - cell_position.y);
            break;
        case Side::DOWN:
            cell_advance = this->cell_size - cell_position.y;
            cell_alignment = std::abs(this->cell_size / 2.0F - cell_position.x);
            break;
    }

    Information information{};

    if (cell_advance < this->front_sensors_region_division) {
        if (this->wall_sensors.get_observation(Sensor::FRONT_LEFT) == core::Observation::WALL and
            this->wall_sensors.get_observation(Sensor::FRONT_RIGHT) == core::Observation::WALL) {
            information.front = core::Observation::WALL;
        } else if (this->wall_sensors.get_observation(Sensor::FRONT_LEFT) == core::Observation::FREE_SPACE and
                   this->wall_sensors.get_observation(Sensor::FRONT_RIGHT) == core::Observation::FREE_SPACE) {
            information.front = core::Observation::FREE_SPACE;
        }
    }

    if (cell_alignment < this->alignment_threshold * this->cell_size) {
        if (cell_advance < this->side_sensors_region_division) {
            information.left = this->wall_sensors.get_observation(Sensor::LEFT);
            information.right = this->wall_sensors.get_observation(Sensor::RIGHT);
        } else if (information.front != core::Observation::WALL and
                   cell_advance > this->side_sensors_region_division + this->wall_thickness) {
            information.front_left = this->wall_sensors.get_observation(Sensor::LEFT);
            information.front_right = this->wall_sensors.get_observation(Sensor::RIGHT);
        }
    }

    this->maze.update({pose.position.to_grid(this->cell_size), side}, information);
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
Pose Mapping<width, height>::correct_pose(const Pose& pose, bool can_follow_wall) const {
    float reliability = 50.0F * (std::cos(4 * pose.orientation) + 1);

    if (reliability < 50.0F) {
        return pose;
    }

    Pose corrected_pose = pose;
    Side side = angle_to_grid(pose.orientation);

    switch (side) {
        case Side::RIGHT:
            if (this->is_front_aligned()) {
                corrected_pose.position.x -= std::fmod(pose.position.x, this->cell_size) - this->cell_size / 2.0F;
            } else if (this->is_side_aligned()) {
                corrected_pose.orientation = 0.0F;
                corrected_pose.position.y -= std::fmod(pose.position.y, this->cell_size) - this->cell_size / 2.0F;
            }
            break;
        case Side::UP:
            if (this->is_front_aligned()) {
                corrected_pose.position.y -= std::fmod(pose.position.y, this->cell_size) - this->cell_size / 2.0F;
            } else if (this->is_side_aligned()) {
                corrected_pose.orientation = std::numbers::pi_v<float> / 2.0F;
                corrected_pose.position.x -= std::fmod(pose.position.x, this->cell_size) - this->cell_size / 2.0F;
            }
            break;
        case Side::LEFT:
            if (this->is_front_aligned()) {
                corrected_pose.position.x -= std::fmod(pose.position.x, this->cell_size) - this->cell_size / 2.0F;
            } else if (this->is_side_aligned()) {
                corrected_pose.orientation = std::numbers::pi_v<float>;
                corrected_pose.position.y -= std::fmod(pose.position.y, this->cell_size) - this->cell_size / 2.0F;
            }
            break;
        case Side::DOWN:
            if (this->is_front_aligned()) {
                corrected_pose.position.y -= std::fmod(pose.position.y, this->cell_size) - this->cell_size / 2.0F;
            } else if (this->is_side_aligned()) {
                corrected_pose.orientation = -std::numbers::pi_v<float> / 2.0F;
                corrected_pose.position.x -= std::fmod(pose.position.x, this->cell_size) - this->cell_size / 2.0F;
            }
            break;
    }

    return corrected_pose;
}

template <uint8_t width, uint8_t height>
void Mapping<width, height>::calibrate_front() {
    this->front_alignment_measure[0] = this->wall_sensors.get_reading(0);
    this->front_alignment_measure[1] = this->wall_sensors.get_reading(3);
}

template <uint8_t width, uint8_t height>
void Mapping<width, height>::calibrate_side() {
    this->side_alignment_measure[0] = this->wall_sensors.get_reading(1);
    this->side_alignment_measure[1] = this->wall_sensors.get_reading(2);
}

template <uint8_t width, uint8_t height>
bool Mapping<width, height>::is_front_aligned() const {
    return core::is_near(
               this->wall_sensors.get_reading(0), this->front_alignment_measure[0], this->front_alignment_tolerance
           ) and
           core::is_near(
               this->wall_sensors.get_reading(3), this->front_alignment_measure[1], this->front_alignment_tolerance
           );
}

template <uint8_t width, uint8_t height>
bool Mapping<width, height>::is_side_aligned() const {
    return core::is_near(
               this->wall_sensors.get_reading(1), this->side_alignment_measure[0], this->side_alignment_tolerance
           ) and
           core::is_near(
               this->wall_sensors.get_reading(2), this->side_alignment_measure[1], this->side_alignment_tolerance
           );
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAPPING_CPP
