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
Mapping<width, height>::Mapping(const proxy::DistanceSensors<4>& distance_sensors, Mapping::Config config) :
    distance_sensors{distance_sensors},
    maze{config.start},
    wall_thickness{config.wall_thickness},
    cell_size{config.cell_size},
    wall_distance_threshold{config.wall_distance_threshold},
    free_distance_threshold{config.free_distance_threshold},
    alignment_threshold{config.alignment_threshold},
    front_sensor_pose{config.front_sensor_pose},
    side_sensor_pose{config.side_sensor_pose},
    side_sensors_region_division{
        cell_size - wall_thickness / 2.0F - side_sensor_pose.position.y +
        (side_sensor_pose.position.x + (wall_thickness - cell_size) / 2.0F) * std::tan(side_sensor_pose.orientation)
    },
    front_sensors_region_division{cell_size - wall_thickness / 2.0F - front_sensor_pose.position.y} { }

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
        if (this->get_wall_information(Sensor::FRONT_LEFT) == Information::WALL and
            this->get_wall_information(Sensor::FRONT_RIGHT) == Information::WALL) {
            information.front = Information::WALL;
        } else if (this->get_wall_information(Sensor::FRONT_LEFT) == Information::FREE and
                   this->get_wall_information(Sensor::FRONT_RIGHT) == Information::FREE) {
            information.front = Information::FREE;
        }
    }

    if (cell_alignment < this->alignment_threshold * this->cell_size) {
        if (cell_advance < this->side_sensors_region_division) {
            information.left = this->get_wall_information(Sensor::LEFT);
            information.right = this->get_wall_information(Sensor::RIGHT);
        } else if (information.front == Information::FREE and
                   cell_advance > this->side_sensors_region_division + this->wall_thickness) {
            information.front_left = this->get_wall_information(Sensor::LEFT);
            information.front_right = this->get_wall_information(Sensor::RIGHT);
        }
    }

    this->maze.update({pose.position.to_grid(this->cell_size), side}, information);
}

template <uint8_t width, uint8_t height>
Mapping<width, height>::Action Mapping<width, height>::get_action(const Pose& pose) const {
    Point current_goal =
        Point::from_grid(this->maze.get_current_goal(pose.position.to_grid(this->cell_size)), this->cell_size);

    float angle_error = core::assert_angle(pose.orientation - pose.position.angle_between(current_goal));

    if (std::abs(angle_error) > std::numbers::pi_v<float> / 4.0F) {
        return {Action::Type::LOOK_AT, current_goal};
    }

    return {Action::Type::GO_TO, current_goal};
}

template <uint8_t width, uint8_t height>
Information::Existence Mapping<width, height>::get_wall_information(Sensor sensor) const {
    if (this->distance_sensors.get_distance(sensor) < this->wall_distance_threshold) {
        return Information::WALL;
    }

    if (this->distance_sensors.get_distance(sensor) < this->free_distance_threshold) {
        return Information::FREE;
    }

    return Information::UNKNOWN;
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAPPING_CPP
