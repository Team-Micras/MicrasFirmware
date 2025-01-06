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
    front_sensor_pose{config.front_sensor_pose},
    side_sensor_pose{config.side_sensor_pose},
    front_sensors_region_division{cell_size - wall_thickness / 2.0F - front_sensor_pose.position.y},
    side_sensors_region_division{
        cell_size - side_sensor_pose.position.y +
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

    if (reliability < 30.0F) {
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

    if (information.front != core::Observation::WALL and
        cell_position.y > this->side_sensors_region_division + 2 * this->wall_thickness) {
        information.front_left = this->wall_sensors.get_observation(Sensor::LEFT);
        information.front_right = this->wall_sensors.get_observation(Sensor::RIGHT);
    }

    this->maze.update(pose.to_grid(this->cell_size), information);
}

template <uint8_t width, uint8_t height>
Mapping<width, height>::Action Mapping<width, height>::get_action(const Pose& pose, core::Objective objective) {
    GridPoint grid_position = pose.position.to_grid(this->cell_size);
    GridPose  current_grid_goal{};

    Point     current_goal{};
    Direction orientation{};

    switch (objective) {
        case core::Objective::EXPLORE:
            if (this->maze.finished(grid_position)) {
                return {Action::Type::FINISHED, pose.position, Direction::EAST};
            }

            current_grid_goal = this->maze.get_current_exploration_goal(grid_position);
            current_goal = Point::from_grid(current_grid_goal.position, this->cell_size);
            orientation = static_cast<Direction>(2 * current_grid_goal.orientation);
            break;
        case core::Objective::RETURN:
            if (this->maze.returned(grid_position)) {
                this->maze.optimize_route();
                return {Action::Type::FINISHED, pose.position, Direction::EAST};
            }

            this->maze.calculate_best_route();
            current_grid_goal = this->maze.get_current_returning_goal(grid_position);
            current_goal = Point::from_grid(current_grid_goal.position, this->cell_size);
            orientation = static_cast<Direction>(2 * current_grid_goal.orientation);
            break;
        case core::Objective::SOLVE:
            if (this->maze.finished(grid_position)) {
                return {Action::Type::FINISHED, pose.position, Direction::EAST};
            }

            this->best_route_iterator++;
            current_goal = this->best_route_iterator->first;
            orientation = this->best_route_iterator->second;
            break;
    }

    Point current_goal_rotated = current_goal.rotate(orientation);

    if (std::abs(core::assert_angle(pose.orientation - pose.position.angle_between(current_goal))) >
        0.375F * std::numbers::pi_v<float>) {
        if (objective == core::Objective::SOLVE) {
            this->best_route_iterator--;
        }

        return {Action::Type::LOOK_AT, current_goal_rotated, orientation};
    }

    return {Action::Type::GO_TO, current_goal_rotated, orientation};
}

template <uint8_t width, uint8_t height>
Pose Mapping<width, height>::correct_pose(const Pose& pose, core::FollowWallType follow_wall_type) const {
    float reliability = 50.0F * (std::cos(4 * pose.orientation) + 1);

    if (reliability < 20.0F) {
        return pose;
    }

    Point cell_position = pose.to_cell(this->cell_size);
    Point pose_correction{};
    Pose  corrected_pose = pose;
    Side  direction = angle_to_grid(pose.orientation);

    switch (follow_wall_type) {
        case core::FollowWallType::BACK:
            pose_correction.y = cell_position.y - (0.04F + this->wall_thickness / 2);
            corrected_pose.orientation =
                core::assert_angle(static_cast<uint8_t>(direction) * std::numbers::pi_v<float> / 2.0F);
            break;

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
                pose_correction.y = cell_position.y - std::hypot(cell_position.x, cell_position.y);

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

    pose_correction = pose_correction.rotate(static_cast<Direction>((12 - 2 * direction) % 8));
    corrected_pose.position = corrected_pose.position - pose_correction;

    return corrected_pose;
}

template <uint8_t width, uint8_t height>
void Mapping<width, height>::calibrate_front() {
    this->front_distance_reading[0] = this->wall_sensors.get_reading(0);
    this->front_orientation_reading[0] = this->wall_sensors.get_reading(1);
    this->front_orientation_reading[1] = this->wall_sensors.get_reading(2);
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

    core::FollowWallType follow_wall_type = this->maze.get_follow_wall_type(pose.to_grid(this->cell_size));

    if (follow_wall_type == core::FollowWallType::FRONT and cell_position.y > this->side_sensors_region_division) {
        return follow_wall_type;
    }

    if (cell_position.y < this->side_sensors_region_division and follow_wall_type != core::FollowWallType::FRONT) {
        return follow_wall_type;
    }

    bool can_follow_left = this->wall_sensors.get_observation(Sensor::LEFT) == core::Observation::WALL;
    bool can_follow_right = this->wall_sensors.get_observation(Sensor::RIGHT) == core::Observation::WALL;

    if (can_follow_left and can_follow_right) {
        return core::FollowWallType::PARALLEL;
    }

    if (can_follow_left) {
        return core::FollowWallType::LEFT;
    }

    if (can_follow_right) {
        return core::FollowWallType::RIGHT;
    }

    return core::FollowWallType::NONE;
}

template <uint8_t width, uint8_t height>
std::vector<uint8_t> Mapping<width, height>::serialize() const {
    std::vector<uint8_t> buffer;
    buffer.reserve(3 * this->maze.get_best_route().size());

    for (const auto& [cost, grid_pose] : this->maze.get_best_route()) {
        buffer.emplace_back(grid_pose.position.x);
        buffer.emplace_back(grid_pose.position.y);
        buffer.emplace_back(grid_pose.orientation);
    }

    return buffer;
}

template <uint8_t width, uint8_t height>
void Mapping<width, height>::deserialize(const uint8_t* buffer, uint16_t size) {
    this->best_route.clear();

    for (uint32_t i = 0; i < size; i += 3) {
        this->best_route.emplace_back(
            Point::from_grid({buffer[i], buffer[i + 1]}, this->cell_size), static_cast<Direction>(2 * buffer[i + 2])
        );
    }
}

template <uint8_t width, uint8_t height>
bool Mapping<width, height>::can_align_back(const Pose& pose) const {
    return this->maze.has_wall(pose.to_grid(this->cell_size).turned_back());
}

template <uint8_t width, uint8_t height>
void Mapping<width, height>::diagonalize_best_route() {
    for (auto it = std::next(this->best_route.begin()); it != this->best_route.end(); it++) {
        auto next_it = std::next(it);

        if (next_it == this->best_route.end()) {
            break;
        }

        auto diagonal_direction = static_cast<Direction>((next_it->second + it->second) / 2);

        if (std::abs(next_it->second - it->second) > 2) {
            diagonal_direction = Direction::SOUTHEAST;
        }

        this->best_route.insert(it, {it->first.move_towards(std::prev(it)->first, this->cell_size / 2), it->second});
        this->best_route.insert(it, {it->first.move_towards(next_it->first, this->cell_size / 2), diagonal_direction});
    }

    uint8_t pos = 1;

    for (auto it = std::next(this->best_route.begin()); it != this->best_route.end(); pos++) {
        if (pos % 3 == 0) {
            it = this->best_route.erase(it);
        } else {
            it++;
        }
    }

    for (auto it = std::next(this->best_route.begin()); it != this->best_route.end();) {
        auto next_it = std::next(it);

        if (next_it == this->best_route.end()) {
            break;
        }

        if (it->second == next_it->second) {
            it = this->best_route.erase(it);
        } else if (it->first == next_it->first) {
            this->best_route.erase(next_it);
        } else {
            it++;
        }
    }
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAPPING_CPP
