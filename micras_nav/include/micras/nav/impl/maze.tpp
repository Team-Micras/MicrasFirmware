/**
 * @file
 */

#ifndef MICRAS_NAV_MAZE_TPP
#define MICRAS_NAV_MAZE_TPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "micras/nav/grid_pose.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
TMaze<width, height>::TMaze(const Config& config) : start{config.start}, goal{config.goal} {
    this->reset();
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::reset() {
    for (auto& row : this->walls) {
        for (auto& cell : row) {
            cell.fill(WallState::UNKNOWN);
        }
    }

    for (uint8_t row = 0; row < height; row++) {
        this->write_wall({.position = {.x = 0, .y = row}, .orientation = Side::LEFT}, WallState::WALL);
        this->write_wall(
            {.position = {.x = static_cast<uint8_t>(width - 1), .y = row}, .orientation = Side::RIGHT}, WallState::WALL
        );
    }

    for (uint8_t column = 0; column < width; column++) {
        this->write_wall({.position = {.x = column, .y = 0}, .orientation = Side::DOWN}, WallState::WALL);
        this->write_wall(
            {.position = {.x = column, .y = static_cast<uint8_t>(height - 1)}, .orientation = Side::UP}, WallState::WALL
        );
    }

    for (const Side side : all_sides) {
        const GridPose wall{.position = this->start.position, .orientation = side};
        this->write_wall(wall, side == this->start.orientation ? WallState::NO_WALL : WallState::WALL);
    }

    for (const GridPoint& cell : this->goal) {
        for (const Side side : all_sides) {
            if (this->is_goal(cell + side)) {
                this->write_wall({.position = cell, .orientation = side}, WallState::NO_WALL);
            }
        }
    }

    this->revision = 0;
    this->flood(this->goal);
}

template <uint8_t width, uint8_t height>
WallState TMaze<width, height>::get_wall(const GridPose& pose) const {
    if (not contains(pose.position)) {
        return WallState::WALL;
    }

    return this->walls.at(pose.position.y).at(pose.position.x).at(std::to_underlying(pose.orientation));
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::set_wall(const GridPose& pose, bool present) {
    if (this->get_wall(pose) != WallState::UNKNOWN) {
        return false;
    }

    this->write_wall(pose, present ? WallState::WALL : WallState::NO_WALL);
    this->revision++;

    return true;
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::is_blocked(const GridPose& pose) const {
    return this->get_wall(pose) == WallState::WALL;
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::is_possibly_open(const GridPose& pose) const {
    return this->get_wall(pose) != WallState::WALL;
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::is_known_open(const GridPose& pose) const {
    return this->get_wall(pose) == WallState::NO_WALL;
}

template <uint8_t width, uint8_t height>
bool TMaze<width, height>::is_goal(const GridPoint& position) const {
    for (const GridPoint& cell : this->goal) {
        if (cell == position) {
            return true;
        }
    }

    return false;
}

template <uint8_t width, uint8_t height>
const GridPose& TMaze<width, height>::get_start() const {
    return this->start;
}

template <uint8_t width, uint8_t height>
std::span<const GridPoint> TMaze<width, height>::get_goal() const {
    return this->goal;
}

template <uint8_t width, uint8_t height>
uint32_t TMaze<width, height>::get_revision() const {
    return this->revision;
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::flood(std::span<const GridPoint> targets) {
    for (auto& row : this->costs) {
        row.fill(unreachable);
    }

    std::size_t head = 0;
    std::size_t tail = 0;

    for (const GridPoint& target : targets) {
        if (contains(target) and this->costs.at(target.y).at(target.x) != 0) {
            this->costs.at(target.y).at(target.x) = 0;
            this->queue.at(tail++) = target;
        }
    }

    while (head < tail) {
        const GridPoint cell = this->queue.at(head++);
        const uint16_t  next_cost = this->costs.at(cell.y).at(cell.x) + 1;

        for (const Side side : all_sides) {
            if (this->is_blocked({.position = cell, .orientation = side})) {
                continue;
            }

            const GridPoint neighbor = cell + side;

            if (this->costs.at(neighbor.y).at(neighbor.x) == unreachable) {
                this->costs.at(neighbor.y).at(neighbor.x) = next_cost;
                this->queue.at(tail++) = neighbor;
            }
        }
    }
}

template <uint8_t width, uint8_t height>
uint16_t TMaze<width, height>::get_cost(const GridPoint& position) const {
    return contains(position) ? this->costs.at(position.y).at(position.x) : unreachable;
}

template <uint8_t width, uint8_t height>
std::optional<GridPose> TMaze<width, height>::get_next(const GridPose& pose) const {
    const std::array<Side, 4> preference{
        pose.orientation,
        pose.turned_left().orientation,
        pose.turned_right().orientation,
        pose.turned_back().orientation,
    };

    std::optional<GridPose> best{};
    uint16_t                best_cost = unreachable;

    for (const Side side : preference) {
        if (this->is_blocked({.position = pose.position, .orientation = side})) {
            continue;
        }

        const GridPoint neighbor = pose.position + side;
        const uint16_t  cost = this->get_cost(neighbor);

        if (cost < best_cost) {
            best_cost = cost;
            best = GridPose{.position = neighbor, .orientation = side};
        }
    }

    return best;
}

template <uint8_t width, uint8_t height>
std::vector<uint8_t> TMaze<width, height>::serialize() const {
    std::vector<uint8_t> buffer(header_size + walls_size, 0);

    buffer.at(0) = format_version;
    buffer.at(1) = width;
    buffer.at(2) = height;

    for (uint8_t row = 0; row < height; row++) {
        for (uint8_t column = 0; column < width; column++) {
            const auto&    cell = this->walls.at(row).at(column);
            const uint16_t index = row * width + column;

            const auto packed = static_cast<uint8_t>(
                std::to_underlying(cell.at(std::to_underlying(Side::RIGHT))) |
                (std::to_underlying(cell.at(std::to_underlying(Side::UP))) << 2U)
            );

            buffer.at(header_size + index / 2U) |= static_cast<uint8_t>(packed << (4U * (index % 2U)));
        }
    }

    return buffer;
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::deserialize(const uint8_t* buffer, uint16_t size) {
    const std::span<const uint8_t> data{buffer, size};

    if (size != header_size + walls_size or data[0] != format_version or data[1] != width or data[2] != height) {
        return;
    }

    this->reset();

    for (uint8_t row = 0; row < height; row++) {
        for (uint8_t column = 0; column < width; column++) {
            const uint16_t index = row * width + column;
            const uint8_t  packed = (data[header_size + index / 2U] >> (4U * (index % 2U))) & 0x0FU;

            const std::array<std::pair<Side, uint8_t>, 2> sides{
                {{Side::RIGHT, packed & 0x03U}, {Side::UP, packed >> 2U}}
            };

            for (const auto& [side, state] : sides) {
                if (state == std::to_underlying(WallState::WALL) or state == std::to_underlying(WallState::NO_WALL)) {
                    this->set_wall(
                        {.position = {.x = column, .y = row}, .orientation = side},
                        state == std::to_underlying(WallState::WALL)
                    );
                }
            }
        }
    }

    this->flood(this->goal);
}

template <uint8_t width, uint8_t height>
void TMaze<width, height>::write_wall(const GridPose& pose, WallState state) {
    this->walls.at(pose.position.y).at(pose.position.x).at(std::to_underlying(pose.orientation)) = state;

    const GridPose opposite = pose.front().turned_back();

    if (contains(opposite.position)) {
        this->walls.at(opposite.position.y).at(opposite.position.x).at(std::to_underlying(opposite.orientation)) =
            state;
    }
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_MAZE_TPP
