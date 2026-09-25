/**
 * @file
 */

#ifndef MICRAS_NAV_EXPLORER_TPP
#define MICRAS_NAV_EXPLORER_TPP

#include <algorithm>
#include <cstdint>
#include <span>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/planner.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
TExplorer<width, height>::TExplorer(TPlanner<width, height>& planner, std::span<const RunProfile> profiles) :
    planner{planner}, profiles{profiles} {
    this->route.steps.reserve(static_cast<std::size_t>(width) * height);
}

template <uint8_t width, uint8_t height>
void TExplorer<width, height>::reset() {
    this->number_of_targets = 0;
    this->number_collected = 0;
    this->profile_index = 0;
    this->planning = false;
    this->answered = false;
}

template <uint8_t width, uint8_t height>
bool TExplorer<width, height>::update(const TMaze<width, height>& maze, uint32_t max_edges) {
    if (this->profiles.empty()) {
        this->answered = true;
        return false;
    }

    if (this->planning and maze.get_revision() != this->round_revision) {
        this->planning = false;
        this->profile_index = 0;
    }

    if (not this->planning) {
        if (this->answered and maze.get_revision() == this->answered_revision) {
            return false;
        }

        if (this->profile_index == 0) {
            this->number_collected = 0;
            this->round_revision = maze.get_revision();
        }

        this->planner.begin(maze, WallAssumption::OPTIMISTIC, this->profiles[this->profile_index]);
        this->planning = true;
    }

    if (not this->planner.step(max_edges)) {
        return false;
    }

    this->planning = false;

    float best_time = 0.0F;

    for (uint8_t i = 0; i < this->planner.get_number_of_routes(); i++) {
        this->planner.get_route(i, this->route);

        if (i == 0) {
            best_time = this->route.time;
        } else if (this->route.time > best_time + candidate_window) {
            break;
        }

        TPlanner<width, height>::for_each_wall(this->route, [this, &maze](const GridPose& wall) {
            if (maze.get_wall(wall) == WallState::UNKNOWN) {
                this->add_target(wall);
            }
        });
    }

    this->profile_index++;

    if (this->profile_index < this->profiles.size()) {
        return false;
    }

    this->profile_index = 0;

    const bool changed =
        not this->answered or this->number_collected != this->number_of_targets or
        not std::equal(
            this->collected.begin(), this->collected.begin() + this->number_collected, this->targets.begin()
        );

    this->targets = this->collected;
    this->number_of_targets = this->number_collected;
    this->answered = true;
    this->answered_revision = this->round_revision;

    return changed;
}

template <uint8_t width, uint8_t height>
bool TExplorer<width, height>::has_targets() const {
    return this->answered;
}

template <uint8_t width, uint8_t height>
bool TExplorer<width, height>::is_complete() const {
    return this->answered and this->number_of_targets == 0;
}

template <uint8_t width, uint8_t height>
std::span<const GridPoint> TExplorer<width, height>::get_targets() const {
    return std::span{this->targets}.first(this->number_of_targets);
}

template <uint8_t width, uint8_t height>
void TExplorer<width, height>::add_target(const GridPose& wall) {
    for (const GridPoint& cell : {wall.position, wall.front().position}) {
        const auto end = this->collected.begin() + this->number_collected;

        if (not TMaze<width, height>::contains(cell) or this->number_collected == max_targets or
            std::find(this->collected.begin(), end, cell) != end) {
            continue;
        }

        this->collected.at(this->number_collected) = cell;
        this->number_collected++;
    }
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_EXPLORER_TPP
