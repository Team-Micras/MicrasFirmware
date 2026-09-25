/**
 * @file
 */

#ifndef MICRAS_NAV_RACING_LINE_TPP
#define MICRAS_NAV_RACING_LINE_TPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/core/math.hpp"
#include "micras/core/vector.hpp"
#include "micras/nav/curve_speed.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/line.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/planner.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/state.hpp"
#include "micras/nav/turn_table.hpp"

namespace micras::nav {
template <uint8_t width, uint8_t height>
TRacingLine<width, height>::TRacingLine(const Dynamics& dynamics, const Config& config) :
    dynamics{dynamics}, config{config} {
    const float front = dynamics.get_model().chassis.front_length;
    const float rear = dynamics.get_model().chassis.rear_length;
    const float half = dynamics.get_model().chassis.half_width;

    const std::array<core::Vector, 4> corners{{
        {.x = front, .y = half},
        {.x = -rear, .y = half},
        {.x = -rear, .y = -half},
        {.x = front, .y = -half},
    }};

    for (uint8_t side = 0; side < corners.size(); side++) {
        const core::Vector& from = corners.at(side);
        const core::Vector& to = corners.at((side + 1) % corners.size());
        const auto          steps = static_cast<uint8_t>(std::ceil(from.distance(to) / outline_spacing));

        for (uint8_t step = 0; step < steps and this->outline_size < max_outline_points; step++) {
            const float fraction = static_cast<float>(step) / static_cast<float>(steps);

            this->outline.at(this->outline_size++) = {
                .x = from.x + fraction * (to.x - from.x),
                .y = from.y + fraction * (to.y - from.y),
            };
        }
    }

    for (uint8_t i = 0; i < this->outline_size; i++) {
        this->reach = std::max(this->reach, this->outline.at(i).magnitude());
    }
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::reset() {
    this->line.ready = false;
    this->phase = Phase::IDLE;
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::begin(
    const Route& route, std::span<const Segment> segments, const TMaze<width, height>& maze, const RunProfile& profile
) {
    this->reset();
    this->maze = &maze;
    this->segments = segments;
    this->run_profile = profile;
    this->least_margin = profile.risky ? this->config.risky_least_margin : this->config.least_margin;
    this->openings.fill(0);
    this->sweeps = 0;

    TPlanner<width, height>::for_each_wall(route, [this](const GridPose& wall) { this->open(wall); });

    for (const GridPoint& cell : maze.get_goal()) {
        for (const Side side : all_sides) {
            const GridPoint neighbor = cell + side;

            if (TMaze<width, height>::contains(neighbor) and maze.is_goal(neighbor) and
                maze.get_wall({.position = cell, .orientation = side}) == WallState::NO_WALL) {
                this->open({.position = cell, .orientation = side});
            }
        }
    }

    float total = 0.0F;

    for (const Segment& segment : segments) {
        total += std::abs(segment.length);
    }

    const auto samples = static_cast<int32_t>(std::lround(total / this->config.spacing)) + 1;

    if (segments.empty() or samples > Line::max_samples or samples < 2 * fixed_samples + 2) {
        this->finish(false);
        return;
    }

    this->line.size = static_cast<uint16_t>(samples);
    this->line.spacing = total / static_cast<float>(samples - 1);
    this->cursor = 0;
    this->segment_index = 0;
    this->segment_start = 0.0F;
    this->phase = Phase::SAMPLE;
}

template <uint8_t width, uint8_t height>
bool TRacingLine<width, height>::step() {
    uint32_t budget = this->config.samples_per_step;

    while (budget > 0) {
        switch (this->phase) {
            case Phase::IDLE:
                return true;

            case Phase::SAMPLE:
                this->sample_route(budget);
                break;

            case Phase::BOUNDS:
                this->find_bounds(budget);
                break;

            case Phase::SOLVE:
                this->solve_window();
                return false;

            case Phase::MEASURE:
                this->measure(budget);
                break;

            case Phase::RESAMPLE:
                this->resample(budget);
                break;

            case Phase::CHECK:
                this->check(budget);
                break;

            case Phase::CURVATURE:
                this->measure_curvature(budget);
                break;

            case Phase::SMOOTH:
                this->smooth_curvature(budget);
                break;

            case Phase::SPEED:
                this->plan_speeds();
                return this->phase == Phase::IDLE;
        }
    }

    return this->phase == Phase::IDLE;
}

template <uint8_t width, uint8_t height>
const Line& TRacingLine<width, height>::get_line() const {
    return this->line;
}

template <uint8_t width, uint8_t height>
uint8_t TRacingLine<width, height>::get_sweeps() const {
    return this->sweeps;
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::sample_route(uint32_t& budget) {
    for (; budget > 0 and this->cursor < this->line.size; budget--, this->cursor++) {
        const float distance = static_cast<float>(this->cursor) * this->line.spacing;

        while (this->segment_index + 1U < this->segments.size() and
               distance > this->segment_start + std::abs(this->segments[this->segment_index].length)) {
            this->segment_start += std::abs(this->segments[this->segment_index].length);
            this->segment_index++;
        }

        const Segment& segment = this->segments[this->segment_index];
        const float    side = std::copysign(1.0F, segment.length);
        const float    along = std::clamp(distance - this->segment_start, 0.0F, std::abs(segment.length));

        Pose local{.position = {.x = side * along, .y = 0.0F}, .orientation = 0.0F};

        if (segment.kind == SegmentKind::TURN) {
            const auto point = this->dynamics.get_turn(this->run_profile, segment.turn).template sample<float>(along);
            local = {.position = {.x = point.x, .y = side * point.y}, .orientation = side * point.heading};
        }

        const Pose pose = segment.start.compose(local);

        this->line.xs.at(this->cursor) = pose.position.x;
        this->line.ys.at(this->cursor) = pose.position.y;
    }

    if (this->cursor == this->line.size) {
        this->start_sweep();
    }
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::find_bounds(uint32_t& budget) {
    for (; budget > 0 and this->cursor < this->window_size; budget--, this->cursor++) {
        const auto  index = static_cast<uint16_t>(this->window_start + this->cursor);
        const float heading = this->get_heading(index);
        const float cosine = std::cos(heading);
        const float sine = std::sin(heading);

        const core::Vector position{.x = this->line.xs.at(index), .y = this->line.ys.at(index)};

        this->headings.at(this->cursor) = heading;
        this->lower.at(this->cursor) = 0.0F;
        this->upper.at(this->cursor) = 0.0F;
        this->old_x.at(this->cursor) = position.x;
        this->old_y.at(this->cursor) = position.y;
        this->was_clear.at(this->cursor + 1U) = this->is_clear(position, cosine, sine, this->config.margin);

        if (not this->was_clear.at(this->cursor + 1U)) {
            continue;
        }

        const auto steps = static_cast<uint8_t>(
            std::min<float>(std::floor(this->config.trust / this->config.scan_step), max_scan_steps)
        );

        for (const float direction : {1.0F, -1.0F}) {
            float reached = 0.0F;

            for (uint8_t step = 1; step <= steps; step++) {
                const float        offset = direction * static_cast<float>(step) * this->config.scan_step;
                const core::Vector moved{.x = position.x - offset * sine, .y = position.y + offset * cosine};

                if (not this->is_clear(moved, cosine, sine, this->config.margin)) {
                    break;
                }

                reached = offset;
            }

            if (direction > 0.0F) {
                this->upper.at(this->cursor) = reached;
            } else {
                this->lower.at(this->cursor) = reached;
            }
        }
    }

    if (this->cursor == this->window_size) {
        this->phase = Phase::SOLVE;
    }
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::solve_window() {
    const uint16_t first = this->window_start;
    const uint8_t  size = this->window_size;

    std::array<double, window> normals_x{};
    std::array<double, window> normals_y{};

    for (uint8_t j = 0; j < size; j++) {
        normals_x.at(j) = -std::sin(static_cast<double>(this->headings.at(j)));
        normals_y.at(j) = std::cos(static_cast<double>(this->headings.at(j)));
    }

    this->solve_slides(std::span{normals_x}.first(size), std::span{normals_y}.first(size));

    const auto neighbor_clear = [this](uint16_t index) {
        const float heading = this->get_heading(index);

        return this->is_clear(
            {.x = this->line.xs.at(index), .y = this->line.ys.at(index)}, std::cos(heading), std::sin(heading),
            this->config.margin
        );
    };

    this->was_clear.at(0) = neighbor_clear(first - 1U);
    this->was_clear.at(size + 1U) = neighbor_clear(first + size);

    const auto place = [&]() {
        for (uint8_t j = 0; j < size; j++) {
            this->line.xs.at(first + j) = this->old_x.at(j) + static_cast<float>(this->slides.at(j) * normals_x.at(j));
            this->line.ys.at(first + j) = this->old_y.at(j) + static_cast<float>(this->slides.at(j) * normals_y.at(j));
        }
    };

    place();

    for (uint8_t shrink = 0;; shrink++) {
        std::array<bool, window + 2> clashes{};
        bool                         clash = false;

        for (uint8_t j = 0; j < size + 2U; j++) {
            clashes.at(j) = this->was_clear.at(j) and not neighbor_clear(first + j - 1U);
            clash = clash or clashes.at(j);
        }

        if (not clash) {
            break;
        }

        if (shrink == max_shrinks) {
            this->slides.fill(0.0);
            place();
            break;
        }

        for (uint8_t j = 0; j < size + 2U; j++) {
            if (not clashes.at(j)) {
                continue;
            }

            for (int32_t k = std::max<int32_t>(j - 4, 0); k <= std::min<int32_t>(j + 2, size - 1); k++) {
                this->slides.at(k) *= 0.5;
            }
        }

        place();
    }

    for (uint8_t j = 0; j < size; j++) {
        this->sweep_slide = std::max(this->sweep_slide, static_cast<float>(std::abs(this->slides.at(j))));
    }

    this->window_start += stride;

    if (this->window_start + fixed_samples >= this->line.size) {
        this->cursor = 0;
        this->measured_length = 0.0F;
        this->phase = Phase::MEASURE;
        return;
    }

    this->start_window();
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::measure(uint32_t& budget) {
    for (; budget > 0 and this->cursor + 1U < this->line.size; budget--, this->cursor++) {
        this->measured_length += std::hypot(
            this->line.xs.at(this->cursor + 1U) - this->line.xs.at(this->cursor),
            this->line.ys.at(this->cursor + 1U) - this->line.ys.at(this->cursor)
        );
    }

    if (this->cursor + 1U < this->line.size) {
        return;
    }

    const auto samples = static_cast<int32_t>(std::lround(this->measured_length / this->config.spacing)) + 1;

    if (samples > Line::max_samples or samples < 2 * fixed_samples + 2) {
        this->finish(false);
        return;
    }

    this->resampled_size = static_cast<uint16_t>(samples);
    this->cursor = 0;
    this->read_index = 0;
    this->read_length = 0.0F;
    this->phase = Phase::RESAMPLE;
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::resample(uint32_t& budget) {
    const float spacing = this->measured_length / static_cast<float>(this->resampled_size - 1);

    for (; budget > 0 and this->cursor < this->resampled_size; budget--, this->cursor++) {
        if (this->cursor + 1U == this->resampled_size) {
            this->line.curvatures.at(this->cursor) = this->line.xs.at(this->line.size - 1);
            this->line.speeds.at(this->cursor) = this->line.ys.at(this->line.size - 1);
            continue;
        }

        const float distance = static_cast<float>(this->cursor) * spacing;

        float step_length = 0.0F;

        while (true) {
            step_length = std::hypot(
                this->line.xs.at(this->read_index + 1U) - this->line.xs.at(this->read_index),
                this->line.ys.at(this->read_index + 1U) - this->line.ys.at(this->read_index)
            );

            if (this->read_length + step_length >= distance or this->read_index + 2U >= this->line.size) {
                break;
            }

            this->read_length += step_length;
            this->read_index++;
        }

        const float fraction =
            step_length > 0.0F ? std::clamp((distance - this->read_length) / step_length, 0.0F, 1.0F) : 0.0F;
        const uint16_t from = this->read_index;

        this->line.curvatures.at(this->cursor) =
            this->line.xs.at(from) + fraction * (this->line.xs.at(from + 1U) - this->line.xs.at(from));
        this->line.speeds.at(this->cursor) =
            this->line.ys.at(from) + fraction * (this->line.ys.at(from + 1U) - this->line.ys.at(from));
    }

    if (this->cursor < this->resampled_size) {
        return;
    }

    std::copy_n(this->line.curvatures.begin(), this->resampled_size, this->line.xs.begin());
    std::copy_n(this->line.speeds.begin(), this->resampled_size, this->line.ys.begin());
    this->line.size = this->resampled_size;
    this->line.spacing = spacing;
    this->sweeps++;

    if (this->sweep_slide < this->config.convergence or this->sweeps >= this->config.max_sweeps) {
        this->cursor = fixed_samples;
        this->phase = Phase::CHECK;
        return;
    }

    this->start_sweep();
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::check(uint32_t& budget) {
    const uint16_t end = this->line.size - fixed_samples;

    for (; budget > 0 and this->cursor < end; budget--, this->cursor++) {
        const float heading = this->get_heading(this->cursor);

        if (not this->is_clear(
                {.x = this->line.xs.at(this->cursor), .y = this->line.ys.at(this->cursor)}, std::cos(heading),
                std::sin(heading), this->least_margin
            )) {
            this->finish(false);
            return;
        }
    }

    if (this->cursor == end) {
        this->cursor = 0;
        this->phase = Phase::CURVATURE;
    }
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::measure_curvature(uint32_t& budget) {
    const uint16_t last = this->line.size - 1;

    for (; budget > 0 and this->cursor <= last; budget--, this->cursor++) {
        if (this->cursor == last) {
            this->line.times.at(this->cursor) = 0.0F;
            continue;
        }

        const float heading = std::atan2(
            this->line.ys.at(this->cursor + 1U) - this->line.ys.at(this->cursor),
            this->line.xs.at(this->cursor + 1U) - this->line.xs.at(this->cursor)
        );

        this->line.times.at(this->cursor) =
            this->cursor == 0 ? 0.0F : core::math::wrap_angle(heading - this->previous_heading) / this->line.spacing;
        this->previous_heading = heading;
    }

    if (this->cursor > last) {
        this->cursor = 0;
        this->phase = Phase::SMOOTH;
    }
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::smooth_curvature(uint32_t& budget) {
    const int32_t last = this->line.size - 1;
    const int32_t reach = this->config.smoothing;

    for (; budget > 0 and this->cursor < this->line.size; budget--, this->cursor++) {
        const int32_t from = std::max<int32_t>(this->cursor - reach, 0);
        const int32_t to = std::min<int32_t>(this->cursor + reach, last);

        float sum = 0.0F;

        for (int32_t i = from; i <= to; i++) {
            sum += this->line.times.at(i);
        }

        this->line.curvatures.at(this->cursor) = sum / static_cast<float>(to - from + 1);
    }

    if (this->cursor == this->line.size) {
        this->phase = Phase::SPEED;
    }
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::plan_speeds() {
    const uint16_t size = this->line.size;
    const float    spacing = this->line.spacing;

    const auto bending = [this, size, spacing](std::size_t index) {
        const std::size_t from = std::min<std::size_t>(index, size - 2U);

        return Bending{
            .curvature = this->line.curvatures.at(index),
            .sharpness = (this->line.curvatures.at(from + 1U) - this->line.curvatures.at(from)) / spacing,
        };
    };

    CurveLimits limits = this->dynamics.get_curve_limits(this->run_profile);
    limits.lateral *= this->config.lateral_share;

    CurveSpeed::plan(bending, std::span{this->line.speeds}.first(size), spacing, 0.0F, 0.0F, limits);
    CurveSpeed::integrate(std::span{this->line.speeds}.first(size), spacing, std::span{this->line.times}.first(size));

    this->line.finish_time = this->line.times.at(size - 1);

    const float cell_size = this->dynamics.get_model().maze.cell_size;

    for (uint16_t i = 0; i < size; i++) {
        const GridPoint cell = GridPoint::from_vector({.x = this->line.xs.at(i), .y = this->line.ys.at(i)}, cell_size);

        if (this->maze->is_goal(cell)) {
            this->line.finish_time = this->line.times.at(i);
            break;
        }
    }

    this->finish(true);
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::start_sweep() {
    this->sweep_slide = 0.0F;
    this->window_start = fixed_samples;
    this->start_window();
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::start_window() {
    this->window_size =
        static_cast<uint8_t>(std::min<int32_t>(window, this->line.size - fixed_samples - this->window_start));
    this->cursor = 0;
    this->phase = Phase::BOUNDS;
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::finish(bool found) {
    this->line.ready = found;
    this->maze = nullptr;
    this->segments = {};
    this->phase = Phase::IDLE;
}

template <uint8_t width, uint8_t height>
float TRacingLine<width, height>::get_heading(uint16_t index) const {
    return this->line.get_heading(index);
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::solve_slides(std::span<const double> normals_x, std::span<const double> normals_y) {
    const uint16_t first = this->window_start;
    const auto     size = static_cast<int32_t>(normals_x.size());
    const double   length_weight = static_cast<double>(this->config.length_weight) *
                                 static_cast<double>(this->line.spacing) * static_cast<double>(this->line.spacing);

    this->diagonal.fill(0.0);
    this->first_band.fill(0.0);
    this->second_band.fill(0.0);
    this->linear.fill(0.0);

    const auto add_row = [&](int32_t row, std::span<const double> weights, double scale) {
        double base_x = 0.0;
        double base_y = 0.0;

        for (std::size_t k = 0; k < weights.size(); k++) {
            const auto index = static_cast<uint16_t>(row + static_cast<int32_t>(k));
            base_x += weights[k] * static_cast<double>(this->line.xs.at(index));
            base_y += weights[k] * static_cast<double>(this->line.ys.at(index));
        }

        for (std::size_t k = 0; k < weights.size(); k++) {
            const int32_t j = row + static_cast<int32_t>(k) - first;

            if (j < 0 or j >= size) {
                continue;
            }

            this->linear.at(j) += scale * weights[k] * (base_x * normals_x[j] + base_y * normals_y[j]);

            for (std::size_t l = k; l < weights.size(); l++) {
                const int32_t m = row + static_cast<int32_t>(l) - first;

                if (m >= size) {
                    continue;
                }

                const double value =
                    scale * weights[k] * weights[l] * (normals_x[j] * normals_x[m] + normals_y[j] * normals_y[m]);

                if (m == j) {
                    this->diagonal.at(j) += value;
                } else if (m == j + 1) {
                    this->first_band.at(j) += value;
                } else {
                    this->second_band.at(j) += value;
                }
            }
        }
    };

    constexpr std::array<double, 3> second_difference{1.0, -2.0, 1.0};
    constexpr std::array<double, 2> first_difference{-1.0, 1.0};

    for (int32_t row = first - 2; row <= first + size - 1; row++) {
        add_row(row, second_difference, 1.0);
    }

    for (int32_t row = first - 1; row <= first + size - 1; row++) {
        add_row(row, first_difference, length_weight);
    }

    const auto product = [this, size](int32_t j, std::span<const double> values) {
        double result = this->diagonal.at(j) * values[j];

        if (j + 1 < size) {
            result += this->first_band.at(j) * values[j + 1];
        }

        if (j + 2 < size) {
            result += this->second_band.at(j) * values[j + 2];
        }

        if (j >= 1) {
            result += this->first_band.at(j - 1) * values[j - 1];
        }

        if (j >= 2) {
            result += this->second_band.at(j - 2) * values[j - 2];
        }

        return result;
    };

    const auto objective = [&](std::span<const double> values) {
        double result = 0.0;

        for (int32_t j = 0; j < size; j++) {
            result += values[j] * (product(j, values) + 2.0 * this->linear.at(j));
        }

        return result;
    };

    const auto coupling = [this](int32_t from, int32_t to) {
        if (to - from == 1) {
            return this->first_band.at(from);
        }

        return to - from == 2 ? this->second_band.at(from) : 0.0;
    };

    this->slides.fill(0.0);

    const std::span<const double> current{this->slides.data(), static_cast<std::size_t>(size)};
    const std::span<const double> candidate{this->trial.data(), static_cast<std::size_t>(size)};

    double value = 0.0;

    for (uint8_t iteration = 0; iteration < max_newton_iterations; iteration++) {
        uint8_t free = 0;

        for (int32_t j = 0; j < size; j++) {
            const double slope = product(j, current) + this->linear.at(j);
            const double low = this->lower.at(j);
            const double high = this->upper.at(j);

            this->gradient.at(j) = slope;
            this->direction.at(j) = 0.0;

            const bool at_lower = this->slides.at(j) <= low + 1.0e-9 and slope > 0.0;
            const bool at_upper = this->slides.at(j) >= high - 1.0e-9 and slope < 0.0;

            if (not at_lower and not at_upper) {
                this->free_indices.at(free++) = static_cast<uint8_t>(j);
            }
        }

        if (free == 0) {
            break;
        }

        bool singular = false;

        for (uint8_t c = 0; c < free; c++) {
            const uint8_t j = this->free_indices.at(c);
            const double  first_coupling = c >= 1 ? coupling(this->free_indices.at(c - 1), j) : 0.0;
            const double  second_coupling = c >= 2 ? coupling(this->free_indices.at(c - 2), j) : 0.0;

            const double second = c >= 2 ? second_coupling / this->factor_diagonal.at(c - 2) : 0.0;
            const double first_factor =
                c >= 1 ? (first_coupling -
                          (c >= 2 ? second * this->factor_first.at(c - 1) * this->factor_diagonal.at(c - 2) : 0.0)) /
                             this->factor_diagonal.at(c - 1) :
                         0.0;

            this->factor_second.at(c) = second;
            this->factor_first.at(c) = first_factor;
            this->factor_diagonal.at(c) =
                this->diagonal.at(j) - first_factor * first_factor * (c >= 1 ? this->factor_diagonal.at(c - 1) : 0.0) -
                second * second * (c >= 2 ? this->factor_diagonal.at(c - 2) : 0.0);

            if (this->factor_diagonal.at(c) <= 0.0) {
                singular = true;
                break;
            }
        }

        if (singular) {
            break;
        }

        for (uint8_t c = 0; c < free; c++) {
            double result = -this->gradient.at(this->free_indices.at(c));

            if (c >= 1) {
                result -= this->factor_first.at(c) * this->trial.at(c - 1);
            }

            if (c >= 2) {
                result -= this->factor_second.at(c) * this->trial.at(c - 2);
            }

            this->trial.at(c) = result;
        }

        for (int32_t c = free - 1; c >= 0; c--) {
            double result = this->trial.at(c) / this->factor_diagonal.at(c);

            if (c + 1 < free) {
                result -= this->factor_first.at(c + 1) * this->direction.at(this->free_indices.at(c + 1));
            }

            if (c + 2 < free) {
                result -= this->factor_second.at(c + 2) * this->direction.at(this->free_indices.at(c + 2));
            }

            this->direction.at(this->free_indices.at(c)) = result;
        }

        double step = 1.0;
        double trial_value = value;
        double moved = 0.0;

        for (uint8_t search = 0; search < 20; search++) {
            double decrease = 0.0;
            moved = 0.0;

            for (int32_t j = 0; j < size; j++) {
                this->trial.at(j) = std::clamp(
                    this->slides.at(j) + step * this->direction.at(j), static_cast<double>(this->lower.at(j)),
                    static_cast<double>(this->upper.at(j))
                );
                decrease += 2.0 * this->gradient.at(j) * (this->trial.at(j) - this->slides.at(j));
                moved = std::max(moved, std::abs(this->trial.at(j) - this->slides.at(j)));
            }

            trial_value = objective(candidate);

            if (trial_value <= value + 1.0e-4 * decrease) {
                break;
            }

            step /= 2.0;
        }

        if (trial_value > value) {
            break;
        }

        std::copy_n(this->trial.begin(), size, this->slides.begin());
        value = trial_value;

        if (moved < 1.0e-7) {
            break;
        }
    }
}

template <uint8_t width, uint8_t height>
bool TRacingLine<width, height>::is_clear(const core::Vector& position, float cosine, float sine, float margin) const {
    const float cell_size = this->dynamics.get_model().maze.cell_size;
    const float half_cell = cell_size / 2.0F;
    const float half_wall = this->dynamics.get_model().maze.wall_thickness / 2.0F;

    const auto post_x = static_cast<int32_t>(2 * std::lround(position.x / cell_size));
    const auto post_y = static_cast<int32_t>(2 * std::lround(position.y / cell_size));

    const core::Vector post{.x = static_cast<float>(post_x) * half_cell, .y = static_cast<float>(post_y) * half_cell};

    if (this->hits(position, cosine, sine, post, {.x = half_wall, .y = half_wall}, margin)) {
        return false;
    }

    constexpr std::array<std::array<int32_t, 2>, 4> around{{{0, 1}, {0, -1}, {1, 0}, {-1, 0}}};

    for (const auto& [dx, dy] : around) {
        if (this->is_open(post_x + dx, post_y + dy)) {
            continue;
        }

        const core::Vector center{
            .x = static_cast<float>(post_x + dx) * half_cell, .y = static_cast<float>(post_y + dy) * half_cell
        };
        const core::Vector half_size =
            dx == 0 ? core::Vector{.x = half_wall, .y = half_cell} : core::Vector{.x = half_cell, .y = half_wall};

        if (this->hits(position, cosine, sine, center, half_size, margin)) {
            return false;
        }
    }

    return true;
}

template <uint8_t width, uint8_t height>
bool TRacingLine<width, height>::hits(
    const core::Vector& position, float cosine, float sine, const core::Vector& center, const core::Vector& half_size,
    float margin
) const {
    const auto outside = [&center, &half_size, margin](float x, float y) {
        const float dx = std::max(std::abs(x - center.x) - half_size.x, 0.0F);
        const float dy = std::max(std::abs(y - center.y) - half_size.y, 0.0F);
        return dx * dx + dy * dy >= margin * margin;
    };

    const float bound = this->reach + margin;
    const float gap_x = std::max(std::abs(position.x - center.x) - half_size.x, 0.0F);
    const float gap_y = std::max(std::abs(position.y - center.y) - half_size.y, 0.0F);

    if (gap_x * gap_x + gap_y * gap_y >= bound * bound) {
        return false;
    }

    for (uint8_t i = 0; i < this->outline_size; i++) {
        const core::Vector& point = this->outline.at(i);

        if (not outside(
                position.x + cosine * point.x - sine * point.y, position.y + sine * point.x + cosine * point.y
            )) {
            return true;
        }
    }

    const float front = this->dynamics.get_model().chassis.front_length;
    const float rear = this->dynamics.get_model().chassis.rear_length;
    const float half = this->dynamics.get_model().chassis.half_width;

    for (const float sx : {-1.0F, 1.0F}) {
        for (const float sy : {-1.0F, 1.0F}) {
            const float offset_x = center.x + sx * half_size.x - position.x;
            const float offset_y = center.y + sy * half_size.y - position.y;
            const float along = cosine * offset_x + sine * offset_y;
            const float across = -sine * offset_x + cosine * offset_y;

            float dx = 0.0F;

            if (along > front) {
                dx = along - front;
            } else if (along < -rear) {
                dx = -rear - along;
            }

            const float dy = std::max(std::abs(across) - half, 0.0F);

            if (dx * dx + dy * dy < margin * margin) {
                return true;
            }
        }
    }

    return false;
}

template <uint8_t width, uint8_t height>
bool TRacingLine<width, height>::is_open(int32_t x, int32_t y) const {
    if (x < 0 or y < 0 or x >= lattice_width or y >= lattice_height) {
        return false;
    }

    const auto bit = static_cast<uint16_t>(y * lattice_width + x);
    return (this->openings.at(bit / 8U) & (1U << (bit % 8U))) != 0;
}

template <uint8_t width, uint8_t height>
void TRacingLine<width, height>::open(const GridPose& wall) {
    int32_t x = 2 * wall.position.x + 1;
    int32_t y = 2 * wall.position.y + 1;

    switch (wall.orientation) {
        case Side::RIGHT:
            x++;
            break;
        case Side::UP:
            y++;
            break;
        case Side::LEFT:
            x--;
            break;
        case Side::DOWN:
            y--;
            break;
    }

    if (x < 0 or y < 0 or x >= lattice_width or y >= lattice_height) {
        return;
    }

    const auto bit = static_cast<uint16_t>(y * lattice_width + x);
    this->openings.at(bit / 8U) = static_cast<uint8_t>(this->openings.at(bit / 8U) | (1U << (bit % 8U)));
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_RACING_LINE_TPP
