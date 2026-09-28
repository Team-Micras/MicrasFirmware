/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "micras/nav/motion_limits.hpp"
#include "micras/nav/speed_profile.hpp"

namespace micras::nav {
SpeedProfile::SpeedProfile(float distance, float start_speed, float end_speed, const MotionLimits& limits) :
    total_distance{std::max(distance, 0.0F)}, last_speed{start_speed} {
    if (this->total_distance <= 0.0F) {
        return;
    }

    const float braking_distance = get_braking_distance(start_speed, end_speed, limits);

    if (braking_distance >= this->total_distance) {
        if (limits.braking_crossover_speed() <= end_speed) {
            const float deceleration =
                (start_speed * start_speed - end_speed * end_speed) / (2.0F * this->total_distance);

            this->append({.duration = (start_speed - end_speed) / deceleration, .acceleration = -deceleration});
            return;
        }

        MotionLimits harder = limits;
        harder.deceleration *= braking_distance / this->total_distance;
        harder.motor_acceleration *= braking_distance / this->total_distance;

        this->append_braking(start_speed, end_speed, harder);
        return;
    }

    const float cap = std::max(limits.max_speed, start_speed);
    float       low = std::max(start_speed, end_speed);
    float       peak = cap;

    const auto excess = [&](float speed) {
        return get_acceleration_distance(start_speed, speed, limits) + get_braking_distance(speed, end_speed, limits) -
               this->total_distance;
    };

    if (excess(low) >= 0.0F) {
        peak = get_reachable_speed(this->total_distance, start_speed, limits);
    } else if (excess(cap) > 0.0F) {
        float high = cap;

        for (uint8_t i = 0; i < 32; i++) {
            const float middle = (low + high) / 2.0F;

            if (excess(middle) > 0.0F) {
                high = middle;
            } else {
                low = middle;
            }
        }

        peak = low;
    }

    const float crossover = limits.crossover_speed();
    const float traction_end = std::clamp(crossover, start_speed, peak);

    if (traction_end > start_speed) {
        this->append(
            {.duration = (traction_end - start_speed) / limits.acceleration, .acceleration = limits.acceleration}
        );
    }

    if (peak > traction_end) {
        const float rate = limits.motor_acceleration / limits.motor_speed;

        this->append({
            .duration = std::log((limits.motor_speed - traction_end) / (limits.motor_speed - peak)) / rate,
            .rate = rate,
            .target = limits.motor_speed,
        });
    }

    const float final_speed = std::min(peak, end_speed);
    const float braking = get_braking_distance(peak, final_speed, limits);

    float accelerated = 0.0F;

    if (this->size > 0) {
        const Phase& last = this->phases.at(this->size - 1);
        accelerated = evaluate(last, last.duration).distance;
    }

    const float cruise = this->total_distance - accelerated - braking;

    if (cruise > 0.0F and peak > 0.0F) {
        this->append({.duration = cruise / peak});
    }

    this->append_braking(peak, final_speed, limits);
}

float SpeedProfile::duration() const {
    return this->total_duration;
}

float SpeedProfile::distance() const {
    return this->total_distance;
}

SpeedProfile::Sample SpeedProfile::sample(float time) const {
    if (this->size == 0) {
        return {.distance = this->total_distance, .speed = this->last_speed, .acceleration = 0.0F};
    }

    time = std::clamp(time, 0.0F, this->total_duration);

    for (uint8_t i = 0; i < this->size; i++) {
        const Phase& phase = this->phases.at(i);

        if (time <= phase.duration or i + 1 == this->size) {
            Sample result = evaluate(phase, std::min(time, phase.duration));
            result.distance = std::min(result.distance, this->total_distance);
            return result;
        }

        time -= phase.duration;
    }

    return {.distance = this->total_distance, .speed = this->last_speed, .acceleration = 0.0F};
}

float SpeedProfile::time_at(float distance) const {
    distance = std::clamp(distance, 0.0F, this->total_distance);
    float elapsed = 0.0F;

    for (uint8_t i = 0; i < this->size; i++) {
        const Phase& phase = this->phases.at(i);
        const float  end_distance = evaluate(phase, phase.duration).distance;

        if (distance > end_distance and i + 1 < this->size) {
            elapsed += phase.duration;
            continue;
        }

        float low = 0.0F;
        float high = phase.duration;

        for (uint8_t j = 0; j < 32; j++) {
            const float middle = (low + high) / 2.0F;

            if (evaluate(phase, middle).distance < distance) {
                low = middle;
            } else {
                high = middle;
            }
        }

        return elapsed + (low + high) / 2.0F;
    }

    return elapsed;
}

float SpeedProfile::get_reachable_speed(float distance, float start_speed, const MotionLimits& limits) {
    if (distance <= 0.0F or start_speed >= limits.max_speed) {
        return start_speed;
    }

    if (get_acceleration_distance(start_speed, limits.max_speed, limits) <= distance) {
        return limits.max_speed;
    }

    const float crossover = limits.crossover_speed();
    const float unlimited = std::sqrt(start_speed * start_speed + 2.0F * limits.acceleration * distance);

    if (unlimited <= crossover) {
        return unlimited;
    }

    float low = start_speed;
    float high = limits.max_speed;

    for (uint8_t i = 0; i < 32; i++) {
        const float middle = (low + high) / 2.0F;

        if (get_acceleration_distance(start_speed, middle, limits) > distance) {
            high = middle;
        } else {
            low = middle;
        }
    }

    return low;
}

float SpeedProfile::get_brakeable_speed(float distance, float end_speed, const MotionLimits& limits) {
    if (distance <= 0.0F) {
        return end_speed;
    }

    const float crossover = limits.braking_crossover_speed();

    if (crossover <= end_speed or end_speed >= limits.max_speed) {
        return std::min(limits.max_speed, std::sqrt(end_speed * end_speed + 2.0F * limits.deceleration * distance));
    }

    const float motor_distance = get_braking_distance(crossover, end_speed, limits);

    if (distance >= motor_distance) {
        return std::min(
            limits.max_speed,
            std::sqrt(crossover * crossover + 2.0F * limits.deceleration * (distance - motor_distance))
        );
    }

    float speed = crossover;

    for (uint8_t i = 0; i < newton_steps; i++) {
        speed -= (get_braking_distance(speed, end_speed, limits) - distance) * limits.deceleration_at(speed) / speed;
    }

    return std::clamp(speed, end_speed, limits.max_speed);
}

float SpeedProfile::get_braking_distance(float start_speed, float end_speed, const MotionLimits& limits) {
    if (start_speed <= end_speed) {
        return 0.0F;
    }

    const float braking_end = std::clamp(limits.braking_crossover_speed(), end_speed, start_speed);

    float distance = (start_speed * start_speed - braking_end * braking_end) / (2.0F * limits.deceleration);

    if (braking_end > end_speed) {
        const float rate = limits.motor_acceleration / limits.motor_speed;
        const float span = braking_end - end_speed;

        distance +=
            std::max(span - limits.motor_speed * std::log1p(span / (limits.motor_speed + end_speed)), 0.0F) / rate;
    }

    return distance;
}

float SpeedProfile::get_braked_speed(float distance, float start_speed, const MotionLimits& limits) {
    if (get_braking_distance(start_speed, 0.0F, limits) <= distance) {
        return 0.0F;
    }

    float low = 0.0F;
    float high = start_speed;

    for (uint8_t i = 0; i < bisection_steps; i++) {
        const float middle = (low + high) / 2.0F;

        if (get_braking_distance(start_speed, middle, limits) > distance) {
            low = middle;
        } else {
            high = middle;
        }
    }

    return high;
}

void SpeedProfile::append_braking(float start_speed, float end_speed, const MotionLimits& limits) {
    const float braking_end = std::clamp(limits.braking_crossover_speed(), end_speed, start_speed);

    if (start_speed > braking_end) {
        this->append(
            {.duration = (start_speed - braking_end) / limits.deceleration, .acceleration = -limits.deceleration}
        );
    }

    if (braking_end > end_speed) {
        const float rate = limits.motor_acceleration / limits.motor_speed;

        this->append({
            .duration = std::log1p((braking_end - end_speed) / (limits.motor_speed + end_speed)) / rate,
            .rate = rate,
            .target = -limits.motor_speed,
        });
    }
}

float SpeedProfile::get_acceleration_distance(float start_speed, float end_speed, const MotionLimits& limits) {
    if (end_speed <= start_speed) {
        return 0.0F;
    }

    const float traction_end = std::clamp(limits.crossover_speed(), start_speed, end_speed);

    float distance = (traction_end * traction_end - start_speed * start_speed) / (2.0F * limits.acceleration);

    if (end_speed > traction_end) {
        const float rate = limits.motor_acceleration / limits.motor_speed;

        distance +=
            (limits.motor_speed * std::log((limits.motor_speed - traction_end) / (limits.motor_speed - end_speed)) -
             (end_speed - traction_end)) /
            rate;
    }

    return distance;
}

void SpeedProfile::append(Phase phase) {
    if (phase.duration <= 0.0F or this->size == number_of_phases) {
        return;
    }

    if (this->size > 0) {
        const Phase& last = this->phases.at(this->size - 1);
        phase.start_distance = evaluate(last, last.duration).distance;
    }

    phase.start_speed = this->last_speed;

    this->last_speed = evaluate(phase, phase.duration).speed;
    this->total_duration += phase.duration;
    this->phases.at(this->size) = phase;
    this->size++;
}

SpeedProfile::Sample SpeedProfile::evaluate(const Phase& phase, float time) {
    if (phase.rate > 0.0F) {
        const float gap = phase.target - phase.start_speed;
        const float decay = std::exp(-phase.rate * time);

        return {
            .distance = phase.start_distance + phase.target * time - gap * (1.0F - decay) / phase.rate,
            .speed = phase.target - gap * decay,
            .acceleration = phase.rate * gap * decay,
        };
    }

    return {
        .distance = phase.start_distance + phase.start_speed * time + phase.acceleration * time * time / 2.0F,
        .speed = phase.start_speed + phase.acceleration * time,
        .acceleration = phase.acceleration,
    };
}
}  // namespace micras::nav
