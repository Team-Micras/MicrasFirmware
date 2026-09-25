/**
 * @file
 */

#ifndef MICRAS_NAV_TURN_TABLE_HPP
#define MICRAS_NAV_TURN_TABLE_HPP

#include <algorithm>
#include <array>
#include <concepts>
#include <cstdint>
#include <numbers>
#include <utility>

#include "micras/core/math.hpp"
#include "micras/nav/lattice.hpp"
#include "micras/nav/robot_model.hpp"

namespace micras::nav {
/**
 * @brief How a path bends at one point: its curvature and the rate at which the curvature changes
 * with the distance traveled.
 *
 * @note Both are signed, positive to the left.
 */
struct Bending {
    float curvature;
    float sharpness;
};

/**
 * @brief One bend of a turn: a clothoid that ramps the curvature up, an arc at the peak curvature and
 * the mirrored clothoid that ramps it down.
 *
 * @details It is the path of a robot that holds its linear speed while its angular speed follows a
 * trapezoid. Only the path is stored: the speed it is driven at is chosen when a run is planned, and
 * scaling the angular speed with it and the angular acceleration with its square keeps the robot on
 * this same path at any speed.
 *
 * @note The angle is positive to the left. The curvature and the sharpness are magnitudes, and the
 * sign of the angle gives them their direction.
 */
struct TurnBend {
    /**
     * @brief Point of a curve, in the frame of its start.
     *
     * @tparam T Floating point type.
     */
    template <std::floating_point T>
    struct Sample {
        T x;
        T y;
        T heading;
        T curvature;
        T sharpness;
    };

    /**
     * @brief Evaluate the bend at a distance from its start.
     *
     * @tparam T Floating point type.
     * @param distance Distance traveled along the bend, clamped to its length.
     * @return The point of the bend, with the curvature and its rate of change there.
     */
    template <std::floating_point T>
    constexpr Sample<T> sample(T distance) const {
        const T ramp = static_cast<T>(this->ramp_length);
        const T arc = static_cast<T>(this->arc_length);
        const T rate = static_cast<T>(this->sharpness);
        const T peak = static_cast<T>(this->curvature);
        const T total = T{2} * ramp + arc;
        const T side = this->angle < 0.0F ? T{-1} : T{1};

        if (distance < T{0}) {
            distance = T{0};
        } else if (distance > total) {
            distance = total;
        }

        if (distance <= ramp) {
            const auto point = core::math::clothoid(distance, rate);

            return {
                .x = point.x,
                .y = side * point.y,
                .heading = side * rate * distance * distance / T{2},
                .curvature = side * rate * distance,
                .sharpness = side * rate,
            };
        }

        const auto ramp_end = core::math::clothoid(ramp, rate);
        const T    ramp_heading = rate * ramp * ramp / T{2};

        if (distance <= ramp + arc) {
            const T heading = ramp_heading + peak * (distance - ramp);

            return {
                .x = ramp_end.x + (core::math::sin(heading) - core::math::sin(ramp_heading)) / peak,
                .y = side * (ramp_end.y - (core::math::cos(heading) - core::math::cos(ramp_heading)) / peak),
                .heading = side * heading,
                .curvature = side * peak,
                .sharpness = T{0},
            };
        }

        const T angle = T{2} * ramp_heading + peak * arc;
        const T cosine = core::math::cos(angle);
        const T sine = core::math::sin(angle);

        const T arc_heading = ramp_heading + peak * arc;
        const T arc_end_x = arc > T{0} ?
                                ramp_end.x + (core::math::sin(arc_heading) - core::math::sin(ramp_heading)) / peak :
                                ramp_end.x;
        const T arc_end_y = arc > T{0} ?
                                ramp_end.y - (core::math::cos(arc_heading) - core::math::cos(ramp_heading)) / peak :
                                ramp_end.y;

        const T end_x = arc_end_x + cosine * ramp_end.x + sine * ramp_end.y;
        const T end_y = arc_end_y + sine * ramp_end.x - cosine * ramp_end.y;

        const T    remaining = total - distance;
        const auto mirrored = core::math::clothoid(remaining, rate);

        return {
            .x = end_x - (cosine * mirrored.x + sine * mirrored.y),
            .y = side * (end_y - (sine * mirrored.x - cosine * mirrored.y)),
            .heading = side * (angle - rate * remaining * remaining / T{2}),
            .curvature = side * rate * remaining,
            .sharpness = -side * rate,
        };
    }

    /**
     * @brief Get how the bend bends at a distance from its start, without evaluating its points.
     *
     * @param distance Distance traveled along the bend, clamped to its length.
     * @return The curvature and the sharpness there.
     */
    constexpr Bending bending_at(float distance) const {
        const float side = this->angle < 0.0F ? -1.0F : 1.0F;
        const float total = this->length();

        distance = std::clamp(distance, 0.0F, total);

        if (distance <= this->ramp_length) {
            return {.curvature = side * this->sharpness * distance, .sharpness = side * this->sharpness};
        }

        if (distance <= this->ramp_length + this->arc_length) {
            return {.curvature = side * this->curvature, .sharpness = 0.0F};
        }

        return {.curvature = side * this->sharpness * (total - distance), .sharpness = -side * this->sharpness};
    }

    /**
     * @brief Get the length of the bend.
     *
     * @return The length of the two ramps and the arc in meters.
     */
    constexpr float length() const { return 2.0F * this->ramp_length + this->arc_length; }

    float angle{};
    float curvature{};
    float sharpness{};
    float ramp_length{};
    float arc_length{};
};

/**
 * @brief Geometry of a slalom turn.
 *
 * @details A turn is one bend, or two bends joined by a straight, which is how a turn that fits inside
 * one cell swings out before it turns in. The planner prices the whole curve at one speed, so the
 * curvature and the sharpness that limit that speed are the largest of its bends. The robot then
 * drives it at the speed each point allows, which is only faster.
 *
 * @note The straights before and after the curve place it between its entry and exit nodes. All
 * lengths are in meters and the angles, in radians, are those of the turn to the left.
 */
struct TurnShape {
    /**
     * @brief Largest number of bends of a turn.
     */
    static constexpr uint8_t max_bends{2};

    /**
     * @brief Point of the curve, in the frame of its start, for the turn to the left.
     *
     * @tparam T Floating point type.
     */
    template <std::floating_point T>
    using Sample = TurnBend::Sample<T>;

    /**
     * @brief Evaluate the curve at a distance from its start.
     *
     * @tparam T Floating point type.
     * @param distance Distance traveled along the curve, clamped to its length.
     * @return The point of the curve, with the curvature and its rate of change there.
     */
    template <std::floating_point T>
    constexpr Sample<T> sample(T distance) const {
        const TurnBend& first = this->bends.at(0);
        const T         first_length = static_cast<T>(first.length());

        if (this->number_of_bends < 2 or distance <= first_length) {
            return first.template sample<T>(distance);
        }

        const Sample<T> joint = first.template sample<T>(first_length);
        const T         cosine = core::math::cos(joint.heading);
        const T         sine = core::math::sin(joint.heading);
        const T         straight = static_cast<T>(this->straight_length);
        const T         along = distance - first_length;

        if (along <= straight) {
            return {
                .x = joint.x + along * cosine,
                .y = joint.y + along * sine,
                .heading = joint.heading,
                .curvature = T{0},
                .sharpness = T{0},
            };
        }

        const Sample<T> local = this->bends.at(1).template sample<T>(along - straight);
        const T         start_x = joint.x + straight * cosine;
        const T         start_y = joint.y + straight * sine;

        return {
            .x = start_x + cosine * local.x - sine * local.y,
            .y = start_y + sine * local.x + cosine * local.y,
            .heading = joint.heading + local.heading,
            .curvature = local.curvature,
            .sharpness = local.sharpness,
        };
    }

    /**
     * @brief Get how the curve bends at a distance from its start, without evaluating its points.
     *
     * @param distance Distance traveled along the curve.
     * @return The curvature and the sharpness there.
     */
    constexpr Bending bending_at(float distance) const {
        const float first_length = this->bends.at(0).length();

        if (this->number_of_bends < 2 or distance <= first_length) {
            return this->bends.at(0).bending_at(distance);
        }

        if (distance <= first_length + this->straight_length) {
            return {.curvature = 0.0F, .sharpness = 0.0F};
        }

        return this->bends.at(1).bending_at(distance - first_length - this->straight_length);
    }

    /**
     * @brief Get the length of the curve.
     *
     * @return The length of the bends and of the straight between them in meters.
     */
    constexpr float length() const {
        if (this->number_of_bends < 2) {
            return this->bends.at(0).length();
        }

        return this->bends.at(0).length() + this->straight_length + this->bends.at(1).length();
    }

    /**
     * @brief Get the length of the whole turn, from its entry node to its exit node.
     *
     * @return The length of the curve and of the straights around it in meters.
     */
    constexpr float total_length() const { return this->pre + this->length() + this->post; }

    std::array<TurnBend, max_bends> bends{};
    uint8_t                         number_of_bends{};
    float                           straight_length{};

    /**
     * @brief Heading at the exit relative to the entry, and the largest curvature and sharpness.
     */
    float angle{};
    float curvature{};
    float sharpness{};

    float pre{};
    float post{};

    /**
     * @brief Distance from the entry node at which each wall between the nodes is crossed.
     */
    std::array<float, TurnPrimitive::max_gates> gates{};

    /**
     * @brief Whether a curve that fits between the nodes with the required clearance exists.
     */
    bool valid{};
};

/**
 * @brief Parameters of a turn of two bends, found by a search too long to run in the compiler.
 *
 * @details The angles are those of the turn to the left, positive to the left, and the curvatures
 * are the peaks asked of each bend. The straight between the bends and the one after them follow
 * from the place of the exit node, so a design is only the choice of how the turn swings.
 *
 */
struct TwoBendDesign {
    float first_angle;
    float first_curvature;
    float second_angle;
    float second_curvature;
    float pre;
};

/**
 * @brief Shape of every turn, computed from the geometry of the maze and of the robot.
 *
 * @details Each turn is made as wide as it can be while it still fits between its two nodes and
 * keeps the outline of the robot a safety margin away from every post and from every wall that is
 * not known to be absent, since a wider turn is a faster one. The clearance is measured on the path
 * itself, rather than on a circular arc of equivalent displacement.
 *
 * @note Everything here is a constant expression, so a table is normally built when the firmware is
 * compiled, where a turn that does not fit can be rejected by a static_assert on is_valid().
 */
class TurnTable {
public:
    /**
     * @brief Designs of the turns of two bends, from first_two_bend_turn on.
     */
    using TwoBendDesigns = std::array<TwoBendDesign, number_of_two_bend_turns>;

    /**
     * @brief Shapes of the turns of two bends, from first_two_bend_turn on.
     */
    using TwoBendShapes = std::array<TurnShape, number_of_two_bend_turns>;

    /**
     * @brief Compute the shape of every turn of one bend, and take those of two.
     *
     * @note The turns of two bends are placed between their nodes by place(), and checked to clear
     * the walls by clears(), each a constant expression of its own, since the compiler gives each one
     * a budget and designing the turns of one bend takes most of this one.
     *
     * @param model Physical description of the robot and of the maze.
     * @param margin Smallest distance allowed between the outline of the robot and an obstacle.
     * @param two_bend_shapes Shape of each turn of two bends, for this margin.
     */
    constexpr TurnTable(const RobotModel& model, float margin, const TwoBendShapes& two_bend_shapes) {
        const Designer designer{model, margin};

        for (uint8_t i = 0; i < first_two_bend_turn; i++) {
            this->shapes.at(i) = designer.design(static_cast<TurnId>(i));
        }

        std::ranges::copy(two_bend_shapes, this->shapes.begin() + first_two_bend_turn);
    }

    /**
     * @brief Place every turn of two bends between its nodes.
     *
     * @param model Physical description of the robot and of the maze.
     * @param margin Smallest distance allowed between the outline of the robot and an obstacle.
     * @param designs Design of each turn of two bends, for this margin.
     * @return The shapes, with the valid flag of each cleared if it does not end on the exit node of
     * its turn.
     */
    static constexpr TwoBendShapes place(const RobotModel& model, float margin, const TwoBendDesigns& designs) {
        const Designer designer{model, margin};
        TwoBendShapes  shapes{};

        for (uint8_t i = 0; i < number_of_two_bend_turns; i++) {
            shapes.at(i) = designer.build(static_cast<TurnId>(first_two_bend_turn + i), designs.at(i));
        }

        return shapes;
    }

    /**
     * @brief Build the shape of a turn of two bends and check that it fits.
     *
     * @note This is what the compiler runs on every design, and what the search for the designs runs
     * on every candidate, so a design is accepted by the same test that found it.
     *
     * @param model Physical description of the robot and of the maze.
     * @param margin Smallest distance allowed between the outline of the robot and an obstacle.
     * @param turn The turn.
     * @param design The parameters of the bends.
     * @return The shape, with its valid flag cleared if it does not end on the exit node of the turn
     * or does not clear every obstacle.
     */
    static constexpr TurnShape check(const RobotModel& model, float margin, TurnId turn, const TwoBendDesign& design) {
        return Designer{model, margin}.check(turn, design);
    }

    /**
     * @brief Check that every turn of two bends clears the walls.
     *
     * @param model Physical description of the robot and of the maze.
     * @param margin Smallest distance allowed between the outline of the robot and an obstacle.
     * @param designs Design of each turn of two bends, from first_two_bend_turn on, for this margin.
     * @return True if every design ends on its exit node and clears every obstacle.
     */
    static constexpr bool clears(const RobotModel& model, float margin, const TwoBendDesigns& designs) {
        const Designer designer{model, margin};

        for (uint8_t i = 0; i < number_of_two_bend_turns; i++) {
            if (not designer.check(static_cast<TurnId>(first_two_bend_turn + i), designs.at(i)).valid) {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Build one bend the way the turns are built.
     *
     * @param model Physical description of the robot.
     * @param angle The angle of the bend, positive to the left.
     * @param curvature The peak curvature asked for, which a bend too short for its ramps lowers.
     * @return The bend.
     */
    static constexpr TurnBend make_bend(const RobotModel& model, double angle, double curvature) {
        return Designer{model, 0.0F}.make_bend(angle, curvature);
    }

    /**
     * @brief Get the shape of a turn.
     *
     * @param turn The turn.
     * @return The shape of the turn.
     */
    constexpr const TurnShape& get(TurnId turn) const { return this->shapes.at(std::to_underlying(turn)); }

    /**
     * @brief Check if every turn fits.
     *
     * @return True if a shape was found for every turn, false otherwise.
     */
    constexpr bool is_valid() const {
        return std::ranges::all_of(this->shapes, [](const TurnShape& shape) { return shape.valid; });
    }

private:
    /**
     * @brief Solver for the shape of one turn, in double precision.
     */
    class Designer {
    public:
        /**
         * @brief Construct a new Designer object.
         *
         * @param model Physical description of the robot and of the maze.
         * @param margin Smallest distance allowed between the outline of the robot and an obstacle.
         */
        constexpr Designer(const RobotModel& model, float margin) :
            half_cell{static_cast<double>(model.maze.cell_size) / 2.0},
            half_wall{static_cast<double>(model.maze.wall_thickness) / 2.0},
            ramp_length{static_cast<double>(model.turn_ramp_length())},
            half_width{static_cast<double>(model.chassis.half_width)},
            front_length{static_cast<double>(model.chassis.front_length)},
            rear_length{static_cast<double>(model.chassis.rear_length)},
            margin{static_cast<double>(margin)} { }

        /**
         * @brief Find the widest shape of a turn that fits.
         *
         * @param turn The turn.
         * @return The shape, with its valid flag cleared if none fits.
         */
        constexpr TurnShape design(TurnId turn) const {
            const TurnPrimitive& primitive = get_primitive(turn);
            const Layout         layout = this->get_layout(primitive);

            if (primitive.rotation == 4) {
                return this->design_half_turn(primitive, layout);
            }

            double infeasible = 1.0 / (16.0 * this->half_cell);
            double feasible = 0.0;
            double curvature = infeasible;

            for (uint8_t i = 0; i < coarse_steps; i++) {
                if (this->fits(this->place(layout, curvature), layout)) {
                    feasible = curvature;
                    break;
                }

                infeasible = curvature;
                curvature *= coarse_ratio;
            }

            if (feasible == 0.0) {
                return {};
            }

            for (uint8_t i = 0; i < 14; i++) {
                const double middle = (infeasible + feasible) / 2.0;

                if (this->fits(this->place(layout, middle), layout)) {
                    feasible = middle;
                } else {
                    infeasible = middle;
                }
            }

            return finish(this->place(layout, feasible), layout);
        }

        /**
         * @brief Build the shape of a turn of two bends and check that it fits.
         *
         * @param turn The turn.
         * @param design The parameters of the bends.
         * @return The shape, with its valid flag cleared if it does not end on the exit node of the
         * turn or does not clear every obstacle.
         */
        constexpr TurnShape check(TurnId turn, const TwoBendDesign& design) const {
            TurnShape shape = this->build(turn, design);

            if (not shape.valid) {
                return shape;
            }

            const Layout layout = this->get_layout(get_primitive(turn));
            const double total = shape.total_length();
            const auto   samples = static_cast<uint16_t>(total / sample_spacing) + 2;

            for (uint16_t i = 0; i < samples; i++) {
                if (not this->is_clear(get_placement(shape, layout, total * i / (samples - 1)), layout)) {
                    shape.valid = false;
                    return shape;
                }
            }

            return shape;
        }

        /**
         * @brief Place a turn of two bends between its nodes.
         *
         * @details Given the bends and the straight before them, the exit node leaves two unknowns,
         * the straight between the bends and the one after them, and two equations for its position.
         * The shape is valid when both straights come out positive and the bends turn the angle of
         * the turn. Whether it clears the walls is left to check().
         *
         * @param turn The turn.
         * @param design The parameters of the bends.
         * @return The shape, with its valid flag cleared if it does not end on the exit node.
         */
        constexpr TurnShape build(TurnId turn, const TwoBendDesign& design) const {
            const TurnPrimitive& primitive = get_primitive(turn);

            if (design.first_curvature <= 0.0F or design.second_curvature <= 0.0F or design.pre < 0.0F) {
                return {};
            }

            const double first_angle = design.first_angle;
            const double second_angle = design.second_angle;
            const double turned = (first_angle + second_angle) / (std::numbers::pi / 4.0) - primitive.rotation;
            const double revolutions = turned / 8.0;
            const double rounded = revolutions < 0.0 ? -static_cast<double>(static_cast<int32_t>(0.5 - revolutions)) :
                                                       static_cast<double>(static_cast<int32_t>(revolutions + 0.5));

            if (core::math::abs(turned - 8.0 * rounded) > 1.0e-4) {
                return {};
            }

            const Candidate first = this->make_curve(core::math::abs(first_angle), design.first_curvature);
            const Candidate second = this->make_curve(core::math::abs(second_angle), design.second_curvature);

            TurnShape shape{};
            shape.bends.at(0) = to_bend(first, first_angle);
            shape.bends.at(1) = to_bend(second, second_angle);
            shape.number_of_bends = 2;
            shape.angle = static_cast<float>(first_angle + second_angle);
            shape.curvature = std::max(shape.bends.at(0).curvature, shape.bends.at(1).curvature);
            shape.sharpness = std::max(shape.bends.at(0).sharpness, shape.bends.at(1).sharpness);
            shape.pre = design.pre;

            const Layout layout = this->get_layout(primitive);

            const double entry = layout.entry_heading;
            const double middle = entry + first_angle;
            const double last = middle + second_angle;
            const double determinant = core::math::sin(second_angle);

            if (core::math::abs(determinant) < 1.0e-6) {
                return {};
            }

            const auto   first_end = shape.bends.at(0).template sample<double>(1.0e3);
            const auto   second_end = shape.bends.at(1).template sample<double>(1.0e3);
            const double remaining_x =
                layout.exit_x - shape.pre * core::math::cos(entry) -
                (core::math::cos(entry) * first_end.x - core::math::sin(entry) * first_end.y) -
                (core::math::cos(middle) * second_end.x - core::math::sin(middle) * second_end.y);
            const double remaining_y =
                layout.exit_y - shape.pre * core::math::sin(entry) -
                (core::math::sin(entry) * first_end.x + core::math::cos(entry) * first_end.y) -
                (core::math::sin(middle) * second_end.x + core::math::cos(middle) * second_end.y);

            const double straight =
                (remaining_x * core::math::sin(last) - remaining_y * core::math::cos(last)) / determinant;
            const double post =
                (core::math::cos(middle) * remaining_y - core::math::sin(middle) * remaining_x) / determinant;

            if (straight < -1.0e-6 or post < -1.0e-6) {
                return {};
            }

            shape.straight_length = static_cast<float>(straight > 0.0 ? straight : 0.0);
            shape.post = static_cast<float>(post > 0.0 ? post : 0.0);
            shape.valid = true;

            return shape;
        }

        /**
         * @brief Build one bend.
         *
         * @param angle The angle of the bend, positive to the left.
         * @param curvature The peak curvature asked for.
         * @return The bend.
         */
        constexpr TurnBend make_bend(double angle, double curvature) const {
            return to_bend(this->make_curve(core::math::abs(angle), curvature), angle);
        }

    private:
        /**
         * @brief Largest distance between the points of a turn of two bends where the clearance is
         * checked, in meters.
         *
         * @note The points are spread over the whole turn, straights included.
         */
        static constexpr double sample_spacing{0.005};

        /**
         * @brief Number of sizes tried, from the widest down, before refining the first that fits.
         */
        static constexpr uint8_t coarse_steps{48};

        /**
         * @brief Number of lengths tried for the straights around a turn of half a revolution.
         */
        static constexpr uint8_t straight_steps{40};

        /**
         * @brief Ratio between the curvatures of two consecutive sizes tried.
         */
        static constexpr double coarse_ratio{1.12};

        /**
         * @brief Largest number of obstacles considered around a turn.
         */
        static constexpr uint8_t max_obstacles{96};

        /**
         * @brief Number of points of the curve where the clearance is checked.
         */
        static constexpr uint8_t clearance_samples{32};

        /**
         * @brief Axis aligned rectangle in the canonical frame of a turn, in meters.
         */
        struct Rectangle {
            double center_x;
            double center_y;
            double half_x;
            double half_y;
        };

        /**
         * @brief Where a turn sits in the canonical frame, in meters and radians.
         */
        struct Layout {
            double                                          entry_heading{};
            double                                          angle{};
            double                                          exit_x{};
            double                                          exit_y{};
            double                                          tangent_in{};
            double                                          tangent_out{};
            std::array<Rectangle, max_obstacles>            obstacles{};
            uint8_t                                         number_of_obstacles{};
            std::array<Rectangle, TurnPrimitive::max_gates> gates{};
            uint8_t                                         number_of_gates{};
        };

        /**
         * @brief Candidate shape, in double precision.
         */
        struct Candidate {
            double curvature{};
            double sharpness{};
            double ramp{};
            double arc{};
            double pre{};
            double post{};
        };

        /**
         * @brief Pose of the robot in the canonical frame.
         */
        struct Placement {
            double x;
            double y;
            double heading;
        };

        /**
         * @brief Build the curve of a candidate as a shape, to evaluate it.
         *
         * @param candidate The candidate.
         * @param angle The angle of the turn.
         * @return The shape with the fields of the curve filled in.
         */
        static constexpr TurnShape to_shape(const Candidate& candidate, double angle) {
            TurnShape shape{};
            shape.bends.at(0) = to_bend(candidate, angle);
            shape.number_of_bends = 1;
            shape.angle = static_cast<float>(angle);
            shape.curvature = static_cast<float>(candidate.curvature);
            shape.sharpness = static_cast<float>(candidate.sharpness);
            shape.pre = static_cast<float>(candidate.pre);
            shape.post = static_cast<float>(candidate.post);
            return shape;
        }

        /**
         * @brief Build the bend of a candidate.
         *
         * @param candidate The candidate.
         * @param angle The angle of the bend, positive to the left.
         * @return The bend.
         */
        static constexpr TurnBend to_bend(const Candidate& candidate, double angle) {
            return {
                .angle = static_cast<float>(angle),
                .curvature = static_cast<float>(candidate.curvature),
                .sharpness = static_cast<float>(candidate.sharpness),
                .ramp_length = static_cast<float>(candidate.ramp),
                .arc_length = static_cast<float>(candidate.arc),
            };
        }

        /**
         * @brief Build the curve that turns a given angle with a given peak curvature.
         *
         * @note When the ramps alone would turn more than the angle, they are shortened and the arc
         * disappears, which keeps the rate of change of the curvature.
         *
         * @param angle The angle of the turn.
         * @param curvature The peak curvature requested.
         * @return The candidate, without the straights.
         */
        constexpr Candidate make_curve(double angle, double curvature) const {
            Candidate candidate{};
            candidate.sharpness = curvature / this->ramp_length;
            candidate.ramp = this->ramp_length;
            candidate.curvature = curvature;

            if (curvature * this->ramp_length > angle) {
                candidate.ramp = core::math::sqrt(angle / candidate.sharpness);
                candidate.curvature = candidate.sharpness * candidate.ramp;
            }

            candidate.arc = (angle - candidate.curvature * candidate.ramp) / candidate.curvature;

            return candidate;
        }

        /**
         * @brief Get the pose of the robot at a distance from the entry node.
         *
         * @param candidate The candidate shape.
         * @param layout The place of the turn.
         * @param distance Distance traveled from the entry node.
         * @return The pose in the canonical frame.
         */
        static constexpr Placement get_placement(const Candidate& candidate, const Layout& layout, double distance) {
            const double cosine = core::math::cos(layout.entry_heading);
            const double sine = core::math::sin(layout.entry_heading);
            const double length = 2.0 * candidate.ramp + candidate.arc;

            double local_x = distance;
            double local_y = 0.0;
            double heading = 0.0;

            if (distance > candidate.pre) {
                const auto sample = to_shape(candidate, layout.angle).template sample<double>(distance - candidate.pre);

                local_x = candidate.pre + sample.x;
                local_y = sample.y;
                heading = sample.heading;

                if (distance > candidate.pre + length) {
                    const double extra = distance - candidate.pre - length;
                    local_x += extra * core::math::cos(layout.angle);
                    local_y += extra * core::math::sin(layout.angle);
                }
            }

            return {
                .x = cosine * local_x - sine * local_y,
                .y = sine * local_x + cosine * local_y,
                .heading = layout.entry_heading + heading,
            };
        }

        /**
         * @brief Get the pose of the robot at a distance from the entry node, along a shape.
         *
         * @param shape The shape, with its straights.
         * @param layout The place of the turn.
         * @param distance Distance traveled from the entry node.
         * @return The pose in the canonical frame.
         */
        static constexpr Placement get_placement(const TurnShape& shape, const Layout& layout, double distance) {
            const double cosine = core::math::cos(layout.entry_heading);
            const double sine = core::math::sin(layout.entry_heading);
            const double pre = shape.pre;
            const double length = shape.length();

            double local_x = distance;
            double local_y = 0.0;
            double heading = 0.0;

            if (distance > pre) {
                const auto sample = shape.template sample<double>(distance > pre + length ? length : distance - pre);

                local_x = pre + sample.x;
                local_y = sample.y;
                heading = sample.heading;

                if (distance > pre + length) {
                    const double extra = distance - pre - length;
                    local_x += extra * core::math::cos(heading);
                    local_y += extra * core::math::sin(heading);
                }
            }

            return {
                .x = cosine * local_x - sine * local_y,
                .y = sine * local_x + cosine * local_y,
                .heading = layout.entry_heading + heading,
            };
        }

        /**
         * @brief Collect the geometry of a turn in its canonical frame.
         *
         * @param primitive The place of the turn in the lattice.
         * @return The layout, with every post and every wall that may exist around the turn.
         */
        constexpr Layout get_layout(const TurnPrimitive& primitive) const {
            Layout layout{};
            layout.entry_heading = primitive.diagonal_entry ? std::numbers::pi / 4.0 : 0.0;
            layout.angle = primitive.rotation * std::numbers::pi / 4.0;
            layout.exit_x = primitive.exit.x * this->half_cell;
            layout.exit_y = primitive.exit.y * this->half_cell;
            layout.number_of_gates = primitive.number_of_gates;

            const double cosine = core::math::cos(layout.entry_heading);
            const double sine = core::math::sin(layout.entry_heading);
            const double forward = cosine * layout.exit_x + sine * layout.exit_y;
            const double lateral = -sine * layout.exit_x + cosine * layout.exit_y;

            if (primitive.rotation % 4 != 0) {
                layout.tangent_out = lateral / core::math::sin(layout.angle);
                layout.tangent_in = forward - layout.tangent_out * core::math::cos(layout.angle);
            }

            for (uint8_t i = 0; i < primitive.number_of_gates; i++) {
                layout.gates.at(i) = this->get_wall(primitive.gates.at(i));
            }

            const double reach = 3.0 * this->half_cell;
            const double low_x = (layout.exit_x < 0.0 ? layout.exit_x : 0.0) - reach;
            const double high_x = (layout.exit_x > 0.0 ? layout.exit_x : 0.0) + reach;
            const double low_y = (layout.exit_y < 0.0 ? layout.exit_y : 0.0) - reach;
            const double high_y = (layout.exit_y > 0.0 ? layout.exit_y : 0.0) + reach;

            for (int8_t column = -6; column <= 8; column++) {
                for (int8_t row = -7; row <= 7; row++) {
                    const LatticePoint point{.x = column, .y = row};

                    if (column % 2 != 0 and row % 2 == 0) {
                        continue;
                    }

                    if (is_open(primitive, point) or layout.number_of_obstacles == max_obstacles) {
                        continue;
                    }

                    const Rectangle obstacle = this->get_wall(point);

                    if (obstacle.center_x < low_x or obstacle.center_x > high_x or obstacle.center_y < low_y or
                        obstacle.center_y > high_y) {
                        continue;
                    }

                    layout.obstacles.at(layout.number_of_obstacles) = obstacle;
                    layout.number_of_obstacles++;
                }
            }

            return layout;
        }

        /**
         * @brief Check if a point of the canonical frame is a wall the turn needs to be absent.
         *
         * @note In the canonical frame the entry node is on a vertical wall, so the posts are the
         * points with an even x and an odd y, the vertical walls have both coordinates even and
         * the horizontal ones have both odd.
         *
         * @param primitive The place of the turn in the lattice.
         * @param point The point, relative to the entry node.
         * @return True if the robot crosses the wall at that point.
         */
        static constexpr bool is_open(const TurnPrimitive& primitive, const LatticePoint& point) {
            if (point == LatticePoint{.x = 0, .y = 0} or point == primitive.exit) {
                return true;
            }

            for (uint8_t i = 0; i < primitive.number_of_gates; i++) {
                if (point == primitive.gates.at(i)) {
                    return true;
                }
            }

            return false;
        }

        /**
         * @brief Get the rectangle occupied by a post or by a wall.
         *
         * @param point The post or the midpoint of the wall, relative to the entry node.
         * @return The rectangle in the canonical frame.
         */
        constexpr Rectangle get_wall(const LatticePoint& point) const {
            const bool is_post = point.x % 2 == 0 and point.y % 2 != 0;
            const bool is_vertical = point.x % 2 == 0;

            return {
                .center_x = point.x * this->half_cell,
                .center_y = point.y * this->half_cell,
                .half_x = (is_post or is_vertical) ? this->half_wall : this->half_cell,
                .half_y = (is_post or not is_vertical) ? this->half_wall : this->half_cell,
            };
        }

        /**
         * @brief Place a curve between the nodes of a turn.
         *
         * @param layout The place of the turn.
         * @param curvature The peak curvature requested.
         * @return The candidate, whose straights are negative when the curve does not fit.
         */
        constexpr Candidate place(const Layout& layout, double curvature) const {
            Candidate    candidate = this->make_curve(layout.angle, curvature);
            const auto   end = to_shape(candidate, layout.angle).template sample<double>(1.0e3);
            const double tangent = end.y / core::math::sin(layout.angle);

            candidate.pre = layout.tangent_in - tangent;
            candidate.post = layout.tangent_out - tangent;

            return candidate;
        }

        /**
         * @brief Check if a candidate fits between its nodes and clears every obstacle.
         *
         * @param candidate The candidate shape.
         * @param layout The place of the turn.
         * @return True if the candidate can be driven.
         */
        constexpr bool fits(const Candidate& candidate, const Layout& layout) const {
            if (candidate.pre < 0.0 or candidate.post < 0.0) {
                return false;
            }

            const double length = 2.0 * candidate.ramp + candidate.arc;

            for (uint8_t i = 0; i <= clearance_samples + 1; i++) {
                double distance = candidate.pre + length * (i - 1) / (clearance_samples - 1);

                if (i == 0) {
                    distance = 0.0;
                } else if (i == clearance_samples + 1) {
                    distance = candidate.pre + length + candidate.post;
                }

                if (not this->is_clear(get_placement(candidate, layout, distance), layout)) {
                    return false;
                }
            }

            return true;
        }

        /**
         * @brief Check if the robot keeps clear of every obstacle at one pose.
         *
         * @param placement The pose of the robot.
         * @param layout The place of the turn.
         * @return True if the robot stays at least the margin away from every obstacle.
         */
        constexpr bool is_clear(const Placement& placement, const Layout& layout) const {
            const double longest = this->front_length > this->rear_length ? this->front_length : this->rear_length;
            const double reach =
                core::math::sqrt(this->half_width * this->half_width + longest * longest) + this->margin;

            for (uint8_t j = 0; j < layout.number_of_obstacles; j++) {
                const Rectangle& obstacle = layout.obstacles.at(j);

                if (core::math::abs(obstacle.center_x - placement.x) > reach + obstacle.half_x or
                    core::math::abs(obstacle.center_y - placement.y) > reach + obstacle.half_y) {
                    continue;
                }

                if (this->touches(placement, obstacle)) {
                    return false;
                }
            }

            return true;
        }

        /**
         * @brief Check if a point is closer than the margin to a rectangle centered at the origin.
         *
         * @param x The x coordinate of the point in the frame of the rectangle.
         * @param y The y coordinate of the point in the frame of the rectangle.
         * @param low_x The lowest x coordinate of the rectangle.
         * @param high_x The highest x coordinate of the rectangle.
         * @param half_y Half of the size of the rectangle along y.
         * @return True if the point is within the margin of the rectangle.
         */
        constexpr bool is_near(double x, double y, double low_x, double high_x, double half_y) const {
            double excess_x = 0.0;

            if (x > high_x) {
                excess_x = x - high_x;
            } else if (x < low_x) {
                excess_x = low_x - x;
            }

            const double excess_y = core::math::abs(y) > half_y ? core::math::abs(y) - half_y : 0.0;

            return excess_x * excess_x + excess_y * excess_y < this->margin * this->margin;
        }

        /**
         * @brief Check if the outline of the robot comes within the margin of an obstacle.
         *
         * @note Both shapes are convex and none of them can be inside the other, so the smallest
         * distance between them is always measured from a corner of one of the two.
         *
         * @param placement The pose of the robot.
         * @param obstacle The obstacle.
         * @return True if the robot is closer to the obstacle than the margin.
         */
        constexpr bool touches(const Placement& placement, const Rectangle& obstacle) const {
            const double cosine = core::math::cos(placement.heading);
            const double sine = core::math::sin(placement.heading);

            for (uint8_t corner = 0; corner < 4; corner++) {
                const double along = corner < 2 ? this->front_length : -this->rear_length;
                const double across = corner % 2 == 0 ? this->half_width : -this->half_width;

                const double corner_x = placement.x + cosine * along - sine * across - obstacle.center_x;
                const double corner_y = placement.y + sine * along + cosine * across - obstacle.center_y;

                if (this->is_near(corner_x, corner_y, -obstacle.half_x, obstacle.half_x, obstacle.half_y)) {
                    return true;
                }

                const double offset_x =
                    obstacle.center_x + (corner < 2 ? obstacle.half_x : -obstacle.half_x) - placement.x;
                const double offset_y =
                    obstacle.center_y + (corner % 2 == 0 ? obstacle.half_y : -obstacle.half_y) - placement.y;

                const double body_x = cosine * offset_x + sine * offset_y;
                const double body_y = -sine * offset_x + cosine * offset_y;

                if (this->is_near(body_x, body_y, -this->rear_length, this->front_length, this->half_width)) {
                    return true;
                }
            }

            return false;
        }

        /**
         * @brief Find the shape of a turn of half a revolution.
         *
         * @note The exit is parallel to the entry, so the size of the curve is fixed by the distance
         * between them and what is left to choose is how far into the cell it is made.
         *
         * @param primitive The place of the turn in the lattice.
         * @param layout The place of the turn.
         * @return The shape, with its valid flag cleared if none fits.
         */
        constexpr TurnShape design_half_turn(const TurnPrimitive& primitive, const Layout& layout) const {
            const double lateral = primitive.exit.y * this->half_cell;

            double tight = 1.0 / (0.1 * this->half_cell);
            double wide = 1.0 / (16.0 * this->half_cell);

            for (uint8_t i = 0; i < 48; i++) {
                const double    middle = (tight + wide) / 2.0;
                const Candidate candidate = this->make_curve(layout.angle, middle);
                const auto      end = to_shape(candidate, layout.angle).template sample<double>(1.0e3);

                if (end.y > lateral) {
                    wide = middle;
                } else {
                    tight = middle;
                }
            }

            Candidate candidate = this->make_curve(layout.angle, (tight + wide) / 2.0);
            double    blocked = -1.0;
            double    clear = -1.0;

            for (uint8_t i = 0; i < straight_steps; i++) {
                const double straight = 2.0 * this->half_cell * i / straight_steps;

                candidate.pre = straight;
                candidate.post = straight;

                if (this->fits(candidate, layout)) {
                    clear = straight;
                    break;
                }

                blocked = straight;
            }

            if (clear < 0.0) {
                return {};
            }

            for (uint8_t i = 0; i < 10 and blocked >= 0.0; i++) {
                const double middle = (blocked + clear) / 2.0;
                candidate.pre = middle;
                candidate.post = middle;

                if (this->fits(candidate, layout)) {
                    clear = middle;
                } else {
                    blocked = middle;
                }
            }

            candidate.pre = clear;
            candidate.post = clear;

            return finish(candidate, layout);
        }

        /**
         * @brief Turn a candidate into a shape, locating where it crosses each gate.
         *
         * @note A gate is looked for from the end of the turn backwards, since a turn that doubles
         * back can touch the line of a wall before the place where it actually crosses it.
         *
         * @param candidate The candidate shape.
         * @param layout The place of the turn.
         * @return The shape.
         */
        static constexpr TurnShape finish(const Candidate& candidate, const Layout& layout) {
            TurnShape shape = to_shape(candidate, layout.angle);
            shape.valid = true;

            const double total = candidate.pre + 2.0 * candidate.ramp + candidate.arc + candidate.post;

            for (uint8_t i = 0; i < layout.number_of_gates; i++) {
                const Rectangle& gate = layout.gates.at(i);
                const bool       vertical = gate.half_x < gate.half_y;

                const auto side = [&](double distance) {
                    const Placement placement = get_placement(candidate, layout, distance);
                    return vertical ? placement.x - gate.center_x : placement.y - gate.center_y;
                };

                const double final_side = side(total);
                double       after = total;
                double       before = 0.0;

                for (uint8_t j = 1; j <= 64; j++) {
                    const double distance = total * (64 - j) / 64.0;

                    if ((side(distance) > 0.0) != (final_side > 0.0)) {
                        before = distance;
                        break;
                    }

                    after = distance;
                }

                for (uint8_t j = 0; j < 30; j++) {
                    const double middle = (before + after) / 2.0;

                    if ((side(middle) > 0.0) != (final_side > 0.0)) {
                        before = middle;
                    } else {
                        after = middle;
                    }
                }

                shape.gates.at(i) = static_cast<float>((before + after) / 2.0);
            }

            return shape;
        }

        double half_cell;
        double half_wall;
        double ramp_length;
        double half_width;
        double front_length;
        double rear_length;
        double margin;
    };

    /**
     * @brief Shape of each turn, indexed by TurnId.
     */
    std::array<TurnShape, number_of_turns> shapes{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_TURN_TABLE_HPP
