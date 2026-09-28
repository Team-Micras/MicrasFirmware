/**
 * @file
 *
 * @brief Designs the turns of two bends and writes them as the firmware's two_bend_turns.hpp.
 *
 * @note A turn of two bends is two bends joined by a straight, and choosing them is a search over the
 *       angle of the first bend, the curvature of each and the straight before them, far too long for
 *       the compiler to run. So it runs here: every combination on a grid that ends on the exit node of
 *       the turn is timed at the speed its tighter bend allows, and the fastest one that the firmware's
 *       own check accepts, clear of every post and of every wall the turn does not cross, is the
 *       design. The firmware checks it again when it is compiled.
 *
 *       Usage: micras_turn_designer > MicrasFirmware/config/two_bend_turns.hpp
 *       The table of what was found, speeds and radii, goes to stderr.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <future>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

#include "micras/nav/lattice.hpp"
#include "micras/nav/turn_table.hpp"
#include "robot.hpp"
#include "turn_margins.hpp"

namespace {
using micras::nav::TurnBend;
using micras::nav::TurnId;
using micras::nav::TurnTable;
using micras::nav::TwoBendDesign;

/**
 * @brief Names of the turns of two bends, in the order of TurnId.
 */
constexpr std::array<const char*, micras::nav::number_of_two_bend_turns> names{
    "SD45E", "SD45T", "SD135T", "DS45T", "DD0E", "DS45E", "DD90E", "DS135B",
};

/**
 * @brief Step of the grid of the angle of the first bend, and its largest magnitude, in degrees.
 */
///@{
constexpr double angle_step{2.5};
constexpr double max_angle{220.0};
///@}

/**
 * @brief Smallest angle of a bend, in degrees, below which it is not worth a bend.
 */
constexpr double min_angle{4.0};

/**
 * @brief Number of radii of each bend, spaced evenly in logarithm between the smallest and largest.
 */
///@{
constexpr int    number_of_radii{30};
constexpr double min_radius{0.03};
constexpr double max_radius{0.6};
///@}

/**
 * @brief Step and number of lengths of the straight before the bends, in meters.
 */
///@{
constexpr double pre_step{0.005};
constexpr int    number_of_pres{41};

///@}

/**
 * @brief Where a turn starts and ends, in its canonical frame.
 */
struct Frame {
    double entry;
    double exit_x;
    double exit_y;
};

/**
 * @brief A design with the time it takes, for sorting.
 */
struct Candidate {
    double        time;
    TwoBendDesign design;
};

/**
 * @brief What was found for one turn and one margin.
 */
struct Result {
    TwoBendDesign design{};
    double        speed{};
    double        time{};
    std::size_t   tried{};
};

/**
 * @brief Convert degrees to radians.
 *
 * @param degrees The angle in degrees.
 * @return The angle in radians.
 */
double to_radians(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

/**
 * @brief Curvature of one radius of the grid.
 *
 * @param index The index of the radius.
 * @return The curvature in 1/m.
 */
double curvature_of(int index) {
    return 1.0 / (min_radius * std::pow(max_radius / min_radius, index / (number_of_radii - 1.0)));
}

/**
 * @brief Speed a bend can be driven at, relative to the traction, which only scales it.
 *
 * @param bend The bend.
 * @return The speed with a lateral acceleration of 1 m/s^2 and the matching angular one.
 */
double relative_speed(const TurnBend& bend) {
    const double lateral = micras::robot_model.traction_acceleration(true);
    const double angular = micras::robot_model.traction_angular_acceleration(true);

    return std::min(std::sqrt(1.0 / bend.curvature), std::sqrt(angular / lateral / bend.sharpness));
}

/**
 * @brief Add every straight before two bends that closes the turn on its exit node.
 *
 * @param frame Where the turn starts and ends.
 * @param design The bends, whose straight before them is filled in.
 * @param candidates The list to add to.
 */
void add_closures(const Frame& frame, TwoBendDesign design, std::vector<Candidate>& candidates) {
    const TurnBend first = TurnTable::make_bend(micras::robot_model, design.first_angle, design.first_curvature);
    const TurnBend second = TurnTable::make_bend(micras::robot_model, design.second_angle, design.second_curvature);
    const auto     first_end = first.sample<double>(1.0e3);
    const auto     second_end = second.sample<double>(1.0e3);

    const double middle = frame.entry + first.angle;
    const double last = middle + second.angle;
    const double determinant = std::sin(static_cast<double>(second.angle));
    const double speed = std::min(relative_speed(first), relative_speed(second));
    const double curve = first.length() + second.length();

    const double bends_x = std::cos(frame.entry) * first_end.x - std::sin(frame.entry) * first_end.y +
                           std::cos(middle) * second_end.x - std::sin(middle) * second_end.y;
    const double bends_y = std::sin(frame.entry) * first_end.x + std::cos(frame.entry) * first_end.y +
                           std::sin(middle) * second_end.x + std::cos(middle) * second_end.y;

    for (int k = 0; k < number_of_pres; k++) {
        const double pre = pre_step * k;
        const double remaining_x = frame.exit_x - pre * std::cos(frame.entry) - bends_x;
        const double remaining_y = frame.exit_y - pre * std::sin(frame.entry) - bends_y;

        const double straight = (remaining_x * std::sin(last) - remaining_y * std::cos(last)) / determinant;
        const double post = (std::cos(middle) * remaining_y - std::sin(middle) * remaining_x) / determinant;

        if (straight >= 0.0 and post >= 0.0) {
            design.pre = static_cast<float>(pre);
            candidates.push_back({.time = (pre + curve + straight + post) / speed, .design = design});
        }
    }
}

/**
 * @brief List every design on the grid that ends on the exit node of a turn.
 *
 * @param turn The turn.
 * @return The designs, with the time each takes.
 */
std::vector<Candidate> list_candidates(TurnId turn) {
    const micras::nav::TurnPrimitive& primitive = micras::nav::get_primitive(turn);
    const double                      half_cell = micras::robot_model.maze.cell_size / 2.0;
    const Frame                       frame{
        .entry = primitive.diagonal_entry ? std::numbers::pi / 4.0 : 0.0,
        .exit_x = primitive.exit.x * half_cell,
        .exit_y = primitive.exit.y * half_cell,
    };

    std::vector<double> totals{primitive.rotation * 45.0};

    if (primitive.rotation != 0) {
        totals.push_back((primitive.rotation - 8) * 45.0);
    }

    const auto             steps = static_cast<int>(std::lround(2.0 * max_angle / angle_step));
    std::vector<Candidate> candidates;

    for (const double total : totals) {
        for (int step = 0; step <= steps; step++) {
            const double first_angle = -max_angle + angle_step * step;
            const double second_angle = total - first_angle;

            if (std::abs(first_angle) < min_angle or std::abs(second_angle) < min_angle or
                std::abs(second_angle) > max_angle or std::abs(std::sin(to_radians(second_angle))) < 1.0e-6) {
                continue;
            }

            for (int first = 0; first < number_of_radii; first++) {
                for (int second = 0; second < number_of_radii; second++) {
                    const TwoBendDesign bends{
                        .first_angle = static_cast<float>(to_radians(first_angle)),
                        .first_curvature = static_cast<float>(curvature_of(first)),
                        .second_angle = static_cast<float>(to_radians(second_angle)),
                        .second_curvature = static_cast<float>(curvature_of(second)),
                        .pre = 0.0F,
                    };

                    add_closures(frame, bends, candidates);
                }
            }
        }
    }

    std::ranges::sort(candidates, {}, &Candidate::time);

    return candidates;
}

/**
 * @brief Find the fastest design of a turn that fits.
 *
 * @param turn The turn.
 * @param margin The clearance asked of the turn.
 * @return The design, with no curvature if nothing on the grid fits.
 */
Result design(TurnId turn, float margin) {
    Result result{};

    for (const Candidate& candidate : list_candidates(turn)) {
        result.tried++;

        const auto shape = TurnTable::check(micras::robot_model, margin, turn, candidate.design);

        if (shape.valid) {
            result.design = candidate.design;
            result.speed = std::min(
                std::sqrt(micras::robot_model.traction_acceleration(true) / shape.curvature),
                std::sqrt(micras::robot_model.traction_angular_acceleration(true) / shape.sharpness)
            );
            result.time = shape.total_length() / result.speed;
            break;
        }
    }

    return result;
}

/**
 * @brief Write a number as a float literal.
 *
 * @param value The number.
 * @return The literal, with enough digits to read back the same float.
 */
std::string literal(float value) {
    std::string text = std::format("{:.9g}", value);

    if (text.find_first_of(".e") == std::string::npos) {
        text += ".0";
    }

    return text + "F";
}

/**
 * @brief Write the designs of one margin as a C++ array.
 *
 * @param name The name of the array.
 * @param margin The margin, for the comment.
 * @param results What was found for each turn.
 * @return The declaration.
 */
std::string to_array(const char* name, float margin, const std::vector<Result>& results) {
    std::string text = std::format(
        "/**\n * @brief Designs of the turns of two bends, from nav::first_two_bend_turn on, with a margin of {:.0f} "
        "mm.\n"
        " */\nconstexpr std::array<nav::TwoBendDesign, nav::number_of_two_bend_turns> {}{{{{\n",
        1000.0F * margin, name
    );

    for (std::size_t i = 0; i < results.size(); i++) {
        const TwoBendDesign& found = results.at(i).design;

        text += std::format(
            "    {{.first_angle = {},\n     .first_curvature = {},\n     .second_angle = {},\n"
            "     .second_curvature = {},\n     .pre = {}}},  // {}\n",
            literal(found.first_angle), literal(found.first_curvature), literal(found.second_angle),
            literal(found.second_curvature), literal(found.pre), names.at(i)
        );
    }

    return text + "}};\n";
}

/**
 * @brief Print what was found for one margin.
 *
 * @param margin The margin.
 * @param results What was found for each turn.
 */
void report(float margin, const std::vector<Result>& results) {
    std::cerr << std::format("margin {:.0f} mm, speed with the fan at full traction\n", 1000.0F * margin);

    for (std::size_t i = 0; i < results.size(); i++) {
        const Result&        result = results.at(i);
        const TwoBendDesign& found = result.design;

        if (found.first_curvature <= 0.0F) {
            std::cerr << std::format("  {:7} no fit ({} tried)\n", names.at(i), result.tried);
            continue;
        }

        std::cerr << std::format(
            "  {:7} {:4.0f}/{:4.0f} deg  R {:3.0f}/{:3.0f} mm  pre {:3.0f} mm  {:.2f} m/s  {:.3f} s  ({} tried)\n",
            names.at(i), found.first_angle * 180.0 / std::numbers::pi, found.second_angle * 180.0 / std::numbers::pi,
            1000.0 / found.first_curvature, 1000.0 / found.second_curvature, 1000.0 * found.pre, result.speed,
            result.time, result.tried
        );
    }
}
}  // namespace

int main() {
    const std::array<float, 2> margins{micras::turn_margin, micras::risky_turn_margin};

    std::vector<std::future<Result>> futures;

    for (const float margin : margins) {
        for (uint8_t i = 0; i < micras::nav::number_of_two_bend_turns; i++) {
            const auto turn = static_cast<TurnId>(micras::nav::first_two_bend_turn + i);
            futures.push_back(std::async(std::launch::async, design, turn, margin));
        }
    }

    std::vector<std::vector<Result>> results(margins.size());

    for (std::size_t i = 0; i < futures.size(); i++) {
        results.at(i / micras::nav::number_of_two_bend_turns).push_back(futures.at(i).get());
    }

    for (std::size_t i = 0; i < margins.size(); i++) {
        report(margins.at(i), results.at(i));
    }

    std::cout << "/**\n * @file\n *\n * @brief Designs of the turns of two bends, for each margin.\n *\n"
                 " * @note Written by the turn designer of the simulator (`just micras turn-designs`), which searches\n"
                 " * for the fastest two bends that clear the walls. Do not edit by hand: the build checks every "
                 "design\n"
                 " * against the robot and the maze, and a change to either that makes one no longer fit stops it, "
                 "which\n"
                 " * is when the designer has to run again. A turn nothing fits is written with no curvature, which "
                 "also\n"
                 " * stops the build.\n"
                 " */\n\n"
                 "#ifndef MICRAS_TWO_BEND_TURNS_HPP\n#define MICRAS_TWO_BEND_TURNS_HPP\n\n#include <array>\n\n"
                 "#include \"micras/nav/lattice.hpp\"\n#include \"micras/nav/turn_table.hpp\"\n\nnamespace micras {\n"
              << to_array("two_bend_designs", margins.at(0), results.at(0)) << "\n"
              << to_array("risky_two_bend_designs", margins.at(1), results.at(1))
              << "}  // namespace micras\n\n#endif  // MICRAS_TWO_BEND_TURNS_HPP\n";

    return 0;
}
