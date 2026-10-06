/**
 * @file
 *
 * @brief Derives each wall sensor's gain from the firmware's last calibration.
 *
 * @note Places the robot where the firmware's calibration is taken, centered in a
 *       cell: in a corridor for the sensors that look at the side walls, facing
 *       a wall for the ones that look forward, as the firmware's two calibration
 *       steps do. It fires every emitter and compares the simulated lit minus
 *       dark reading of each sensor with the reference reading target.hpp
 *       records from the robot. The simulation runs with the gains already in
 *       robot.toml, and each is corrected by the ratio of the two readings, so a
 *       calibrated file prints its own gains back.
 *
 * @note With --sweep it moves the robot instead: toward and away from the wall
 *       ahead for the front sensors, across the corridor for the side ones. At
 *       each place it prints the range along each sensor's axis and the
 *       distance the firmware makes of the simulated reading, with its model of
 *       the receiver's offset and half angle.
 */

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <format>
#include <iostream>
#include <numbers>
#include <span>
#include <string_view>
#include <vector>

#include <mujoco/mjtype.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

#include "micras/sim/arenas/maze.hpp"
#include "micras/sim/core/clock.hpp"
#include "micras/sim/core/mujoco_world.hpp"
#include "micras/sim/core/span_at.hpp"
#include "micras/sim/devices/wall_sensors.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "micras/sim/robot/robot_model.hpp"
#include "target.hpp"

namespace {
using micras::sim::Clock;
using micras::sim::Maze;
using micras::sim::MazeConfig;
using micras::sim::MujocoWorld;
using micras::sim::robot_mjcf;
using micras::sim::RobotDescription;
using micras::sim::WallSensorDescription;
using micras::sim::WallSensors;

/**
 * @brief A corridor three cells long: the middle cell and the ones around it have walls on both sides.
 */
constexpr std::string_view corridor_maze{"o---o---o---o\n"
                                         "|   |   |   |\n"
                                         "o   o   o   o\n"
                                         "|   |   |   |\n"
                                         "o   o   o   o\n"
                                         "|   |   |   |\n"
                                         "o---o---o---o\n"};

/**
 * @brief A dead end: the middle cell has walls on both sides and in front.
 */
constexpr std::string_view facing_maze{"o---o---o---o\n"
                                       "|           |\n"
                                       "o   o---o   o\n"
                                       "|   |   |   |\n"
                                       "o   o   o   o\n"
                                       "|           |\n"
                                       "o---o---o---o\n"};

/**
 * @brief What every sensor reads from one place, and how far its axis runs to a wall.
 */
struct Sample {
    std::vector<double> readings;
    std::vector<double> ranges;
};
}  // namespace

/**
 * @brief Read every sensor with its own emitter lit, robot in the middle cell facing up.
 *
 * @param robot The robot.
 * @param drawing The maze.
 * @param across How far right of the cell center the robot is, in meters.
 * @param along How far ahead of the cell center the robot is, in meters.
 * @return Each sensor's lit minus dark reading, as the firmware normalizes it, and its axis range.
 */
static Sample sample(const RobotDescription& robot, std::string_view drawing, double across = 0.0, double along = 0.0) {
    const MazeConfig config{};
    const Maze       maze = Maze::parse(drawing);

    MujocoWorld world;
    world.build(
        robot_mjcf(robot), robot.name, maze.mjcf(config), Maze::body_name,
        {.x = (1.5 * config.cell_size) + across, .y = (1.5 * config.cell_size) + along, .yaw = std::numbers::pi / 2}
    );
    world.reset();

    std::vector<uint32_t> counts(2 * robot.wall_sensors.sensors.size());
    WallSensors           sensors{
        world,
        {
            .name = "wall",
            .description = robot.wall_sensors,
            .scan_ticks = 1,
            .emitter_duty = [](std::size_t) { return 30.0F; },
            .write = [&counts](std::size_t index, uint32_t value) { counts.at(index) = value; },
            .finish_sequence = [] { },
            .reflectance =
                [&world, &config](int geom) {
                    const char* name = mj_id2name(world.model(), mjOBJ_GEOM, geom);
                    return Maze::reflectance(name == nullptr ? "" : name, config);
                },
            .schedule = {},
        },
        {.seed = 1, .ideal = true},
    };

    Clock clock = Clock::from_model(world.timestep(), micras::loop_time_us);
    clock.advance();
    sensors.sample(world, clock);
    clock.advance();
    sensors.sample(world, clock);

    const std::size_t             count = robot.wall_sensors.sensors.size();
    const auto                    sites = static_cast<std::size_t>(world.model()->nsite);
    const std::span<const mjtNum> site_positions(world.data()->site_xpos, 3 * sites);
    const std::span<const mjtNum> site_frames(world.data()->site_xmat, 9 * sites);
    Sample                        result;

    for (std::size_t sensor = 0; sensor < count; sensor++) {
        const WallSensorDescription& description = robot.wall_sensors.sensors.at(sensor);
        const std::size_t            own = description.group == 0 ? 0 : 1;
        const double                 lit = counts.at(own * count + sensor);
        const double                 dark = counts.at((1 - own) * count + sensor);
        result.readings.push_back((lit - dark) / robot.wall_sensors.adc_max_counts);

        const auto site = static_cast<std::size_t>(world.require_id(mjOBJ_SITE, description.name + "_emitter"));
        const std::span<const mjtNum> frame = site_frames.subspan(9 * site, 9);
        const std::array<mjtNum, 3>   axis{
            micras::sim::at(frame, 0), micras::sim::at(frame, 3), micras::sim::at(frame, 6)
        };
        std::array<mjtByte, mjNGROUP> groups{1, 1, 1, 1, 1, 1};
        groups.at(MujocoWorld::unseen_group) = 0;
        int geom = -1;
        result.ranges.push_back(mj_ray(
            world.model(), world.data(), site_positions.subspan(3 * site, 3).data(), axis.data(), groups.data(), true,
            -1, &geom, nullptr
        ));
    }

    return result;
}

/**
 * @brief How the firmware expects a reading to vary with the distance, up to its calibrated constant.
 *
 * @param distance Distance to the wall in meters.
 * @return The shape of the reading.
 */
static double firmware_shape(double distance) {
    const auto&  config = micras::wall_sensors_config;
    const double angle = std::atan(static_cast<double>(config.receiver_offset) / distance) /
                         static_cast<double>(config.receiver_half_angle);
    return std::exp2(-angle * angle) / (distance * distance);
}

/**
 * @brief The distance the firmware makes of a reading.
 *
 * @note Solves the firmware's model of the reading by bisection over the distances past the peak
 *       of the reading, which is what the firmware's table covers.
 *
 * @param sensor Index of the sensor.
 * @param reading Normalized reading.
 * @return Distance in meters.
 */
static double firmware_distance(std::size_t sensor, double reading) {
    const auto&  config = micras::wall_sensors_config;
    const auto   reference = static_cast<double>(config.reference_distances.at(sensor));
    const double target =
        reading / static_cast<double>(config.reference_readings.at(sensor)) * firmware_shape(reference);
    double low = 0.001;

    while (low < static_cast<double>(config.max_distance) and firmware_shape(low + 0.001) > firmware_shape(low)) {
        low += 0.001;
    }

    auto high = static_cast<double>(config.max_distance);

    if (firmware_shape(low) <= target) {
        return low;
    }

    if (firmware_shape(high) >= target) {
        return high;
    }

    for (int step = 0; step < 60; step++) {
        const double middle = (low + high) / 2.0;
        (firmware_shape(middle) > target ? low : high) = middle;
    }

    return (low + high) / 2.0;
}

/**
 * @brief Print how the firmware's distance follows the true range as the robot moves.
 *
 * @param robot The robot.
 */
static void sweep(const RobotDescription& robot) {
    std::cout << "front sensors, robot moved toward the wall ahead\n";
    std::cout << "  along_mm  sensor        range_mm  firmware_mm  error_mm\n";

    for (int along = -200; along <= 40; along += 10) {
        const Sample result = sample(robot, facing_maze, 0.0, along * 1e-3);

        for (std::size_t sensor = 0; sensor < robot.wall_sensors.sensors.size(); sensor++) {
            const WallSensorDescription& description = robot.wall_sensors.sensors.at(sensor);

            if (std::abs(std::sin(description.yaw)) > 0.1) {
                continue;
            }

            const double range = result.ranges.at(sensor) * 1e3;
            const double distance = firmware_distance(sensor, result.readings.at(sensor)) * 1e3;
            std::cout << std::format(
                "  {:8}  {:12}  {:8.1f}  {:11.1f}  {:8.1f}\n", along, description.name, range, distance,
                distance - range
            );
        }
    }

    std::cout << "side sensors, robot moved across the corridor\n";
    std::cout << "  right_mm  sensor        range_mm  firmware_mm  error_mm\n";

    for (int across = -30; across <= 30; across += 10) {
        const Sample result = sample(robot, corridor_maze, across * 1e-3, 0.0);

        for (std::size_t sensor = 0; sensor < robot.wall_sensors.sensors.size(); sensor++) {
            const WallSensorDescription& description = robot.wall_sensors.sensors.at(sensor);

            if (std::abs(std::sin(description.yaw)) <= 0.1) {
                continue;
            }

            const double range = result.ranges.at(sensor) * 1e3;
            const double distance = firmware_distance(sensor, result.readings.at(sensor)) * 1e3;
            std::cout << std::format(
                "  {:8}  {:12}  {:8.1f}  {:11.1f}  {:8.1f}\n", across, description.name, range, distance,
                distance - range
            );
        }
    }
}

static int run(int argc, char** argv) {
    const std::span<char*> arguments(argv, static_cast<std::size_t>(argc));
    const RobotDescription robot = RobotDescription::load(std::filesystem::path{MICRAS_TARGET_DIR} / "robot.toml");

    if (arguments.size() > 1 and std::string_view{micras::sim::at(arguments, 1)} == "--sweep") {
        sweep(robot);
        return 0;
    }

    const std::vector<double> corridor = sample(robot, corridor_maze).readings;
    const std::vector<double> facing = sample(robot, facing_maze).readings;

    std::cout << "sensor        placement  simulated  robot      gain\n";

    for (std::size_t sensor = 0; sensor < robot.wall_sensors.sensors.size(); sensor++) {
        const WallSensorDescription& description = robot.wall_sensors.sensors.at(sensor);
        const bool                   side = std::abs(std::sin(description.yaw)) > 0.1;
        const double                 simulated = side ? corridor.at(sensor) : facing.at(sensor);
        const auto reference = static_cast<double>(micras::wall_sensors_config.reference_readings.at(sensor));

        std::cout << std::format(
            "{:12}  {:9}  {:9.4f}  {:9.4f}  {:.4f}\n", description.name, side ? "corridor" : "facing", simulated,
            reference, description.gain * reference / simulated
        );
    }

    return 0;
}

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
