/**
 * @file
 *
 * @brief Compares robot.toml, the physical description, with robot.hpp, the firmware's belief.
 *
 * @note Prints every quantity both files hold, field by field, with the relative
 *       difference. It shows the differences without forcing them equal: robot.toml
 *       is what the simulator builds, robot.hpp is what the firmware plans with, and
 *       a firmware whose belief is wrong is one of the things a run should expose.
 *       Where robot.toml splits a quantity the firmware lumps, such as the mass of
 *       the chassis and the wheels, the report adds the parts up the way the
 *       firmware means them.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <format>
#include <iostream>
#include <numbers>
#include <string>

#include "micras/nav/robot_model.hpp"
#include "micras/sim/arenas/maze.hpp"
#include "micras/sim/robot/robot_description.hpp"
#include "robot.hpp"

namespace {
using micras::sim::RobotDescription;
}  // namespace

/**
 * @brief Print one row of the report.
 *
 * @param quantity What is compared.
 * @param described Value in robot.toml, or what it adds up to.
 * @param believed Value in robot.hpp.
 * @param unit Unit both values are printed in.
 * @param scale Factor from SI to that unit.
 */
static void
    row(const std::string& quantity, double described, float believed, const std::string& unit, double scale = 1.0) {
    const auto   belief = static_cast<double>(believed);
    const double difference = belief == 0.0 ? 0.0 : 100.0 * (described - belief) / std::abs(belief);
    std::cout << std::format(
        "{:34}  {:>12.5g}  {:>12.5g}  {:8}  {:>+8.1f} %\n", quantity, described * scale, belief * scale, unit,
        difference
    );
}

/**
 * @brief Yaw inertia of the whole robot about the axle midpoint, wheels included.
 *
 * @param robot The description.
 * @return Inertia in kg m^2.
 */
static double yaw_inertia(const RobotDescription& robot) {
    const auto&  wheels = robot.wheels;
    const double offset = wheels.track / 2.0;
    const double transverse = wheels.mass * (3 * wheels.radius * wheels.radius + wheels.width * wheels.width) / 12;
    const double chassis_offset = std::hypot(robot.chassis.center_of_mass.at(0), robot.chassis.center_of_mass.at(1));

    return robot.chassis.inertia.at(2) + robot.chassis.mass * chassis_offset * chassis_offset +
           2.0 * (transverse + wheels.mass * offset * offset);
}

static int run() {
    const RobotDescription robot = RobotDescription::load(std::filesystem::path{MICRAS_TARGET_DIR} / "robot.toml");
    const micras::nav::RobotModel& model = micras::robot_model;
    const micras::sim::MazeConfig  maze{};

    double front = 0.0;
    double rear = 0.0;
    double half_width = robot.wheels.track / 2.0 + robot.wheels.width / 2.0;

    for (const auto& corner : robot.chassis.outline) {
        front = std::max(front, corner.at(0));
        rear = std::max(rear, -corner.at(0));
        half_width = std::max(half_width, std::abs(corner.at(1)));
    }

    std::cout << std::format(
        "{:34}  {:>12}  {:>12}  {:8}  {:>10}\n", "quantity", "robot.toml", "robot.hpp", "unit", "difference"
    );

    row("maze cell size", maze.cell_size, model.maze.cell_size, "mm", 1e3);
    row("maze wall thickness", maze.wall_thickness, model.maze.wall_thickness, "mm", 1e3);

    row("mass, chassis and wheels", robot.chassis.mass + 2.0 * robot.wheels.mass, model.chassis.mass, "g", 1e3);
    row("yaw inertia, chassis and wheels", yaw_inertia(robot), model.chassis.yaw_inertia, "g cm^2", 1e7);
    row("wheel radius", robot.wheels.radius, model.chassis.wheel_radius, "mm", 1e3);
    row("track width", robot.wheels.track, model.chassis.track_width, "mm", 1e3);
    row("half width, board or tires", half_width, model.chassis.half_width, "mm", 1e3);
    row("front length, board", front, model.chassis.front_length, "mm", 1e3);
    row("rear length, board", rear, model.chassis.rear_length, "mm", 1e3);

    row("tire friction coefficient", robot.wheels.friction, model.traction.friction_coefficient, "");
    row("fan downforce", robot.fan.max_downforce, model.traction.fan_downforce, "N");
    row("fan offset ahead of the axle", robot.fan.position.at(0), model.traction.fan_offset, "mm", 1e3);

    const auto& drive = robot.drive;
    row("torque constant", drive.torque_constant, model.drive.torque_constant, "mN m/A", 1e3);
    row("resistance, winding and bridge", drive.resistance(), model.drive.resistance, "ohm");
    row("gear ratio", drive.gear_ratio, model.drive.gear_ratio, "");
    row("supply voltage", drive.supply_voltage, model.drive.supply_voltage, "V");
    row("static friction voltage", drive.no_load_current * drive.resistance(), model.drive.static_friction_voltage,
        "V");

    for (std::size_t index = 0; index < robot.wall_sensors.sensors.size(); index++) {
        const auto&       sensor = robot.wall_sensors.sensors.at(index);
        const auto&       belief = model.wall_sensors.at(index);
        const std::string name = sensor.name + " sensor";

        row(name + " x, lens midpoint", (sensor.emitter.at(0) + sensor.receiver.at(0)) / 2.0, belief.position.x, "mm",
            1e3);
        row(name + " y, lens midpoint", (sensor.emitter.at(1) + sensor.receiver.at(1)) / 2.0, belief.position.y, "mm",
            1e3);
        row(name + " angle", sensor.yaw, belief.angle, "deg", 180.0 / std::numbers::pi);
    }

    row("emitter half angle", robot.wall_sensors.emitter_half_angle, model.wall_sensors.at(0).half_angle, "deg",
        180.0 / std::numbers::pi);
    row("gyroscope noise density", robot.imu.gyro_noise_density, model.noise.gyroscope, "mrad/s/rtHz", 1e3);
    row("wheel angle resolution", 2.0 * std::numbers::pi / robot.encoders.counts_per_revolution,
        model.noise.wheel_angle, "mrad", 1e3);

    return 0;
}

int main() {
    try {
        return run();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
