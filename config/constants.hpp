/**
 * @file
 */

#ifndef MICRAS_CONSTANTS_HPP
#define MICRAS_CONSTANTS_HPP

#include <array>
#include <cstdint>
#include <limits>
#include <numbers>

#include "micras/core/butterworth_filter.hpp"
#include "micras/core/types.hpp"
#include "micras/nav/controller.hpp"
#include "micras/nav/drive_identification.hpp"
#include "micras/nav/executor.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/gyroscope_calibration.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/mission.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/turn_table.hpp"
#include "micras/nav/wall_model.hpp"
#include "robot.hpp"
#include "two_bend_turns.hpp"

namespace micras {
/*****************************************
 * Constants
 *****************************************/

constexpr uint8_t  maze_width{16};
constexpr uint8_t  maze_height{16};
constexpr uint32_t loop_time_us{125};
constexpr uint8_t  max_variables{64};

/**
 * @brief Size of the buffer the radio receives into.
 *
 * @note Nothing stops the DMA from wrapping around it, so it has to hold everything that can
 * arrive between two control loop iterations. At 115200 baud that is a byte every 87 us, so this
 * covers more than four thousand iterations.
 */
constexpr uint16_t bluetooth_rx_buffer_size{512};

/**
 * @brief Size of the buffer the frames waiting to be sent are queued in.
 *
 * @note Whole frames only: a frame that does not fit is not queued at all, so this only has to
 * hold a few of the largest ones.
 */
constexpr uint16_t bluetooth_tx_buffer_size{4096};

/**
 * @brief Horizontal acceleration, in m/s^2, over which the robot has hit something.
 *
 * @note The tires cannot transmit more than the traction with the fan running, so anything above
 * it came from a wall. The margin covers the noise of the accelerometer and the centripetal and
 * tangential acceleration of the IMU, which is not on the axis of rotation. A fixed 35 m/s^2 was
 * below the 32 m/s^2 a fast run plans with the fan, plus the feedback, and stopped fast runs that
 * had touched nothing.
 */
constexpr float crash_acceleration{1.25F * robot_model.traction_acceleration(true)};
constexpr float fan_speed{100.0F};

/**
 * @brief Distance from the back edge of the start cell to the axle, with the robot against the wall.
 */
constexpr float start_offset{0.04F + robot_model.maze.wall_thickness / 2.0F};

/**
 * @brief Rate at which the control loop runs, and therefore the rate at which every filter driven
 * by it is sampled.
 *
 * @note Derived from the loop period rather than written twice: a filter designed for a sampling
 * rate it is not sampled at is a filter with the wrong cutoff.
 *
 * @note The period is the one of the fastest sensor, the 8 kHz of the inertial measurement unit,
 * since an iteration without a new sample of anything has nothing to compute. The next periods that
 * keep that property are 250 and 500 microseconds, with the data rate of the sensor following.
 */
constexpr float loop_frequency{1.0e6F / static_cast<float>(loop_time_us)};

/**
 * @brief Period of the control loop in seconds.
 */
constexpr float loop_time{static_cast<float>(loop_time_us) / 1.0e6F};

/**
 * @brief Rate at which the wall sensors produce a reading, which is the rate their filters run at.
 *
 * @note One reading per period of the emitter timer, which the peripheral configuration makes four
 * periods of the control loop. The wall sensors check this value against the registers of the timer
 * when they start.
 */
constexpr float wall_sensors_frequency{2000.0F};

/**
 * @brief Number of consecutive iterations over the crash acceleration that count as a crash.
 *
 * @note A single sample over the threshold is what a bump in the floor looks like, so the
 * acceleration has to stay there for 5 ms.
 */
constexpr auto crash_debounce{static_cast<uint8_t>(0.005F * loop_frequency)};

/**
 * @brief Number of iterations the speed of the wheels is measured over, which is 2 ms of them.
 *
 * @note One count of an encoder in one iteration would read as 34 mm/s. Over this window it is
 * 2 mm/s, for a delay of 1 ms.
 */
constexpr auto speed_window{static_cast<uint8_t>(0.002F * loop_frequency)};

/**
 * @brief Number of readings that have to agree, net of those that disagree, to decide a wall.
 *
 * @note It is 12 ms of them, which at the speed of a search is 5 mm of travel.
 */
constexpr auto wall_votes{static_cast<int8_t>(0.012F * wall_sensors_frequency)};

/**
 * @brief Number of edges the route planner tries per iteration while a fast run is planned.
 *
 * @note The robot is stopped then, so this only has to keep an iteration well inside the timeout
 * of the watchdog. It is about what 64 nodes of the planner with eight turns cost, and a node of
 * that planner cost some tens of microseconds.
 */
constexpr uint32_t plan_edges_per_iteration{512};

/**
 * @brief Number of samples the racing line advances by per iteration while it is optimized.
 *
 * @note The robot is stopped then too. Finding the bounds of a sample is the costliest work per
 * sample, at up to 31 checks of the outline of the robot.
 */
constexpr uint16_t line_samples_per_iteration{16};

/**
 * @brief Number of edges the route planner tries per iteration while the robot searches.
 *
 * @note A node of the planner with eight turns cost about 8 edges of this one on a PC, and was a good
 * part of what is left of an iteration. A node now tries three to four times as many turns, so a
 * budget in nodes would no longer bound the time of an iteration. The answers of the planner come
 * later with a smaller budget, and the search waits for them: on ten mazes it took 17 % longer with
 * 8 edges, 4 % with 16 and 2 % with 32.
 */
constexpr uint32_t search_edges_per_iteration{16};

/**
 * @brief Time without a control loop iteration that resets the microcontroller.
 *
 * @note A reset brings the driver enable pins and the PWM outputs back to their reset state, which
 * makes the watchdog the shutdown path for a hang or a fault handler that never returns.
 *
 * @note It is a time and not a number of iterations, since what it bounds is how far the robot
 * travels with nobody driving it, and it has to stay above the longest iteration there is.
 */
constexpr uint32_t watchdog_timeout_ms{10};

/**
 * @brief Watchdog timeout used around a flash erase.
 *
 * @note Erasing a sector stalls the core for around 2 s, and up to 4 s in the worst case, since the
 * flash cannot be read while it is being erased. The robot is stopped whenever this happens.
 */
constexpr uint32_t flash_watchdog_timeout_ms{8000};

/**
 * @brief Cutoff frequencies of the sensor filters, in hertz.
 *
 * @note The wall sensors are filtered twice: fast for everything that is a position, slow for
 * deciding whether there is a wall at all. These are starting points, not results.
 */
///@{
constexpr float sensor_filter_cutoff{7.64F};
constexpr float torque_filter_cutoff{15.27F};
constexpr float wall_fast_filter_cutoff{50.0F};
constexpr float wall_slow_filter_cutoff{7.64F};
///@}

/**
 * @brief Longest distance the wall sensors are trusted at, in meters.
 */
constexpr float wall_sensors_range{0.25F};

constexpr core::WallSensorsIndex wall_sensors_index{
    .left_front = 0,
    .left = 1,
    .right = 2,
    .right_front = 3,
};

static_assert(
    wall_sensors_index.left_front < nav::number_of_wall_sensors and
        wall_sensors_index.left < nav::number_of_wall_sensors and
        wall_sensors_index.right < nav::number_of_wall_sensors and
        wall_sensors_index.right_front < nav::number_of_wall_sensors and
        wall_sensors_index.left_front != wall_sensors_index.left and
        wall_sensors_index.left_front != wall_sensors_index.right and
        wall_sensors_index.left_front != wall_sensors_index.right_front and
        wall_sensors_index.left != wall_sensors_index.right and
        wall_sensors_index.left != wall_sensors_index.right_front and
        wall_sensors_index.right != wall_sensors_index.right_front,
    "every wall sensor needs an index of its own"
);

static_assert(watchdog_timeout_ms > 0, "the watchdog counts in milliseconds");
static_assert(
    2.0F * wall_fast_filter_cutoff < wall_sensors_frequency and 2.0F * sensor_filter_cutoff < loop_frequency and
        2.0F * torque_filter_cutoff < loop_frequency,
    "a filter cannot have its cutoff above half of the rate it is sampled at"
);

/*****************************************
 * Template Instantiations
 *****************************************/

namespace nav {
using Maze = TMaze<maze_width, maze_height>;
using Mission = TMission<maze_width, maze_height>;
}  // namespace nav

/*****************************************
 * Run profiles
 *****************************************/

/**
 * @brief Fraction of the available traction a run asks for, without and with the boost switch.
 *
 * @note In a turn the tires slide sideways in proportion to the grip they are asked for, and the
 * pose estimate does not see it. At three quarters of the traction a turn put the robot 10 to 20 mm
 * off its path; at 0.65 every run of ten mazes, with diagonals and the risky turns, stays clean.
 */
///@{
constexpr float normal_utilization{0.6F};
constexpr float boost_utilization{0.65F};
///@}

/**
 * @brief Distance kept between the outline of the robot and any obstacle when a turn is designed,
 * without and with the risky switch.
 */
///@{
constexpr float turn_margin{0.015F};
constexpr float risky_turn_margin{0.010F};
///@}

/**
 * @brief Shape of every turn, computed when the firmware is compiled.
 *
 * @note A turn that does not fit in the maze with the margin asked for stops the build here.
 */
///@{
constexpr nav::TurnTable::TwoBendShapes two_bend_shapes{
    nav::TurnTable::place(robot_model, turn_margin, two_bend_designs)
};
constexpr nav::TurnTable::TwoBendShapes risky_two_bend_shapes{
    nav::TurnTable::place(robot_model, risky_turn_margin, risky_two_bend_designs)
};
constexpr nav::TurnTable turn_table{robot_model, turn_margin, two_bend_shapes};
constexpr nav::TurnTable risky_turn_table{robot_model, risky_turn_margin, risky_two_bend_shapes};

static_assert(turn_table.is_valid(), "a turn does not fit in the maze with the normal margin");
static_assert(risky_turn_table.is_valid(), "a turn does not fit in the maze with the risky margin");
static_assert(
    nav::TurnTable::clears(robot_model, turn_margin, two_bend_designs),
    "a turn of two bends does not clear the walls with the normal margin: run the turn designer"
);
static_assert(
    nav::TurnTable::clears(robot_model, risky_turn_margin, risky_two_bend_designs),
    "a turn of two bends does not clear the walls with the risky margin: run the turn designer"
);

///@}

/**
 * @brief Make the profile of a fast run from the switches.
 *
 * @note Every route may use diagonals. With the racing line, the risky switch also optimizes a line
 * through the route of the risky turns, which comes as close as they do, and the robot drives it
 * only if it is faster than the line through the route of the normal turns.
 *
 * @param racing_line Whether to drive the racing line through the cells of the route instead.
 * @param boost Whether to ask for more of the available traction.
 * @param risky Whether to use the turns designed with the smaller margin.
 * @param fan Whether the fan runs, which adds its downforce to the traction.
 * @return The profile of the run.
 */
constexpr nav::RunProfile make_run_profile(bool racing_line, bool boost, bool risky, bool fan) {
    return {
        .racing_line = racing_line,
        .fan = fan,
        .risky = risky,
        .utilization = boost ? boost_utilization : normal_utilization,
        .max_speed = std::numeric_limits<float>::infinity(),
    };
}

/**
 * @brief Profile of the search runs, which is where the search speed is set.
 *
 * @note Half of the traction without the fan, up to 1 m/s. Braking for a front wall is what limits
 * it: past half, a front wall that corrects the pose late asks for more braking than the tires give.
 */
constexpr nav::RunProfile search_profile{
    .racing_line = false,
    .fan = false,
    .risky = false,
    .utilization = 0.5F,
    .max_speed = 1.0F,
};

/**
 * @brief Profiles the map has to be complete for before the search ends.
 *
 * @note Every combination of the boost and risky switches, with the fan running. The racing line
 * goes through the cells of the route the planner chooses, so it needs no more of the map. A
 * shorter list makes for a shorter search, at the price of a map that may hide the best route of
 * the profiles left out.
 */
constexpr std::array<nav::RunProfile, 4> map_profiles{{
    make_run_profile(false, false, false, true),
    make_run_profile(false, true, false, true),
    make_run_profile(false, false, true, true),
    make_run_profile(false, true, true, true),
}};

/*****************************************
 * Configurations
 *****************************************/

constexpr nav::GridPose maze_start{.position = {.x = 0, .y = 0}, .orientation = nav::Side::UP};

constexpr std::array<nav::GridPoint, 4> maze_goal{{
    {.x = maze_width / 2, .y = maze_height / 2},
    {.x = (maze_width - 1) / 2, .y = maze_height / 2},
    {.x = maze_width / 2, .y = (maze_height - 1) / 2},
    {.x = (maze_width - 1) / 2, .y = (maze_height - 1) / 2},
}};

static_assert(
    nav::Maze::contains(maze_start.position) and nav::Maze::contains(std::get<0>(maze_goal)) and
        nav::Maze::contains(std::get<3>(maze_goal)),
    "the start and the goal have to be inside the maze"
);

/**
 * @brief Fraction of the motor supply kept for the feedback, which the planned motions do not use.
 */
constexpr float voltage_reserve{0.15F};

const nav::Dynamics::Config dynamics_config{
    .model = robot_model,
    .turns = turn_table,
    .risky_turns = risky_turn_table,
    .max_linear_speed = 3.0F,
    .max_angular_speed = 12.0F,
    .voltage_reserve = voltage_reserve,
};

/**
 * @brief Configuration of the wall model.
 *
 * @note The minimum range sits past the distance at which the reading of a wall sensor peaks, about
 * 30 mm (see wall_sensors_config): closer than that a reading no longer tells one range from a
 * longer one, so it cannot correct anything.
 */
const nav::WallModel::Config wall_model_config{
    .model = robot_model,
    .min_range = 0.035F,
    .max_range = wall_sensors_range,
    .edge_margin = 0.005F,
    .confidence = 2.0F,
};

/**
 * @brief Configuration of the localizer.
 *
 * @note Ranges only correct the pose out to 120 mm. The beam of an emitter is 11.75 mm above the
 * floor and a few degrees wide, so farther out part of it lands on the floor before the wall and
 * the reading comes out long: 5 mm at 120 mm, 16 mm at 150 mm and 45 mm at 180 mm for the sensors
 * that look forward.
 *
 * @note The end of a side wall is seen 16 mm inside the wall, whichever way the robot crosses it and
 * at 0.4 to 2.6 m/s alike: the reading only changes once most of the spot has crossed the end. That
 * is 1.32 times the half length of the spot the wall model computes from the half angle of the
 * sensors, which is what the edge inset is.
 */
const nav::Localizer::Config localizer_config{
    .model = robot_model,
    .initial_position_deviation = 0.005F,
    .initial_orientation_deviation = 0.03F,
    .initial_bias_deviation = 0.02F,
    .gate = 9.0F,
    .stationary_gate = 25.0F,
    .max_position_correction = 0.002F,
    .max_orientation_correction = 0.01F,
    .max_angular_speed = 2.0F,
    .stationary_linear_speed = 0.005F,
    .stationary_angular_speed = 0.02F,
    .range_delay = core::ButterworthFilter::get_delay(wall_fast_filter_cutoff),
    .range_correlation = wall_sensors_frequency / (2.22F * wall_fast_filter_cutoff),
    .max_range = 0.12F,
    .rest_window = 0.1F,
    .use_edges = true,
    .edge_deviation = 0.004F,
    .edge_window = 0.025F,
    .edge_speed = 0.1F,
    .edge_range = 0.12F,
    .max_edge_correction = 0.01F,
    .range_tolerance = 0.015F,
    .relative_range_tolerance = 0.15F,
    .speed_window = speed_window,
};

const nav::Controller::Config controller_config{
    .model = robot_model,
    .linear =
        {
            .natural_frequency = 50.0F,
            .damping = 0.8F,
            .max_error = 0.03F,
        },
    .angular =
        {
            .natural_frequency = 60.0F,
            .damping = 0.8F,
            .max_error = 0.5F,
        },
    .steering_gain = 8.0F,
    .max_steering = 0.15F,
    .steering_blend_speed = 0.1F,
    .friction_speed = 0.02F,
    .voltage_reserve = voltage_reserve,
};

/**
 * @brief Configuration of the mission.
 *
 * @note A wall is only voted absent while it would be closer than 130 mm, where a range reads at
 * most 8 mm long (see localizer_config), well inside the tolerance; farther out a wall that is there
 * reads long enough to look missing. The observer keeps voting through the turns of the search, which
 * reach 8 rad/s: the side wall of the cell a turn ends in is only in sight during that turn, and a
 * robot that has not seen it has to stop and spin in the middle of the cell to go on.
 */
const nav::Mission::Config mission_config{
    .maze =
        {
            .start = maze_start,
            .goal = maze_goal,
        },
    .planner =
        {
            .start_distance = robot_model.maze.cell_size - start_offset,
        },
    .observer =
        {
            .tolerance = 0.015F,
            .relative_tolerance = 0.15F,
            .detection_range = 0.13F,
            .max_angular_speed = 12.0F,
            .range_delay = core::ButterworthFilter::get_delay(wall_fast_filter_cutoff),
            .votes_to_decide = wall_votes,
        },
    .racing_line =
        {
            .spacing = 0.01F,
            .margin = turn_margin,
            .least_margin = turn_margin - 0.002F,
            .risky_least_margin = risky_turn_margin - 0.002F,
            .trust = 0.03F,
            .scan_step = 0.002F,
            .length_weight = 10.0F,
            .convergence = 0.0002F,
            .lateral_share = 0.8F,
            .max_sweeps = 12,
            .smoothing = 1,
            .samples_per_step = line_samples_per_iteration,
        },
    .executor =
        {
            .capacity = 256,
            .settle_distance = 0.002F,
            .settle_angle = 0.02F,
            .settle_linear_speed = 0.01F,
            .settle_angular_speed = 0.05F,
            .settle_time = 0.05F,
        },
    .search_profile = search_profile,
    .map_profiles = map_profiles,
    .start_offset = start_offset,
    .stop_time = 0.1F,
    .attach_time = 0.5F,
    .look_time = 0.02F,
    .max_looks = 50,
    .commit_margin = 0.01F,
    .edges_per_iteration = search_edges_per_iteration,
};

/**
 * @brief Configuration of the identification of the drive train.
 *
 * @note The robot needs the distance limit of free floor ahead of it, and comes back to where it
 * started from.
 */
const nav::DriveIdentification::Config drive_identification_config{
    .model = robot_model,
    .ramp_rate = 5.0F,
    .breakaway_speed = 0.02F,
    .linear_command = 20.0F,
    .angular_command = 8.0F,
    .step_time = 0.4F,
    .rest_time = 0.5F,
    .max_distance = 0.8F,
};

/**
 * @brief Configuration of the calibration of the gyroscope scale.
 */
const nav::GyroscopeCalibration::Config gyroscope_calibration_config{
    .model = robot_model,
    .left_sensor = wall_sensors_index.left_front,
    .right_sensor = wall_sensors_index.right_front,
    .turns = 5.0F,
    .settle_time = 0.5F,
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP
