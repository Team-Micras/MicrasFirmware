/**
 * @file
 */

#ifndef MICRAS_CONSTANTS_HPP
#define MICRAS_CONSTANTS_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>

#include "maze_config.hpp"
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
#include "micras/nav/wall_model.hpp"
#include "robot.hpp"
#include "turn_margins.hpp"

namespace micras {
/*****************************************
 * Constants
 *****************************************/

constexpr uint32_t loop_time_us{100};
constexpr uint8_t  max_variables{128};

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
 * @note The tires cannot transmit more than the traction, but the accelerometer sees more than
 * what they transmit: its noise, the centripetal and tangential acceleration of the IMU, which is
 * not on the axis of rotation, and the jolt of the feedback correcting a sudden error. On the robot,
 * with the 0.54 the tires hold, such a correction went past 15 m/s^2 without touching anything,
 * while hitting a wall at the speed of a run stops it in a few millimeters, well above 25 m/s^2.
 */
constexpr float crash_acceleration{25.0F};

/**
 * @brief Speed the fan runs at, in percent of the battery.
 *
 * @note Half of a charged pack, about 6.2 V, where the MicrasHardware fan study puts the fan's 4 N
 * with its skirt. At full speed it would reach 14 N and 17 W, and its winding would pass 140 deg C
 * in five minutes.
 */
constexpr float fan_speed{50.0F};

/**
 * @brief Distance from the back edge of the start cell to the axle, with the robot against the wall,
 * and how much farther the return parks it.
 *
 * @note Backing into its place, the robot stops a few millimeters past where it aims.
 */
///@{
constexpr float start_offset{robot_model.chassis.rear_length + robot_model.maze.wall_thickness / 2.0F};
constexpr float park_clearance{0.008F};
///@}

/**
 * @brief Rate at which the control loop runs, and therefore the rate at which every filter driven
 * by it is sampled.
 *
 * @note Derived from the loop period rather than written twice: a filter designed for a sampling
 * rate it is not sampled at is a filter with the wrong cutoff.
 *
 * @note The loop runs at 10 kHz, faster than the 8 kHz of its fastest sensor, the inertial
 * measurement unit, which it reads once per iteration. At the same rate as the sensor the reads
 * drift in and out of phase with its samples over tens of milliseconds, and a sample that falls
 * inside a read is lost, since the sensor holds its output registers while one is in progress: the
 * samples stopped for up to 100 ms at a time on the robot. At 10 kHz the two beat at 2 kHz, so a lost
 * sample is followed by a new one within two iterations.
 */
constexpr float loop_frequency{1.0e6F / static_cast<float>(loop_time_us)};

/**
 * @brief Period of the control loop in seconds.
 */
constexpr float loop_time{static_cast<float>(loop_time_us) / 1.0e6F};

/**
 * @brief Rate at which the wall sensors produce a reading, which is the rate their filters run at.
 *
 * @note One reading per frame of the emitters, which take turns one at a time and leave one end of
 * the emitter timer dark: five ends of 200 us, 1 ms (see wall_sensors_config). The wall sensors check
 * this value against the registers of the timer when they start.
 */
constexpr float wall_sensors_frequency{1.0e6F / (5.0F * 200.0F)};

/**
 * @brief Number of consecutive iterations over the crash acceleration that count as a crash.
 *
 * @note A single sample over the threshold is what a bump in the floor looks like, so the
 * acceleration has to stay there for 5 ms.
 */
constexpr auto crash_debounce{static_cast<uint8_t>(0.005F * loop_frequency)};

/**
 * @brief Number of consecutive iterations with the motors saturated that count as the robot being
 * stuck.
 *
 * @note It is 100 ms of them. A stalled motor takes the whole supply, which is several times what
 * its winding is rated for, and no planned motion saturates the motors at all, so a run never
 * comes close.
 */
constexpr auto saturation_timeout{static_cast<uint16_t>(0.1F * loop_frequency)};

/**
 * @brief Number of consecutive iterations without a new sample of the inertial measurement unit
 * that count as it being lost.
 *
 * @note It is 5 ms of them. The unit samples at about the rate of the loop, so an iteration without
 * a sample happens, but never several in a row. It is counted in iterations, since the unit is read
 * once per iteration.
 */
constexpr auto imu_timeout{static_cast<uint16_t>(0.005F * loop_frequency)};

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
 * of the watchdog. An edge costs about the same whatever the turn.
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
 * @note An edge costs about the same whatever the turn, so a budget in edges bounds the time of an
 * iteration. A smaller budget delays the answers of the planner, which the search waits for.
 */
constexpr uint32_t search_edges_per_iteration{16};

/**
 * @brief Time without a control loop iteration that resets the microcontroller.
 *
 * @note A reset stops the program that hung, but it does not by itself switch the drivers off: the
 * pins of the microcontroller float from the reset until they are configured again, so the state
 * of the drivers in between depends on the pull resistors of the board.
 *
 * @note It is a time and not a number of iterations, since what it bounds is how far the robot
 * travels with nobody driving it, and it has to stay above the longest iteration there is.
 */
constexpr uint32_t watchdog_timeout_ms{10};

/**
 * @brief Watchdog timeout used around an operation that stalls the control loop while the robot is
 * stopped.
 *
 * @note Erasing a flash sector stalls the core for around 2 s, and up to 4 s in the worst case,
 * since the flash cannot be read while it is being erased. It also covers the construction of the
 * robot, where the proxies wait for their chips, and the planning of a fast run, whose search is
 * bounded per iteration but whose choice among the candidate routes is done in one. It is close to
 * the longest the watchdog allows, 32 s at its largest prescaler: the robot is stopped through all
 * of these, so a longer window costs nothing, and the watchdog reset the robot during saves within
 * 8 s.
 */
constexpr uint32_t stopped_watchdog_timeout_ms{30000};

/**
 * @brief Cutoff frequencies of the sensor filters, in hertz.
 *
 * @note The wall sensors are filtered twice: fast for everything that is a position, slow for
 * deciding whether there is a wall at all. The slow cutoff and the others are close to what the
 * firmware was actually running before the sampling rate and the bilinear prewarping were
 * corrected, when the loop ran at about 500 Hz: 3.98 Hz and 7.95 Hz. None of them were tuned
 * against a working robot; they are starting points, not results.
 */
///@{
constexpr float sensor_filter_cutoff{4.0F};
constexpr float torque_filter_cutoff{8.0F};
constexpr float wall_fast_filter_cutoff{50.0F};
constexpr float wall_slow_filter_cutoff{4.0F};
///@}

/**
 * @brief Longest distance the wall sensors are trusted at, in meters.
 */
constexpr float wall_sensors_range{0.25F};

/**
 * @brief Index of each wall sensor in the readings of the wall sensors.
 */
struct WallSensorsIndex {
    /**
     * @brief Index of the sensor named after where it looks.
     */
    ///@{
    uint8_t left_front{};
    uint8_t left{};
    uint8_t right{};
    uint8_t right_front{};
    ///@}
};

/**
 * @brief Where each wall sensor of the robot is in the readings of the wall sensors.
 */
constexpr WallSensorsIndex wall_sensors_index{
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

/**
 * @brief Command of each wheel, in percent of the supply.
 */
struct WheelCommand {
    float left;
    float right;
};

/**
 * @brief Steps of the check of the polarity, and how long each one is driven, in seconds.
 *
 * @note Each wheel forward and then backward, one wheel at a time, at commands from the lowest to
 * the highest of polarity_commands, with a rest after each step. The steps are fine enough to tell
 * where each wheel breaks away, which differs between the wheels and the directions. On a stand the
 * sweep shows the direction of each wheel and how the speed of a free wheel grows with the command;
 * on the floor every step turns the robot around the wheel that stands still, and the breakaway is
 * the one under the weight of the robot, which needs about 30 cm of free floor around it.
 */
///@{
constexpr std::array<float, 9> polarity_commands{6.0F, 9.0F, 12.0F, 15.0F, 18.0F, 21.0F, 24.0F, 27.0F, 30.0F};
constexpr float                polarity_step_duration{0.4F};

constexpr auto polarity_steps{[] {
    std::array<WheelCommand, 2 * 2 * 2 * polarity_commands.size()> steps{};
    std::size_t                                                    step = 0;

    for (const bool left : {true, false}) {
        for (const float sign : {1.0F, -1.0F}) {
            for (const float command : polarity_commands) {
                steps.at(step) = left ? WheelCommand{.left = sign * command, .right = 0.0F} :
                                        WheelCommand{.left = 0.0F, .right = sign * command};
                step += 2;
            }
        }
    }

    return steps;
}()};

///@}

/**
 * @brief Ranges a measured calibration has to fall in to be used, and the time the wall sensors are
 * left to settle before their offsets are measured, in seconds.
 *
 * @note A spread over the maximum means the robot moved, or something was in front of a sensor,
 * while it was being calibrated, and the result is not kept. An offset is a reading of nearly
 * nothing, whose standard deviation is taken against a bound of its own instead: a lamp flickering
 * at twice the mains frequency spreads the readings of the sensor that sees the most of it by about
 * 0.002 of the full scale, which the mean of the calibration averages out. It can
 * be slightly negative: each emitter's current disturbs the supply its receiver shares, which reads
 * the receiver's own lit scan a little below its dark one even with no light reaching it.
 */
///@{
constexpr float min_wall_offset{-0.05F};
constexpr float max_wall_offset{0.5F};
constexpr float min_gyroscope_scale{0.9F};
constexpr float max_gyroscope_scale{1.1F};
constexpr float max_calibration_spread{0.05F};
constexpr float max_offset_deviation{0.005F};
constexpr float offset_settle_time{0.1F};

///@}

/**
 * @brief Modes of the check of the crosstalk, and how long each one is lit, in seconds.
 *
 * @note Mode 0 has every emitter off, mode i + 1 publishes the light of the emitter of sensor i in
 * every receiver, and the last mode the readings as they are. A reading settles in a few
 * milliseconds, so most of each mode is steady.
 */
///@{
constexpr uint8_t crosstalk_modes{nav::number_of_wall_sensors + 2};
constexpr float   crosstalk_mode_duration{1.0F};

///@}

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
 * @brief Fraction of the available traction a run asks for, without and with the boost switch, and
 * the same with the fan running.
 *
 * @note In a turn the tires slide sideways in proportion to the grip they are asked for, which the
 * pose estimate only predicts. Boost stops at 0.7 of the traction for that reason: at 0.75 the risky
 * turns slide the robot into the walls. On the robot, a turn planned at 0.4 g peaked 45 % above
 * its plan while the controller corrected, and slid into the wall at 0.58 g: the normal runs ask for
 * 0.3 g of the 0.54 the tires hold without the fan, which leaves that peak below it. The first fast
 * runs on the robot also stop at 1 m/s.
 *
 * @note Simulated in the home maze with the 20 mm margin of the turns, 0.6 keeps 16 mm from the
 * walls under every disturbance tried, and 0.65 only 5 mm: the wheels slip as they speed up on the
 * first straight, and the edge that ends it corrects only 3 mm of the 15 mm that leaves. At 0.75
 * every run hits the first turn. Boost stops at 0.65 for that reason.
 *
 * @note With the simulated tires holding 0.46, which is what the fast runs on the robot slip like,
 * and the robot carrying its fan, 0.6 still keeps 12 mm from the walls on tires that hold 0.42. With
 * the fan running a share of the traction comes from the downforce, which loads the tires without
 * the weight that has to be sped up. With the 1.47 N the fan makes with its skirt, 0.5 keeps 16 mm
 * in the race maze with only 1 N on tires that hold 0.42, where 0.6 hits a wall and slid the robot
 * into one in its first fan run, and takes the race maze in 1.71 s against 2.42 s without the fan,
 * 1.40 s on the racing line. Boost asks for 0.55 of it, which keeps 4 mm in the same case.
 */
///@{
constexpr float normal_utilization{0.6F};
constexpr float boost_utilization{0.65F};
constexpr float fan_utilization{0.5F};
constexpr float fan_boost_utilization{0.55F};

///@}

/**
 * @brief Top speed of a fast run, in m/s.
 */
constexpr float run_max_speed{1.0F};

/**
 * @brief Make the profile of a fast run from the switches.
 *
 * @note With the racing line, the risky switch also optimizes a line through the route of the risky
 * turns, which comes as close as they do, and the robot drives it only if it is faster than the
 * line through the route of the normal turns.
 *
 * @param racing_line Whether to drive the racing line through the cells of the route instead.
 * @param boost Whether to ask for more of the available traction.
 * @param risky Whether to use the turns designed with the smaller margin.
 * @param fan Whether the fan runs, which adds its downforce to the traction.
 * @return The profile of the run.
 */
constexpr nav::RunProfile make_run_profile(bool racing_line, bool boost, bool risky, bool fan) {
    float utilization = boost ? boost_utilization : normal_utilization;

    if (fan) {
        utilization = boost ? fan_boost_utilization : fan_utilization;
    }

    return {
        .racing_line = racing_line,
        .fan = fan,
        .risky = risky,
        .utilization = utilization,
        .max_speed = run_max_speed,
    };
}

/**
 * @brief Profile of the search runs, which is where the search speed is set.
 *
 * @note 0.3 g, 0.55 of the traction without the fan, up to 0.3 m/s, the speed of the first searches
 * on the robot. Braking for a front wall is what limits it: past half of the traction, a front wall
 * that corrects the pose late asks for more braking than the tires give.
 */
constexpr nav::RunProfile search_profile{
    .racing_line = false,
    .fan = false,
    .risky = false,
    .utilization = 0.55F,
    .max_speed = 0.3F,
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

static_assert(
    nav::Maze::contains(maze_start.position) and not maze_goal.empty() and
        std::ranges::all_of(maze_goal, [](const nav::GridPoint& cell) { return nav::Maze::contains(cell); }),
    "the start and the goal have to be inside the maze"
);

/**
 * @brief Fraction of the motor supply kept for the feedback, which the planned motions do not use.
 */
constexpr float voltage_reserve{0.15F};

/**
 * @brief Configuration of the dynamics, with the shape of every turn.
 *
 * @note Defined in config/dynamics_config.cpp, the one source that designs the turns.
 */
extern const nav::Dynamics::Config dynamics_config;

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
 * @note Ranges only correct the pose out to 120 mm. The beam of an emitter is 11.8 mm above the
 * floor and a few degrees wide, so farther out part of it lands on the floor before the wall and
 * the reading comes out long. They only correct it while the beam meets the wall within 55 deg of
 * its perpendicular: turning the robot by hand in a cell, the sensors read within a few
 * millimeters up to there, and 20 to 60 mm short at 60 to 80 deg.
 *
 * @note An edge moves the pose by at most 3 mm. At 3 m/s an edge timed a millisecond off is 3 mm
 * off, and a jump of 8 mm made the controller ask the motors for their whole supply at once.
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
    .max_incidence = 55.0F * std::numbers::pi_v<float> / 180.0F,
    .rest_window = 0.1F,
    .edge_deviation = 0.004F,
    .edge_window = 0.025F,
    .edge_speed = 0.1F,
    .edge_range = 0.12F,
    .max_edge_correction = 0.003F,
    .range_tolerance = 0.015F,
    .relative_range_tolerance = 0.15F,
    .recovery_rejections = static_cast<uint16_t>(0.05F * wall_sensors_frequency),
    .recovery_deviation = 0.01F,
    .speed_window = speed_window,
};

/**
 * @brief Configuration of the controller.
 *
 * @note The natural frequencies are angular, in rad/s: the forward loop closes at 40 rad/s, 6.4 Hz.
 * At 50 rad/s a correction of the pose by a few millimeters at 3 m/s took the whole supply at once,
 * and the jolt read as a crash.
 */
const nav::Controller::Config controller_config{
    .model = robot_model,
    .linear =
        {
            .natural_frequency = 40.0F,
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
    .max_time_scale_acceleration = 10.0F,
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
            .trust = 0.03F,
            .scan_step = 0.002F,
            .length_weight = 10.0F,
            .convergence = 0.0002F,
            .lateral_share = 0.9F,
            .max_sweeps = 12,
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
    .park_clearance = park_clearance,
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
