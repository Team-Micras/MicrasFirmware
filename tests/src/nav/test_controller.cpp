/**
 * @file
 */

#include <array>
#include <numbers>

#include "constants.hpp"
#include "micras/nav/controller.hpp"
#include "micras/nav/executor.hpp"
#include "micras/nav/lattice.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/state.hpp"
#include "micras/nav/velocity_planner.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/proxy/stopwatch.hpp"
#include "micras/proxy/tick.hpp"
#include "robot.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_reference_speed{};
static volatile float test_measured_speed{};
static volatile float test_along_error{};
static volatile float test_across_error{};
static volatile float test_orientation_error{};
static volatile float test_forward_feed_forward{};
static volatile float test_forward_feedback{};
static volatile float test_rotation_feed_forward{};
static volatile float test_rotation_feedback{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

/**
 * @brief Drive three cells straight ahead, turn around in place and come back, on every press of
 * the button, with no walls involved: what is left is the reference, the feed forward and the
 * feedback, which is what this is for looking at.
 */
int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Button             button{button_config};
    proxy::Locomotion         locomotion{locomotion_config};
    proxy::Imu                imu{imu_config};
    const proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    const proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};
    proxy::Tick               tick{tick_config};

    const nav::Dynamics dynamics{dynamics_config};
    nav::Localizer      localizer{localizer_config};
    nav::Controller     controller{controller_config};
    nav::Executor       executor{dynamics, mission_config.executor};

    const float       distance = 3.0F * robot_model.maze.cell_size;
    const nav::Pose   origin{.position = {.x = 0.0F, .y = 0.0F}, .orientation = 0.0F};
    const nav::Pose   far_end{.position = {.x = distance, .y = 0.0F}, .orientation = 0.0F};
    const nav::Pose   turned{.position = {.x = distance, .y = 0.0F}, .orientation = std::numbers::pi_v<float>};
    const nav::TurnId unused = nav::TurnId::SS90S;

    std::array<nav::Segment, 5> route{{
        {.kind = nav::SegmentKind::STRAIGHT,
         .turn = unused,
         .length = distance,
         .start_speed = 0.0F,
         .end_speed = 0.0F,
         .max_speed = 0.0F,
         .start = origin},
        {.kind = nav::SegmentKind::STOP,
         .turn = unused,
         .length = 0.2F,
         .start_speed = 0.0F,
         .end_speed = 0.0F,
         .max_speed = 0.0F,
         .start = far_end},
        {.kind = nav::SegmentKind::SPIN,
         .turn = unused,
         .length = std::numbers::pi_v<float>,
         .start_speed = 0.0F,
         .end_speed = 0.0F,
         .max_speed = 0.0F,
         .start = far_end},
        {.kind = nav::SegmentKind::STOP,
         .turn = unused,
         .length = 0.2F,
         .start_speed = 0.0F,
         .end_speed = 0.0F,
         .max_speed = 0.0F,
         .start = turned},
        {.kind = nav::SegmentKind::STRAIGHT,
         .turn = unused,
         .length = distance,
         .start_speed = 0.0F,
         .end_speed = 0.0F,
         .max_speed = 0.0F,
         .start = turned},
    }};

    nav::VelocityPlanner::plan(route, dynamics, search_profile, 0.0F, 0.0F);

    const auto measure = [&imu, &rotary_sensor_left, &rotary_sensor_right]() {
        return nav::Measurements{
            .left_wheel_angle = rotary_sensor_left.get_position(),
            .right_wheel_angle = rotary_sensor_right.get_position(),
            .angular_rate = imu.get_angular_velocity(proxy::Imu::Axis::Z),
            .acceleration =
                {.x = imu.get_linear_acceleration(proxy::Imu::Axis::Y),
                 .y = -imu.get_linear_acceleration(proxy::Imu::Axis::X)},
            .imu_is_new = imu.is_new(),
            .walls = {},
        };
    };

    bool running = false;

    TestCore::loop([&]() {
        tick.wait();
        button.update();
        imu.update();

        const nav::Measurements measurements = measure();
        localizer.predict(measurements, loop_time);

        if (not running) {
            localizer.correct_at_rest(measurements, loop_time);

            if (button.get_status() != proxy::Button::Status::NO_PRESS) {
                proxy::Stopwatch::sleep_ms(3000);

                imu.update();
                localizer.reset(origin, measure());
                executor.reset(origin, search_profile);
                executor.push(route);
                locomotion.enable();
                running = true;
            }

            return;
        }

        const nav::Reference reference = executor.update(loop_time, controller.get_time_scale(), localizer.get_state());
        const nav::Controller::Command command = controller.update(reference, localizer.get_state());

        locomotion.set_command(command.forward, command.rotation);

        const nav::Controller::Status& status = controller.get_status();

        test_reference_speed = reference.twist.linear;
        test_measured_speed = localizer.get_state().velocity.linear;
        test_along_error = status.along_error;
        test_across_error = status.across_error;
        test_orientation_error = status.orientation_error;
        test_forward_feed_forward = status.forward_feed_forward;
        test_forward_feedback = status.forward_feedback;
        test_rotation_feed_forward = status.rotation_feed_forward;
        test_rotation_feedback = status.rotation_feedback;

        if (executor.is_finished()) {
            locomotion.stop();
            locomotion.disable();
            running = false;
        }
    });

    return 0;
}
