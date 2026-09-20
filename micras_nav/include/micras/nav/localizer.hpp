/**
 * @file
 */

#ifndef MICRAS_NAV_LOCALIZER_HPP
#define MICRAS_NAV_LOCALIZER_HPP

#include <array>
#include <cstdint>

#include "micras/nav/maze.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/robot_model.hpp"
#include "micras/nav/state.hpp"
#include "micras/nav/wall_model.hpp"

namespace micras::nav {
/**
 * @brief The one owner of the pose of the robot.
 *
 * @details An extended Kalman filter over the position, the orientation and the bias of the
 * gyroscope, in the maze frame. It is advanced every iteration with the encoders and the gyroscope,
 * and corrected with every range of a wall sensor that can be trusted, each as one scalar
 * measurement against the map of the walls. Because the filter knows that a lateral drift and an
 * orientation error go together, a wall seen for a moment identifies the bias of the gyroscope, and
 * the estimate stays good through the gaps, turns and diagonals that follow. Standing still is one
 * more measurement, of the bias alone.
 *
 * Every correction goes through three guards: the illuminated spot has to lie on the face of a
 * single known wall, the innovation has to be plausible for what the filter believes, and the
 * correction is capped, so that a wrong reading is rejected and a wrong wall in the map can only
 * drag the pose slowly.
 *
 * @note The covariance is kept as the factors of `P = U * D * U^T`, with U unit upper triangular and
 * D diagonal, and updated with the algorithms of Thornton and Bierman. In single precision this
 * stays symmetric and positive by construction, which the conventional form does not.
 */
class Localizer {
public:
    /**
     * @brief Number of states of the filter: x, y, orientation and gyroscope bias.
     */
    static constexpr uint8_t number_of_states{4};

    /**
     * @brief Largest number of iterations the speed can be measured over.
     */
    static constexpr uint8_t max_speed_window{32};

    /**
     * @brief Configuration struct for the localizer.
     *
     * @note The gate is the largest squared innovation accepted, in variances, and the one used
     * while standing still is wider since a reading taken at rest has no timing error. The caps
     * bound what one correction may move the pose by. The range delay is the delay of the filter
     * the fast ranges went through. That filter is slower than the ranges are sampled, so the noise
     * of consecutive ranges is mostly the same noise, and the range correlation is how many of them
     * share it: the variance of each is multiplied by it, so that what a second of ranges is worth
     * does not depend on how many there are in it. The speed window is the number of iterations the linear
     * speed is measured over, which trades noise for delay. The rest window is the time over which
     * the gyroscope is compared with the encoders to measure its bias.
     *
     * The ends of the side walls are used as references along the path when use_edges is set. An
     * end is accepted while the robot moves forward faster than the edge speed, since the only time
     * it reverses is to park, from a pose it has just measured. The wall has to be closer than the
     * edge range, so that nothing can be hiding its end, and the end has to be found within the edge
     * window of where it was expected. An end is a single event rather than a stream of readings, so it has
     * a cap of its own. The range tolerance, a constant part plus a part proportional to the range,
     * tells a reading that is on a wall from one that is past its end.
     */
    struct Config {
        RobotModel model;
        float      initial_position_deviation;
        float      initial_orientation_deviation;
        float      initial_bias_deviation;
        float      gate;
        float      stationary_gate;
        float      max_position_correction;
        float      max_orientation_correction;
        float      max_angular_speed;
        float      stationary_linear_speed;
        float      stationary_angular_speed;
        float      range_delay;
        float      range_correlation;
        float      rest_window;
        bool       use_edges;
        float      edge_deviation;
        float      edge_window;
        float      edge_speed;
        float      edge_range;
        float      max_edge_correction;
        float      range_tolerance;
        float      relative_range_tolerance;
        uint8_t    speed_window;
    };

    /**
     * @brief Diagnostics of the filter, for a monitor.
     *
     * @note The innovation level is the running mean of the squared innovations in variances, which
     * stays near one while the filter is consistent.
     */
    struct Status {
        float    innovation_level;
        uint32_t accepted;
        uint32_t rejected;
        uint32_t edges;
    };

    /**
     * @brief Construct a new Localizer object.
     *
     * @param config The configuration for the localizer.
     */
    explicit Localizer(const Config& config);

    /**
     * @brief Place the robot at a known pose, forgetting the previous estimate but not the bias.
     *
     * @param pose The pose of the robot in the maze frame.
     * @param measurements The current measurements, which give the reference for the encoders.
     */
    void reset(const Pose& pose, const Measurements& measurements);

    /**
     * @brief Advance the estimate by one iteration.
     *
     * @param measurements The current measurements.
     * @param elapsed_time Time since the last iteration, in seconds.
     */
    void predict(const Measurements& measurements, float elapsed_time);

    /**
     * @brief Correct the estimate with the ranges of the wall sensors that can be trusted.
     *
     * @tparam width The width of the maze in cells.
     * @tparam height The height of the maze in cells.
     * @param measurements The current measurements.
     * @param wall_model The geometry of the wall sensors.
     * @param maze The map of the walls, of which only the known walls are used.
     */
    template <uint8_t width, uint8_t height>
    void correct(const Measurements& measurements, const WallModel& wall_model, const TMaze<width, height>& maze);

    /**
     * @brief Correct the bias of the gyroscope while the robot is commanded to stand still.
     *
     * @details The rotation the gyroscope reports over a window of time is compared with the one
     * the encoders report, and the difference is a measurement of the bias. The encoders are what
     * makes this safe: a robot that is slowly turning while it believes it is at rest would
     * otherwise have its rotation taken for bias, which is the very error that makes it turn.
     * Nothing is measured while the wheels are moving forward, since wheels that slip would then
     * report a rotation that did not happen.
     *
     * @param measurements The current measurements.
     * @param elapsed_time Time since the last iteration, in seconds.
     */
    void correct_at_rest(const Measurements& measurements, float elapsed_time);

    /**
     * @brief Get the estimated state of the robot.
     *
     * @return The pose in the maze frame and the velocity in the body frame.
     */
    const State& get_state() const;

    /**
     * @brief Get the estimated pose of the robot.
     *
     * @return The pose in the maze frame.
     */
    const Pose& get_pose() const;

    /**
     * @brief Get the cell the robot is in and the direction it faces.
     *
     * @return The grid pose derived from the estimate.
     */
    GridPose get_cell() const;

    /**
     * @brief Get the estimated bias of the gyroscope.
     *
     * @return The bias in rad/s.
     */
    float get_gyroscope_bias() const;

    /**
     * @brief Get the standard deviation of the estimated position.
     *
     * @return The largest of the deviations along x and y, in meters.
     */
    float get_position_deviation() const;

    /**
     * @brief Get the standard deviation of the estimated orientation.
     *
     * @return The deviation in radians.
     */
    float get_orientation_deviation() const;

    /**
     * @brief Get the diagnostics of the filter.
     *
     * @return The status.
     */
    const Status& get_status() const;

private:
    /**
     * @brief Vector with one element per state.
     */
    using Vector = std::array<float, number_of_states>;

    /**
     * @brief Matrix with one row and one column per state.
     */
    using Matrix = std::array<Vector, number_of_states>;

    /**
     * @brief Indexes of the states.
     */
    ///@{
    static constexpr uint8_t x_index{0};
    static constexpr uint8_t y_index{1};
    static constexpr uint8_t orientation_index{2};
    static constexpr uint8_t bias_index{3};

    ///@}

    /**
     * @brief What a side looking sensor was seeing at its previous sample.
     *
     * @note The end of a wall shows up as a change between two consecutive samples, from a reading
     * on the wall to a reading past it or the other way around, so both are remembered.
     */
    struct EdgeTracker {
        bool   tracking;
        bool   on_wall;
        RayHit hit;
    };

    /**
     * @brief Look for the end of a side wall in the reading of a sensor, and use it as a reference.
     *
     * @tparam width The width of the maze in cells.
     * @tparam height The height of the maze in cells.
     * @param sensor The index of the sensor.
     * @param reading The reading of the sensor.
     * @param sampled The pose of the robot when the reading was taken.
     * @param hit What the axis of the sensor meets from that pose.
     * @param wall_model The geometry of the wall sensors.
     * @param maze The map of the walls.
     */
    template <uint8_t width, uint8_t height>
    void track_edge(
        uint8_t sensor, const WallReading& reading, const Pose& sampled, const RayHit& hit, const WallModel& wall_model,
        const TMaze<width, height>& maze
    );

    /**
     * @brief Set the covariance to a diagonal one.
     *
     * @param variances The variance of each state.
     */
    void set_covariance(const Vector& variances);

    /**
     * @brief Propagate the covariance through one iteration, with the algorithm of Thornton.
     *
     * @param transition The derivative of the new state with respect to the old one.
     * @param noise_input How each source of process noise enters the state.
     * @param noise_variances The variance of each source of process noise.
     */
    void propagate(const Matrix& transition, const Matrix& noise_input, const Vector& noise_variances);

    /**
     * @brief Apply one scalar measurement, with the algorithm of Bierman.
     *
     * @param innovation The measurement minus its prediction.
     * @param jacobian The derivative of the measurement with respect to the state.
     * @param variance The variance of the measurement noise.
     * @param gate The largest squared innovation accepted, in variances.
     * @param max_position_correction The most the measurement may move the position by.
     * @return True if the measurement was accepted.
     */
    bool update(float innovation, const Vector& jacobian, float variance, float gate, float max_position_correction);

    /**
     * @brief Get the variance of one state.
     *
     * @param index The index of the state.
     * @return The element of the diagonal of the covariance.
     */
    float get_variance(uint8_t index) const;

    /**
     * @brief Check if the robot is standing still, as far as its sensors can tell.
     *
     * @return True if both speeds are below the stationary thresholds.
     */
    bool is_stationary() const;

    /**
     * @brief Parameters of the filter, with the physical description of the robot.
     */
    Config config;

    /**
     * @brief Estimated state of the robot.
     */
    State state{};

    /**
     * @brief Estimated bias of the gyroscope.
     */
    float bias{};

    /**
     * @brief Unit upper triangular factor of the covariance.
     */
    Matrix upper{};

    /**
     * @brief Diagonal factor of the covariance.
     */
    Vector diagonal{};

    /**
     * @brief Wheel angles at the last iteration.
     */
    ///@{
    float last_left_angle{};
    float last_right_angle{};
    ///@}

    /**
     * @brief Time since the gyroscope last produced a sample.
     */
    float time_since_imu{};

    /**
     * @brief Duration of the window over which the bias is being measured.
     */
    float rest_time{};

    /**
     * @brief Rotation reported by the gyroscope over the window.
     */
    float rest_rotation{};

    /**
     * @brief Difference between the wheel angles at the start of the window.
     */
    float rest_wheel_difference{};

    /**
     * @brief Distance traveled at each of the last iterations, for the speed estimate.
     */
    std::array<float, max_speed_window> distances{};

    /**
     * @brief Duration of each of the last iterations, for the speed estimate.
     */
    std::array<float, max_speed_window> durations{};

    /**
     * @brief Index of the oldest entry of the speed window.
     */
    uint8_t window_index{};

    /**
     * @brief Distance traveled over the speed window.
     */
    float window_distance{};

    /**
     * @brief Duration of the speed window.
     */
    float window_duration{};

    /**
     * @brief What each sensor was seeing at its previous sample.
     */
    std::array<EdgeTracker, number_of_wall_sensors> edge_trackers{};

    /**
     * @brief Diagnostics of the filter.
     */
    Status status{};
};
}  // namespace micras::nav

#include "micras/nav/impl/localizer.tpp"  // IWYU pragma: export

#endif  // MICRAS_NAV_LOCALIZER_HPP
