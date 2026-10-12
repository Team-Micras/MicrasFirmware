/**
 * @file
 */

#ifndef MICRAS_SURVEY_HPP
#define MICRAS_SURVEY_HPP

#include <array>
#include <cstdint>
#include <vector>

#include "micras/core/serializable.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/robot_model.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/state.hpp"

namespace micras {
/**
 * @brief Drive that measures the wall sensors against the distances the wheels travel, for their
 * references to be fitted on the host.
 *
 * @details The robot starts in a straight corridor, centered, with its front caps touching the wall
 * at its end. It backs away from that wall, turns half a turn in place and backs toward it again,
 * steered by the wheels and the gyroscope only, and keeps what every sensor reads at each step of
 * the way. The first pass gives the fronts against their distance to the wall; between the two
 * passes each diagonal sees both walls of the corridor, so how far off its center the robot was
 * cancels out of the product of the two.
 *
 * @note The samples are read from the link one page at a time, through the page the link writes
 * and the serialization of this object, so that a page lost on the way is asked for again instead
 * of being missing from the stream.
 */
class Survey : public core::ISerializable {
public:
    /**
     * @brief Configuration of the survey.
     */
    struct Config {
        float back_distance;
        float return_distance;
        float speed;
        float step;
        float settle_time;
    };

    /**
     * @brief What the sensors read at a point of the survey.
     *
     * @note The travel is how far the axle moved away from the wall since the start, the lateral
     * offset how far it moved to the left of its starting line, and the heading how far it turned
     * from its starting one.
     */
    struct Sample {
        float                                          travel;
        float                                          lateral;
        float                                          heading;
        std::array<float, nav::number_of_wall_sensors> intensities;
        uint8_t                                        pass;
    };

    /**
     * @brief Number of samples a survey keeps.
     */
    static constexpr uint16_t capacity{320};

    /**
     * @brief Number of samples in a page of the serialization.
     */
    static constexpr uint8_t samples_per_page{6};

    /**
     * @brief Construct a new Survey object.
     *
     * @param config The configuration of the survey.
     */
    explicit Survey(const Config& config);

    /**
     * @brief Start a survey from a pose.
     *
     * @param pose The pose of the robot at the start, touching the wall ahead.
     * @param linear Limits of the straight drives.
     * @param angular Limits of the turn in place.
     */
    void start(const nav::Pose& pose, const nav::MotionLimits& linear, const nav::MotionLimits& angular);

    /**
     * @brief Advance the survey.
     *
     * @param pose The estimated pose of the robot.
     * @param intensities What each wall sensor reads.
     * @param elapsed_time Time since the last update, in seconds.
     * @return The reference for the controller to follow.
     */
    nav::Reference update(
        const nav::Pose& pose, const std::array<float, nav::number_of_wall_sensors>& intensities, float elapsed_time
    );

    /**
     * @brief Whether the survey is over.
     *
     * @return True once the robot is back at rest after the second pass.
     */
    bool is_finished() const;

    /**
     * @brief Whether the robot is held still, where the bias of the gyroscope can be measured.
     *
     * @return True while settling before the first pass and around the half turn.
     */
    bool is_resting() const;

    /**
     * @brief Get the page the serialization returns, for the link to choose it.
     *
     * @return Reference to the page index.
     */
    uint16_t& get_page();

    /**
     * @brief Get the number of samples kept.
     *
     * @return Reference to the number of samples.
     */
    const uint16_t& get_count() const;

    /**
     * @brief Serialize the page chosen.
     *
     * @return The page index and the number of samples as two bytes each, then each sample of the page as
     * its pass and its seven floats.
     */
    std::vector<uint8_t> serialize() const override;

    /**
     * @brief Ignore what the link writes, since the samples only go out.
     *
     * @param serial_data Serialized data.
     * @param size Size of the serialized data.
     */
    void deserialize(const uint8_t* serial_data, uint16_t size) override;

private:
    /**
     * @brief Steps of the survey.
     */
    enum class Phase : uint8_t {
        SETTLE,
        BACK,
        PAUSE,
        TURN,
        REST,
        RETURN,
        FINISHED,
    };

    /**
     * @brief Keep a sample, if there is room for it.
     *
     * @param pose The estimated pose of the robot.
     * @param intensities What each wall sensor reads.
     * @param pass The pass the sample belongs to, 0 at rest before the first one.
     */
    void record(const nav::Pose& pose, const std::array<float, nav::number_of_wall_sensors>& intensities, uint8_t pass);

    /**
     * @brief Get how far the axle moved away from the wall since the start.
     *
     * @param pose The estimated pose of the robot.
     * @return The travel, in meters.
     */
    float get_travel(const nav::Pose& pose) const;

    /**
     * @brief Configuration of the survey.
     */
    Config config;

    /**
     * @brief Current step and the time spent in it.
     */
    Phase phase{Phase::FINISHED};
    float phase_time{};

    /**
     * @brief Pose at the start, and pose the current step starts from.
     */
    nav::Pose origin{};
    nav::Pose anchor{};

    /**
     * @brief Profiles of the drives and of the turn.
     */
    nav::SpeedProfile back;
    nav::SpeedProfile turn;
    nav::SpeedProfile ret;

    /**
     * @brief Travel at the last sample kept.
     */
    float last_travel{};

    /**
     * @brief Samples kept, and how many.
     */
    std::array<Sample, capacity> samples{};
    uint16_t                     count{};

    /**
     * @brief Page the serialization returns.
     */
    uint16_t page{};
};
}  // namespace micras

#endif  // MICRAS_SURVEY_HPP
