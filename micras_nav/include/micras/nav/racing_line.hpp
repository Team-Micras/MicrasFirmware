/**
 * @file
 */

#ifndef MICRAS_NAV_RACING_LINE_HPP
#define MICRAS_NAV_RACING_LINE_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/core/vector.hpp"
#include "micras/nav/grid_pose.hpp"
#include "micras/nav/line.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/planner.hpp"
#include "micras/nav/segment.hpp"

namespace micras::nav {
/**
 * @brief The smoothest line through the cells of a planned route, and the speeds to drive it at.
 *
 * @details The route is sampled every few millimeters, and each sample may then slide sideways,
 * along the normal of the line, as far as the outline of the robot, turned along the line, keeps
 * the margin to every post and to every wall the route does not cross. The walls the route does not
 * cross are taken as present, whatever the map says, so the line never leaves the cells of the
 * route. Among the slides allowed, the ones chosen make the sum of the squared second differences
 * of the samples smallest, which is the sum of the squared curvatures, plus a small weight on the
 * squared first differences, which prefers the shorter line.
 *
 * That is a least squares problem with bounds on each unknown, where each unknown only meets its two
 * neighbors on each side. It is solved in windows of a few tens of samples, the samples around a
 * window held where they are, by a projected Newton method on the banded normal equations. The
 * windows overlap and sweep the line from its start to its end, and the line is resampled at equal
 * steps after each sweep. A slide is limited per sweep, and the bounds are found again for every
 * window, since the robot needs more room as the line turns it. After the last sweep every sample
 * is checked, the curvature is measured and lightly smoothed, and the speeds are planned along it
 * with the same rule as a turn.
 *
 * Nothing here is bounded to one iteration of the loop as a whole: every phase advances by a budget
 * of samples per call, except for the speeds, which take a few milliseconds in one call, the robot
 * being stopped.
 *
 * @note The first and last samples never move: the line starts where the robot stands and ends
 * where the route comes to rest.
 *
 * @tparam width The width of the maze in cells.
 * @tparam height The height of the maze in cells.
 */
template <uint8_t width, uint8_t height>
class TRacingLine {
public:
    /**
     * @brief Configuration struct for the racing line.
     *
     * @note The margin is the distance the line keeps between the outline of the robot and any
     * obstacle wherever the route leaves room for it, with the risky switch too: at the margin of
     * the risky turns the line hit a wall on 4 to 6 of 10 mazes. The least margins are what the
     * final check requires everywhere, without and with the risky switch: the margins the turns were
     * designed with, less what the resampling and the headings of the final line use up. Where the
     * route itself comes closer than the margin, which the risky turns do, the line does not move. The trust is
     * how far a sample may slide in one sweep, the scan step the resolution of the bounds, and the
     * length weight the weight of the first differences, in units of the spacing. A sweep that
     * slides no sample farther than the convergence tolerance ends the optimization. The smoothing
     * is the number of samples on each side the curvature is averaged over. The lateral share is the
     * part of the lateral grip of the run the curves of the line are planned with: the tires slide
     * sideways in proportion to what they are asked for, which the pose estimate does not see, and
     * the line holds its curves far longer than a turn does.
     */
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init) no defaults, so that a missing field is a warning
    struct Config {
        float    spacing;
        float    margin;
        float    least_margin;
        float    risky_least_margin;
        float    trust;
        float    scan_step;
        float    length_weight;
        float    convergence;
        float    lateral_share;
        uint8_t  max_sweeps;
        uint8_t  smoothing;
        uint16_t samples_per_step;
    };

    /**
     * @brief Construct a new TRacingLine object.
     *
     * @param dynamics The physical limits of the robot, borrowed for the lifetime of the object.
     * @param config The configuration for the racing line.
     */
    TRacingLine(const Dynamics& dynamics, const Config& config);

    /**
     * @brief Forget the line.
     */
    void reset();

    /**
     * @brief Start optimizing the line of a route.
     *
     * @param route The route on the lattice, which tells the walls it crosses.
     * @param segments The segments the route compiles to, borrowed until the optimization ends.
     * @param maze The map of the walls, borrowed until the optimization ends.
     * @param profile The profile of the run.
     */
    void begin(
        const Route& route, std::span<const Segment> segments, const TMaze<width, height>& maze,
        const RunProfile& profile
    );

    /**
     * @brief Advance the optimization.
     *
     * @return True if it has finished, whether a line was found or not.
     */
    bool step();

    /**
     * @brief Get the line.
     *
     * @return The line, which is ready only if the last optimization found one.
     */
    const Line& get_line() const;

    /**
     * @brief Get the number of sweeps the last optimization made.
     *
     * @return The number of sweeps.
     */
    uint8_t get_sweeps() const;

private:
    /**
     * @brief Largest number of samples solved for at once.
     */
    static constexpr uint8_t window{64};

    /**
     * @brief Number of samples a window starts after the start of the one before.
     */
    static constexpr uint8_t stride{48};

    /**
     * @brief Number of samples at each end of the line that never move.
     */
    static constexpr uint8_t fixed_samples{3};

    /**
     * @brief Largest number of points along the outline of the robot that are checked.
     */
    static constexpr uint8_t max_outline_points{32};

    /**
     * @brief Largest distance between two points of the outline that are checked, in meters.
     *
     * @note Below the thickness of a wall plus twice the smallest margin, so that no wall can pass
     * between two points unseen.
     */
    static constexpr float outline_spacing{0.02F};

    /**
     * @brief Largest number of iterations of the projected Newton method in a window.
     */
    static constexpr uint8_t max_newton_iterations{30};

    /**
     * @brief Largest number of times a window that brings the robot too close is shrunk.
     */
    static constexpr uint8_t max_shrinks{8};

    /**
     * @brief Largest number of steps of distance scanned on each side of a sample.
     */
    static constexpr uint8_t max_scan_steps{32};

    /**
     * @brief Number of lattice points along each axis, a lattice unit being half a cell.
     */
    ///@{
    static constexpr uint16_t lattice_width{2 * width + 1};
    static constexpr uint16_t lattice_height{2 * height + 1};
    ///@}

    /**
     * @brief Phase of the optimization.
     */
    enum class Phase : uint8_t {
        IDLE = 0,
        SAMPLE = 1,
        BOUNDS = 2,
        SOLVE = 3,
        MEASURE = 4,
        RESAMPLE = 5,
        CHECK = 6,
        CURVATURE = 7,
        SMOOTH = 8,
        SPEED = 9,
    };

    /**
     * @brief Sample the compiled route at the next samples of the budget.
     *
     * @param budget The number of samples left in this call, which is decreased.
     */
    void sample_route(uint32_t& budget);

    /**
     * @brief Find the bounds of the next samples of the window.
     *
     * @param budget The number of samples left in this call, which is decreased.
     */
    void find_bounds(uint32_t& budget);

    /**
     * @brief Solve the window, move its samples and go on to the next window.
     */
    void solve_window();

    /**
     * @brief Measure the length of the line for the resampling.
     *
     * @param budget The number of samples left in this call, which is decreased.
     */
    void measure(uint32_t& budget);

    /**
     * @brief Resample the line at equal steps, then start the next sweep or the checks.
     *
     * @param budget The number of samples left in this call, which is decreased.
     */
    void resample(uint32_t& budget);

    /**
     * @brief Check the clearance of the next samples of the line.
     *
     * @param budget The number of samples left in this call, which is decreased.
     */
    void check(uint32_t& budget);

    /**
     * @brief Measure the curvature at the next samples of the line.
     *
     * @param budget The number of samples left in this call, which is decreased.
     */
    void measure_curvature(uint32_t& budget);

    /**
     * @brief Smooth the curvature at the next samples of the line.
     *
     * @param budget The number of samples left in this call, which is decreased.
     */
    void smooth_curvature(uint32_t& budget);

    /**
     * @brief Plan the speeds along the line, time it and mark it ready.
     */
    void plan_speeds();

    /**
     * @brief Start a sweep from the first window.
     */
    void start_sweep();

    /**
     * @brief Start the window at the current window start.
     */
    void start_window();

    /**
     * @brief End the optimization.
     *
     * @param found Whether a line was found.
     */
    void finish(bool found);

    /**
     * @brief Get the heading of the line at a sample, from the samples around it.
     *
     * @param index The index of the sample.
     * @return The heading in radians.
     */
    float get_heading(uint16_t index) const;

    /**
     * @brief Solve for the slides of the window within their bounds.
     *
     * @param normals_x The x component of the normal of each sample of the window.
     * @param normals_y The y component of the normal of each sample of the window.
     */
    void solve_slides(std::span<const double> normals_x, std::span<const double> normals_y);

    /**
     * @brief Check if the robot keeps a margin to every obstacle at a pose.
     *
     * @param position The position of the robot.
     * @param cosine The cosine of its heading.
     * @param sine The sine of its heading.
     * @param margin The distance to keep.
     * @return True if nothing is closer than the margin.
     */
    bool is_clear(const core::Vector& position, float cosine, float sine, float margin) const;

    /**
     * @brief Check if the robot comes within a margin of a rectangle aligned with the grid.
     *
     * @param position The position of the robot.
     * @param cosine The cosine of its heading.
     * @param sine The sine of its heading.
     * @param center The center of the rectangle.
     * @param half_size The half width and half height of the rectangle.
     * @param margin The distance to keep.
     * @return True if the robot is closer than the margin.
     */
    bool hits(
        const core::Vector& position, float cosine, float sine, const core::Vector& center,
        const core::Vector& half_size, float margin
    ) const;

    /**
     * @brief Check if a wall may be crossed by the line.
     *
     * @param x The x coordinate of the midpoint of the wall on the lattice.
     * @param y The y coordinate of the midpoint of the wall on the lattice.
     * @return True if the route crosses the wall, or it is inside the goal and known to be absent.
     */
    bool is_open(int32_t x, int32_t y) const;

    /**
     * @brief Mark a wall as one the line may cross.
     *
     * @param wall The wall, as a side of a cell.
     */
    void open(const GridPose& wall);

    /**
     * @brief Physical limits of the robot.
     */
    // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    const Dynamics& dynamics;

    /**
     * @brief Parameters of the optimization.
     */
    Config config;

    /**
     * @brief Points of the outline of the robot, in its own frame, and how many there are.
     */
    ///@{
    std::array<core::Vector, max_outline_points> outline{};
    uint8_t                                      outline_size{};
    ///@}

    /**
     * @brief Distance from the axle to the farthest point of the outline.
     */
    float reach{};

    /**
     * @brief The line being optimized, and then driven.
     */
    Line line;

    /**
     * @brief Walls the line may cross, one bit per point of the lattice.
     */
    std::array<uint8_t, (lattice_width * lattice_height + 7) / 8> openings{};

    /**
     * @brief Map of the walls, while the line is optimized.
     */
    const TMaze<width, height>* maze{nullptr};

    /**
     * @brief Segments of the route, while the line is optimized.
     */
    std::span<const Segment> segments;

    /**
     * @brief Profile of the run.
     */
    RunProfile run_profile{};

    /**
     * @brief Margin the line must keep everywhere, for the run.
     */
    float least_margin{};

    /**
     * @brief Phase of the optimization.
     */
    Phase phase{Phase::IDLE};

    /**
     * @brief Next sample of the phase in progress.
     */
    uint16_t cursor{};

    /**
     * @brief Segment being sampled, and the distance along the route at its start.
     */
    ///@{
    uint16_t segment_index{};
    float    segment_start{};
    ///@}

    /**
     * @brief Length of the line being resampled, and the distance along it at the sample read.
     */
    ///@{
    float measured_length{};
    float read_length{};
    ///@}

    /**
     * @brief Sample of the line being resampled that is read, and the number of samples written.
     */
    ///@{
    uint16_t read_index{};
    uint16_t resampled_size{};
    ///@}

    /**
     * @brief Number of sweeps done, and the farthest slide of the sweep in progress.
     */
    ///@{
    uint8_t sweeps{};
    float   sweep_slide{};
    ///@}

    /**
     * @brief Heading of the line at the sample before, for the curvature.
     */
    float previous_heading{};

    /**
     * @brief First sample of the window and its number of samples.
     */
    ///@{
    uint16_t window_start{};
    uint8_t  window_size{};
    ///@}

    /**
     * @brief Heading, bounds and position before the move of each sample of the window.
     */
    ///@{
    std::array<float, window> headings{};
    std::array<float, window> lower{};
    std::array<float, window> upper{};
    std::array<float, window> old_x{};
    std::array<float, window> old_y{};
    ///@}

    /**
     * @brief Whether each sample of the window and its two neighbors kept the margin before the move.
     */
    std::array<bool, window + 2> was_clear{};

    /**
     * @brief Banded normal equations of the window: the diagonal, the two bands above it and the
     * linear term.
     */
    ///@{
    std::array<double, window> diagonal{};
    std::array<double, window> first_band{};
    std::array<double, window> second_band{};
    std::array<double, window> linear{};
    ///@}

    /**
     * @brief Slides of the window, and the work space of the projected Newton method.
     */
    ///@{
    std::array<double, window>  slides{};
    std::array<double, window>  gradient{};
    std::array<double, window>  direction{};
    std::array<double, window>  trial{};
    std::array<double, window>  factor_diagonal{};
    std::array<double, window>  factor_first{};
    std::array<double, window>  factor_second{};
    std::array<uint8_t, window> free_indices{};
    ///@}
};
}  // namespace micras::nav

#include "micras/nav/impl/racing_line.tpp"  // IWYU pragma: export

#endif  // MICRAS_NAV_RACING_LINE_HPP
