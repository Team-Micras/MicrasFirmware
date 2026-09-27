/**
 * @file
 */

#ifndef MICRAS_NAV_LATTICE_HPP
#define MICRAS_NAV_LATTICE_HPP

#include <array>
#include <cstdint>
#include <numbers>
#include <utility>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Slalom turns the robot can execute, named by where they start and end.
 *
 * @details S is a straight along the grid and D a diagonal, followed by the angle turned. SS90S is
 * the short turn that fits inside one cell, which is also the turn used while searching, and SS90L
 * the long one that needs a cell of straight on both sides.
 *
 * The turns from SD45E on enter a cell and leave it through one of its other three walls, two bends
 * each. Their last letter is the wall they leave through, in the canonical frame of TurnPrimitive: T
 * the wall on the left of the entry, E the wall across from it and B the wall on the right. Those that
 * start on a diagonal may end turned to the right of it, since the side of a diagonal turn is fixed by
 * the wall it starts on: DS45E, DD90E and DS135B turn right, and DD0E does not turn,
 * shifting to the next diagonal.
 */
enum class TurnId : uint8_t {
    SS90S = 0,
    SS90L = 1,
    SS180 = 2,
    SD45 = 3,
    SD135 = 4,
    DS45 = 5,
    DS135 = 6,
    DD90 = 7,
    SD45E = 8,
    SD45T = 9,
    SD135T = 10,
    DS45T = 11,
    DD0E = 12,
    DS45E = 13,
    DD90E = 14,
    DS135B = 15,
    NUMBER_OF_TURNS = 16,
};

/**
 * @brief Number of different turns.
 */
inline constexpr uint8_t number_of_turns{std::to_underlying(TurnId::NUMBER_OF_TURNS)};

/**
 * @brief First of the turns of two bends, whose shapes are designed before the firmware is compiled
 * rather than while it is.
 */
inline constexpr uint8_t first_two_bend_turn{std::to_underlying(TurnId::SD45E)};

/**
 * @brief Number of turns of two bends.
 */
inline constexpr uint8_t number_of_two_bend_turns{number_of_turns - first_two_bend_turn};

/**
 * @brief Side a turn is made to.
 */
enum class TurnSide : uint8_t {
    LEFT = 0,
    RIGHT = 1,
};

/**
 * @brief Point of the lattice the routes are planned on, in units of half a cell.
 *
 * @details Both coordinates even is a post, both odd is the center of a cell, and one of each is
 * the midpoint of a wall: a vertical wall when x is even and a horizontal one when y is even. Every
 * slalom turn starts and ends at the midpoint of a wall, which is why those are the nodes.
 */
struct LatticePoint {
    /**
     * @brief Check if the point is the midpoint of a wall that runs along y.
     *
     * @return True if the point is on a vertical wall.
     */
    constexpr bool on_vertical_wall() const { return this->x % 2 == 0 and this->y % 2 != 0; }

    /**
     * @brief Check if the point is the midpoint of a wall that runs along x.
     *
     * @return True if the point is on a horizontal wall.
     */
    constexpr bool on_horizontal_wall() const { return this->x % 2 != 0 and this->y % 2 == 0; }

    /**
     * @brief Add two points.
     *
     * @param other The point to add.
     * @return The sum of the points.
     */
    constexpr LatticePoint operator+(const LatticePoint& other) const {
        return {.x = static_cast<int8_t>(this->x + other.x), .y = static_cast<int8_t>(this->y + other.y)};
    }

    /**
     * @brief Compare two points for equality.
     *
     * @param other The other point to compare.
     * @return True if the points are equal, false otherwise.
     */
    constexpr bool operator==(const LatticePoint& other) const = default;

    int8_t x;
    int8_t y;
};

/**
 * @brief Node of the lattice: the midpoint of a wall, crossed with one of eight headings.
 *
 * @note The heading counts steps of 45 degrees from the positive x axis, counterclockwise, so the
 * even ones run along the grid and the odd ones are diagonal.
 */
struct LatticePose {
    /**
     * @brief Number of headings.
     */
    static constexpr uint8_t number_of_headings{8};

    /**
     * @brief Check if the heading is diagonal.
     *
     * @return True if the heading is diagonal, false if it runs along the grid.
     */
    constexpr bool is_diagonal() const { return this->heading % 2 != 0; }

    /**
     * @brief Check if the node exists, which takes a wall that the heading crosses.
     *
     * @return True if the node is the midpoint of a wall and the heading is not parallel to it.
     */
    constexpr bool is_valid() const {
        if (this->point.on_vertical_wall()) {
            return this->heading % 4 != 2;
        }

        return this->point.on_horizontal_wall() and this->heading % 4 != 0;
    }

    /**
     * @brief Get the displacement of one step along the heading.
     *
     * @note A step goes from one wall to the next, so it is a whole cell along the grid and half of
     * the diagonal of a cell along a diagonal.
     *
     * @return The displacement in lattice units.
     */
    constexpr LatticePoint step() const {
        constexpr std::array<LatticePoint, number_of_headings> steps{{
            {.x = 2, .y = 0},
            {.x = 1, .y = 1},
            {.x = 0, .y = 2},
            {.x = -1, .y = 1},
            {.x = -2, .y = 0},
            {.x = -1, .y = -1},
            {.x = 0, .y = -2},
            {.x = 1, .y = -1},
        }};

        return steps.at(this->heading);
    }

    /**
     * @brief Get the node one step ahead.
     *
     * @return The node after moving one step along the heading.
     */
    constexpr LatticePose advanced() const { return {.point = this->point + this->step(), .heading = this->heading}; }

    /**
     * @brief Get the heading as an angle.
     *
     * @return The heading in radians, in [0, 2*pi).
     */
    constexpr float angle() const { return static_cast<float>(this->heading) * std::numbers::pi_v<float> / 4.0F; }

    /**
     * @brief Get the side a diagonal node can turn to.
     *
     * @details A robot that crosses a wall diagonally is veering to one side of the perpendicular
     * of that wall, and every turn that starts there continues to that side: the ones to the other
     * side only become possible one step later, at the next wall.
     *
     * @return The side of the turns that can start at this node.
     */
    constexpr TurnSide diagonal_turn_side() const {
        const bool even_quadrant = (this->heading / 2) % 2 == 0;
        return this->point.on_vertical_wall() == even_quadrant ? TurnSide::LEFT : TurnSide::RIGHT;
    }

    /**
     * @brief Get the wall the node is on, as a side of a cell.
     *
     * @note The cell is the one behind the wall, except for the walls of the left and bottom
     * borders, which only have a cell ahead.
     *
     * @return The cell and the side of it where the wall is.
     */
    constexpr GridPose wall() const {
        if (this->point.on_vertical_wall()) {
            const auto row = static_cast<uint8_t>((this->point.y - 1) / 2);

            if (this->point.x < 2) {
                return {.position = {.x = 0, .y = row}, .orientation = Side::LEFT};
            }

            return {
                .position = {.x = static_cast<uint8_t>(this->point.x / 2 - 1), .y = row}, .orientation = Side::RIGHT
            };
        }

        const auto column = static_cast<uint8_t>((this->point.x - 1) / 2);

        if (this->point.y < 2) {
            return {.position = {.x = column, .y = 0}, .orientation = Side::DOWN};
        }

        return {.position = {.x = column, .y = static_cast<uint8_t>(this->point.y / 2 - 1)}, .orientation = Side::UP};
    }

    /**
     * @brief Get the cell the robot is entering when it crosses the node.
     *
     * @note The coordinates are signed so that a node on the border, heading out, gives a cell
     * outside of the maze instead of wrapping around.
     *
     * @return The coordinates of the cell ahead of the wall.
     */
    constexpr LatticePoint cell_ahead() const {
        const LatticePoint step = this->step();

        if (this->point.on_vertical_wall()) {
            return {
                .x = static_cast<int8_t>(this->point.x / 2 - (step.x > 0 ? 0 : 1)),
                .y = static_cast<int8_t>((this->point.y - 1) / 2),
            };
        }

        return {
            .x = static_cast<int8_t>((this->point.x - 1) / 2),
            .y = static_cast<int8_t>(this->point.y / 2 - (step.y > 0 ? 0 : 1)),
        };
    }

    /**
     * @brief Get the pose of the node in the maze frame.
     *
     * @param cell_size The size of the cells in meters.
     * @return The pose in meters and radians.
     */
    constexpr Pose to_pose(float cell_size) const {
        return {
            .position =
                {.x = static_cast<float>(this->point.x) * cell_size / 2.0F,
                 .y = static_cast<float>(this->point.y) * cell_size / 2.0F},
            .orientation = this->angle(),
        };
    }

    /**
     * @brief Compare two nodes for equality.
     *
     * @param other The other node to compare.
     * @return True if the nodes are equal, false otherwise.
     */
    constexpr bool operator==(const LatticePose& other) const = default;

    LatticePoint point;
    uint8_t      heading;
};

/**
 * @brief Place of a turn in the lattice, for the turn to the left in its canonical frame.
 *
 * @details The canonical frame has the entry node at the origin, on a vertical wall, heading along
 * +x for the turns that start on a straight and along the diagonal between +x and +y for the ones
 * that start on a diagonal. The gates are the walls crossed between the entry and the exit nodes,
 * in the order they are crossed.
 */
struct TurnPrimitive {
    /**
     * @brief Largest number of walls a turn crosses between its entry and exit nodes.
     */
    static constexpr uint8_t max_gates{2};

    bool                                diagonal_entry;
    uint8_t                             rotation;
    LatticePoint                        exit;
    std::array<LatticePoint, max_gates> gates;
    uint8_t                             number_of_gates;
};

/**
 * @brief Place of every turn in the lattice, indexed by TurnId.
 */
inline constexpr std::array<TurnPrimitive, number_of_turns> turn_primitives{{
    {.diagonal_entry = false, .rotation = 2, .exit = {.x = 1, .y = 1}, .gates = {}, .number_of_gates = 0},
    {.diagonal_entry = false,
     .rotation = 2,
     .exit = {.x = 3, .y = 3},
     .gates = {{{.x = 2, .y = 0}, {.x = 3, .y = 1}}},
     .number_of_gates = 2},
    {.diagonal_entry = false,
     .rotation = 4,
     .exit = {.x = 0, .y = 2},
     .gates = {{{.x = 1, .y = 1}, {}}},
     .number_of_gates = 1},
    {.diagonal_entry = false,
     .rotation = 1,
     .exit = {.x = 3, .y = 1},
     .gates = {{{.x = 2, .y = 0}, {}}},
     .number_of_gates = 1},
    {.diagonal_entry = false,
     .rotation = 3,
     .exit = {.x = 2, .y = 2},
     .gates = {{{.x = 2, .y = 0}, {.x = 3, .y = 1}}},
     .number_of_gates = 2},
    {.diagonal_entry = true,
     .rotation = 1,
     .exit = {.x = 1, .y = 3},
     .gates = {{{.x = 1, .y = 1}, {}}},
     .number_of_gates = 1},
    {.diagonal_entry = true,
     .rotation = 3,
     .exit = {.x = -2, .y = 2},
     .gates = {{{.x = 1, .y = 1}, {.x = 0, .y = 2}}},
     .number_of_gates = 2},
    {.diagonal_entry = true,
     .rotation = 2,
     .exit = {.x = 0, .y = 2},
     .gates = {{{.x = 1, .y = 1}, {}}},
     .number_of_gates = 1},
    {.diagonal_entry = false, .rotation = 1, .exit = {.x = 2, .y = 0}, .gates = {}, .number_of_gates = 0},
    {.diagonal_entry = false, .rotation = 1, .exit = {.x = 1, .y = 1}, .gates = {}, .number_of_gates = 0},
    {.diagonal_entry = false, .rotation = 3, .exit = {.x = 1, .y = 1}, .gates = {}, .number_of_gates = 0},
    {.diagonal_entry = true, .rotation = 1, .exit = {.x = 1, .y = 1}, .gates = {}, .number_of_gates = 0},
    {.diagonal_entry = true, .rotation = 0, .exit = {.x = 2, .y = 0}, .gates = {}, .number_of_gates = 0},
    {.diagonal_entry = true, .rotation = 7, .exit = {.x = 2, .y = 0}, .gates = {}, .number_of_gates = 0},
    {.diagonal_entry = true, .rotation = 6, .exit = {.x = 2, .y = 0}, .gates = {}, .number_of_gates = 0},
    {.diagonal_entry = true, .rotation = 5, .exit = {.x = 1, .y = -1}, .gates = {}, .number_of_gates = 0},
}};

/**
 * @brief Get the place of a turn in the lattice.
 *
 * @param turn The turn.
 * @return The primitive of the turn, for the turn to the left in its canonical frame.
 */
constexpr const TurnPrimitive& get_primitive(TurnId turn) {
    return turn_primitives.at(std::to_underlying(turn));
}

/**
 * @brief Move a displacement from the canonical frame of a turn to the frame of an entry node.
 *
 * @param canonical The displacement in the canonical frame of the turn to the left.
 * @param diagonal_entry Whether the canonical heading is diagonal.
 * @param heading The heading of the entry node.
 * @param side The side the turn is made to.
 * @return The displacement in the lattice, relative to the entry node.
 */
constexpr LatticePoint
    from_canonical(const LatticePoint& canonical, bool diagonal_entry, uint8_t heading, TurnSide side) {
    LatticePoint point = canonical;

    if (side == TurnSide::RIGHT) {
        point = diagonal_entry ? LatticePoint{.x = canonical.y, .y = canonical.x} :
                                 LatticePoint{.x = canonical.x, .y = static_cast<int8_t>(-canonical.y)};
    }

    for (uint8_t quarter = 0; quarter < heading / 2; quarter++) {
        point = {.x = static_cast<int8_t>(-point.y), .y = point.x};
    }

    return point;
}

/**
 * @brief Get the node where a turn ends.
 *
 * @param entry The node where the turn starts.
 * @param turn The turn.
 * @param side The side the turn is made to.
 * @return The exit node of the turn.
 */
constexpr LatticePose get_turn_exit(const LatticePose& entry, TurnId turn, TurnSide side) {
    const TurnPrimitive& primitive = get_primitive(turn);
    const uint8_t        rotation =
        side == TurnSide::LEFT ? primitive.rotation : LatticePose::number_of_headings - primitive.rotation;

    return {
        .point = entry.point + from_canonical(primitive.exit, primitive.diagonal_entry, entry.heading, side),
        .heading = static_cast<uint8_t>((entry.heading + rotation) % LatticePose::number_of_headings),
    };
}

/**
 * @brief Get one of the walls a turn crosses between its entry and exit nodes.
 *
 * @param entry The node where the turn starts.
 * @param turn The turn.
 * @param side The side the turn is made to.
 * @param index The index of the gate, in the order the gates are crossed.
 * @return The midpoint of the wall.
 */
constexpr LatticePoint get_turn_gate(const LatticePose& entry, TurnId turn, TurnSide side, uint8_t index) {
    const TurnPrimitive& primitive = get_primitive(turn);

    return entry.point + from_canonical(primitive.gates.at(index), primitive.diagonal_entry, entry.heading, side);
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_LATTICE_HPP
