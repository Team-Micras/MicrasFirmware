/**
 * @file
 */

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "micras/nav/grid_pose.hpp"
#include "micras/nav/maze.hpp"
#include "test_host.hpp"

using namespace micras::nav;

namespace {
constexpr std::array<GridPoint, 1> goal{{{.x = 2, .y = 2}}};

TMaze<4, 4> make_maze() {
    return TMaze<4, 4>{{.start = {.position = {.x = 0, .y = 0}, .orientation = Side::UP}, .goal = goal}};
}
}  // namespace

int main() {
    TMaze<4, 4> maze = make_maze();
    uint32_t    last = maze.get_revision();

    // --- a wall recorded changes the revision, one already known does not ---
    CHECK(maze.set_wall({.position = {.x = 1, .y = 1}, .orientation = Side::UP}, true));
    CHECK(maze.get_revision() > last);
    last = maze.get_revision();
    CHECK(not maze.set_wall({.position = {.x = 1, .y = 1}, .orientation = Side::UP}, false));
    CHECK(maze.get_revision() == last);

    // --- a reset never takes it back, even to where a map with as many walls would leave it ---
    const std::vector<uint8_t> saved = maze.serialize();
    maze.reset();
    CHECK(maze.get_revision() > last);
    last = maze.get_revision();

    // --- loading a map changes it too, and so does loading the same one again ---
    maze.deserialize(saved.data(), static_cast<uint16_t>(saved.size()));
    CHECK(maze.get_wall({.position = {.x = 1, .y = 1}, .orientation = Side::UP}) == WallState::WALL);
    CHECK(maze.get_revision() > last);
    last = maze.get_revision();
    maze.deserialize(saved.data(), static_cast<uint16_t>(saved.size()));
    CHECK(maze.get_revision() > last);

    // --- a record of another size is ignored and changes nothing ---
    last = maze.get_revision();
    maze.deserialize(saved.data(), static_cast<uint16_t>(saved.size() - 1));
    CHECK(maze.get_revision() == last);

    std::puts("maze ok");
}
