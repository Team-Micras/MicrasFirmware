# Changes to MicrasFirmware

Every change this repository made to the firmware submodule
(`targets/micras/MicrasFirmware`, branch `high-level-review` at `0be27df`), why it
was made, the evidence for it, and what it means on the real robot. They are
uncommitted in the submodule; this file is the record.

A change goes in only when it is an error that the simulation demonstrated and that
would affect the real robot the same way, or a constant set from a measurement. No
change was made to make the simulation pass where the robot would behave differently.

## Accessors the simulator needs

**Files:** `include/micras/micras.hpp`, `src/micras.cpp`

`Micras::get_instance()`, `get_variables()` and `get_state()`: read-only access for
the variable columns, the state events and the stop conditions (spec 2, D5). The
firmware's own `main` owns its `Micras`, so the constructor records a pointer to the
last one built. No behaviour changes.

## IMU axes in `Micras::measure()`

**Files:** `src/micras.cpp`, `tests/src/nav/test_controller.cpp`, `tests/src/nav/test_localizer.cpp`

`measure()` read the chip's X and Y accelerations as the robot's. The LSM6DSV is
mounted turned by 90 degrees (the CAD: robot x is chip Y, robot y is minus chip X),
so the robot's forward acceleration was taken from its lateral axis. The localizer
uses it to scale the longitudinal slip noise, and `check_crash` its magnitude.

## Motor dead zone 15 -> 0

**File:** `config/targets/v1/target.hpp` (`locomotion_config`)

The controller's feed-forward already adds the robot model's static friction
voltage, so the proxy's dead zone counted it twice and put a step of its size into
every command crossing zero; the motors chattered around standstill.

## Physical constants in `robot.hpp`

**File:** `config/targets/v1/robot.hpp`

Restated from the CAD, the board and the datasheets (spec 2 section 14): the drive
(DCX 8 M, 12 ohm plus 0.62 ohm, gear 5.25, 19.63 V), the geometry and the notes on
where each value comes from.

## Wall sensors: the receiver sits beside the emitter

**Files:** `micras_proxy/include/micras/proxy/wall_sensors.hpp`,
`micras_proxy/include/micras/proxy/impl/wall_sensors.tpp`, `config/targets/v1/target.hpp`

**Error.** The proxy turned a reading into a range with the inverse square law
calibrated at one distance. Each emitter lens sits 6.5 mm above its receiver lens
(SolidWorks), and the TPS601A halves its sensitivity 10 degrees off axis
(datasheet). So the receiver sees the lit spot 9 degrees off its axis from the
centre of a cell facing a wall, but only 4 degrees off at 100 mm. The reading falls
more slowly than 1/d^2, and the ranges came out 22 mm short at 100 mm and 7 mm long at
30 mm. The localizer trusted them to about 3 mm, so every approach to a front wall
dragged the pose estimate forward.

**Change.** Two constants in `WallSensors::Config`: `receiver_offset` (6.5 mm) and
`receiver_half_angle` (10 deg). The model is
`reading = k * 2^(-(atan(offset / d) / half_angle)^2) / d^2`, and the single-point
calibration fixes `k` as before. Its shape is tabulated once at construction, from
its peak (about 30 mm) to the maximum distance, and inverted by binary search, so an
update costs no transcendental function. With `receiver_offset = 0` the proxy
computes exactly the old formula.

**Evidence.** `just micras wall-calibration --sweep`: from 30 to 100 mm the front
sensors now follow the true range within 1 mm, where the old law was 22 mm off. The
side sensors are within 1 mm across the corridor.

**On the robot.** The same geometry gives the same bias. A bench sweep toward a wall
would confirm the offset and half angle; the calibration procedure is unchanged.

## Wall model: minimum range 20 -> 35 mm

**File:** `config/constants.hpp` (`wall_model_config`)

Closer than about 30 mm the reading of a sensor peaks and falls again, so one reading
fits two ranges. Ranges under 35 mm are no longer used.

## Localizer: ranges correct the pose only out to 120 mm

**Files:** `micras_nav/include/micras/nav/localizer.hpp` (new `Config::max_range`),
`micras_nav/include/micras/nav/impl/localizer.tpp`, `config/constants.hpp`

**Error.** The emitter beam is 11.75 mm above the floor and a few degrees wide.
Farther out, part of it lands on the floor before the wall, the reading loses light
the sensor model does not know about, and the range comes out long:

| true range | front sensors read |
|---|---|
| 120 mm | +5 mm |
| 150 mm | +16 mm |
| 180 mm | +45 mm |
| past 190 mm | nothing |

The side sensors read 12 to 16 mm long past 150 mm. The localizer used ranges out to
250 mm, and a long range from the right diagonal sensor pulled the pose 9 mm back in
the first corridor of maze 1.

**Change.** A new `max_range` in `Localizer::Config`, set to 0.12 m. Ranges beyond
it do not correct the pose. The wall model keeps its own maximum for mapping.

## Wall observer: walls voted absent only within 130 mm

**File:** `config/constants.hpp` (`mission_config.observer.detection_range` 0.18 -> 0.13)

**Error.** With the long-range readings above, a wall 180 mm ahead reads about 45 mm
long, beyond the observer's tolerance of 42 mm there, and it was voted **absent**.
On maze 1 the wall closing the first corridor was cleared from the map this way, and
the robot then corrected its pose against the wall behind it and drove into it at
5.3 s.

**Change.** Absence votes only within 130 mm, where a range reads at most 8 mm long
against a tolerance of about 35 mm. Votes for a wall still count at any range.

## The run waits for the fan

**Files:** `micras_proxy/include/micras/proxy/fan.hpp`, `micras_proxy/src/fan.cpp`
(new `Fan::is_at_speed`), `include/micras/micras.hpp`, `src/micras.cpp` (new
`Micras::is_prepared`), `include/micras/states/wait.hpp`, `src/states/wait.cpp`

**Error.** A solve with the fan switch on is planned with the fan's downforce in the
traction. The fan ramps up at `max_acceleration` = 0.02 %/ms, 5 s to full speed, but
`WAIT_FOR_RUN` starts the run after 3 s. The robot left with 0.25 N of the 0.6 N of
downforce it was planned for, about 73 % of the planned grip, slipped at the first
acceleration and crashed 0.4 s into the run.

**Change.** `WaitState` also waits for `Micras::is_prepared()`, true once the fan
has reached the speed it was asked for; a fan that is off always has. Runs without the
fan are unaffected.

## Localizer: a wall edge is timed by the reading, and the map only names it

**Files:** `micras_nav/include/micras/nav/localizer.hpp` (`EdgeTracker`, new `find_edge`),
`micras_nav/include/micras/nav/impl/localizer.tpp` (`track_edge`, `find_edge`),
`micras_nav/include/micras/nav/wall_model.hpp` and `micras_nav/src/wall_model.cpp`
(`PlaneCrossing` gains the `slope` of the axis to the wall)

**Error.** The edge tracker is the only reference along the path in a corridor. It
takes the moment a diagonal sensor's reading changes between "on a wall" and "past its
end" as a known position. But its "on the wall" also needed the axis, cast from the
estimated pose, to meet the wall in the map. So whichever of the two changed first made
the event:
- When the estimate was ahead of the robot, the map's change came first. That event
  carries no information: its innovation is only the offset the model expects. The
  leaving-a-wall branch dropped such events, but then the reading's real change, which
  came later, was never looked at. The arriving-at-a-wall branch used them.
- So exactly when the pose ran ahead, which fast runs do, the robot had no reference
  along the path.

The model of where a change happens was also wrong at a wall start. Moving toward a
wall start, the beam, which always points ahead, also lights the end face of the wall.
The reading therefore comes on early by about the wall thickness times the slope of the
axis to the wall. Leaving a wall, that face is in the shade.

Traced against the ground truth on three mazes (the diagonal sensors cross the plane of
the wall 98 mm out):
- A start was seen with the axis 13.5 mm (sd 7) short of the wall.
- An end was seen with the axis 2.6 mm (sd 2.8) past it.
- The corrections the tracker accepted raised the error along the wall from 2 to 5 mm
  to 5 to 8 mm on average. One of them moved the pose 10 mm the wrong way on apec2017's
  return and the robot parked into the back wall.

An earlier change here moved the expected end 16.6 mm inside the wall (`edge_inset`).
It was calibrated on innovations that included the estimate-triggered events, and it
is withdrawn.

**Change.**
- A sensor locks on a line of walls once the map and the reading agree it is on one.
  From then on, only the reading, compared with the plane of that line, makes an event.
- The map says which edge it was. `find_edge` follows the line from the locked wall in
  the direction of travel to the first end, or to the first start after a gap, up to
  four cells. It gives up if a wall on the way is unknown.
- The expected axis is on the edge itself. A start is pushed out by the wall thickness
  times `|slope|`.
- The window, the gate and the 10 mm cap are unchanged.

**Evidence.** On the ten saved maps, the fast run with diagonals touched a wall on
apec2017 and japan2017ef, both after 25 mm of error along the path. With this change it
is clean on all ten, at the same times. Exploration, return and the fan-only fast run
stay clean on all ten.

**On the robot.** The logic error is independent of the physics. The end-face effect
depends on the real sensor and wall; a sweep past a wall start at a known pose measures it.

## Controller: the time scale settles instead of alternating

**Files:** `micras_nav/src/controller.cpp`, `micras_nav/include/micras/nav/controller.hpp`

**Error.** When the feed-forward asks for more voltage than the supply less its reserve
gives, the controller slows the clock of the reference (`time_scale`), so the robot
stays on the path and only takes longer. It computed the scale as
`available / demand` from the reference it was handed. But that reference had
already been played at the previous scale, with speeds times `s` and accelerations
times `s^2`. So a scaled reference always looked affordable, the scale jumped back to
1, the next reference was unaffordable again, and the scale alternated between 1
and a fraction every 125 us.

**Change.** The scale is corrected from the one in force:
`time_scale = min(1, time_scale * available / demand)`.

**Evidence.** Driving the firmware's `Controller` with a reference that needs 19.1 V
against 16.7 V available (`runs/timescale/demo.cpp`, which applies the executor's
scaling):
- Before: the scale alternated 1.000, 0.875, 1.000, 0.875, and so on.
- After: 0.875, 0.924, 0.904, 0.912, 0.909, 0.910, settling where the demand meets
  16.7 V.

The runs so far never reached the limit (peak demand 8.4 V), but with the fan's real
downforce the traction allows accelerations the motors cannot always follow.

## Robot model: fan downforce 0.6 N -> 3 N

**File:** `config/targets/v1/robot.hpp` (`robot_model.traction.fan_downforce`)

The owner's figure for the fan at full speed is about 3 N; 0.6 N was an estimate.
`robot.toml` uses the same value, so what the firmware plans with and what the
simulation applies agree.

## Crash detection: above what the tires can transmit

**File:** `config/constants.hpp` (`crash_acceleration`)

**Error.** `check_crash` calls it a crash when the horizontal acceleration stays over
35 m/s^2 for 5 ms. With the fan, a fast run plans lateral and forward accelerations of
up to `utilization` times the traction, 32 m/s^2 in the normal profile with 3 N of
downforce and 40 m/s^2 with boost. The feedback and the IMU sitting off the axis of
rotation add to that. Every simulated fast run with the fan ended in ERROR 0.3 s after
it started, on the acceleration out of its first curve (35.8 m/s^2 measured), having
touched nothing.

**Change.** `crash_acceleration` is 1.25 times `robot_model.traction_acceleration(true)`,
the most the tires can transmit. Anything above that has to come from a wall. That is
49 m/s^2 with the traction below. A crash at search speed was below either threshold,
and a crash at 1.5 m/s or more is well above both.

## Traction: the nose carries part of the downforce

**Files:** `micras_nav/include/micras/nav/robot_model.hpp` (new `Traction::fan_offset`,
new `RobotModel::tire_downforce`), `config/targets/v1/robot.hpp`

**Error.** The traction model added the whole fan downforce to the tires. The fan pulls
17.5 mm ahead of the axle, over its hole. A thin-gap flow estimate (pressure harmonic
between the open edges of the board and the hole) puts the centre of the suction at 14
to 16 mm. The centre of mass is 0.8 mm behind the axle, so from a few tens of
millinewtons of downforce on, the robot tips onto the front edge of its board. The
edge then carries `offset / front_length` of the pull, a third of it. The tires carry
2 N of the 3 N, and the robot planned with 23 % more grip than it had. The normal
profile used 81 % of the real grip rather than 60 %, and boost asked for more than
all of it. In the simulation the tires slid at 0.2 m/s in every fast curve, and the
pose estimate drifted 15 to 25 mm ahead of the robot.

**Change.** `Traction` gains `fan_offset` (0.0175 m), and the traction uses
`tire_downforce() = fan_downforce * (1 - fan_offset / front_length)`.

**Evidence.** Maze 1 solve, normal profile with the fan:
- Lateral acceleration in the curves: 30 to 23.5 m/s^2 (median).
- Tire slip in the curves: 0.21 to 0.13 m/s (median).
- Worst pose error along the run: 22 to 14 mm.

**On the robot.** It follows from the geometry, as long as nothing but the board edge
carries the nose. Kitchen scales under the wheels and under the nose, with the fan
running, measure the share directly. The edge also drags, with the friction of FR-4 on
the floor, which the firmware does not model.

## Search speed: 0.35 and 0.4 m/s -> 0.5 and 1 m/s

**File:** `config/constants.hpp` (`search_profile`)

A tuning constant, not an error. The search runs at half of the traction without the
fan, up to 1 m/s, where it used 35 % of the traction up to 0.4 m/s. On all ten mazes and
three noise seeds, the exploration and return end 10 to 60 s earlier (maze 1: 137 to
104 s before the solve starts; apec2016: 176 to 115 s), with no collision. At 60 % of
the traction seven of ten mazes failed. Braking for a front wall that corrects the pose
late then asked for more than the tires give. So 50 % is the setting with a margin
under that.

**On the robot.** Worth confirming step by step. The margin measured here is against the
simulated tires, and the real ones set the limit.

## Wall observer: keeps voting through the search turns

**File:** `config/constants.hpp` (`mission_config.observer.max_angular_speed` 2 -> 12 rad/s)

**Error.** The search stopped in the middle of a cell and spun in place about 25 times
on maze 1, and at 12 dead ends. A dead end needs it; the 90-degree spins did not. The
observer ignored the sensors while the robot turned faster than 2 rad/s. The search turns
at 8 rad/s, and the side wall of the cell a turn ends in is only in sight during that
turn. So after a turn the robot reached the next cell's entry, where it decides, before
the observer had decided that wall. Traced: the votes on that wall began 6 ms before the
decision and completed 1 ms after it. The robot then had to stop, spin 90 degrees and
set off again, 0.37 s each time.

**Change.** The limit is 12 rad/s, above the search turns. The pose used for the cast
already accounts for the rotation during the range delay.

**Evidence.** On all ten mazes, the maps are the same (every fast run takes exactly the
same time). Exploration and return take 12 to 39 s less (maze 1: 104 to 86 s before
the fast run starts; apec2017: 138 to 99 s), with no collision.

**On the robot.** If the real readings lag the rotation more than the filter delay says,
walls seen in a turn could be misjudged. A wrong wall shows up as a different route, so
compare the map of a search with a slow one.

## Localizer: the wheels roll on a radius the load flattens

**Files:** `micras_nav/include/micras/nav/robot_model.hpp` (new
`Chassis::rolling_compliance`, new `RobotModel::rolling_radius`),
`micras_nav/include/micras/nav/localizer.hpp`, `micras_nav/src/localizer.cpp` (new
`set_downforce`), `src/micras.cpp`, `config/targets/v1/robot.hpp`

**Error.** The odometry used the unloaded wheel radius. The tire band flattens under the
load and the wheel rolls on a smaller radius. With the fan the tires carry four times
the load, so the estimate ran ahead of the robot by 0.67 % of the distance, 20 mm over
3 m. The 3 m/s straights of a fast run with no wall edge on them, such as the bottom row
of maze 2, then reached their turn 25 to 35 mm off.

**Change.** `rolling_compliance` is how much the rolling radius shrinks per newton on a
tire. The localizer rolls on `rolling_radius(downforce)`, the weight plus the fan's
share on the tires, split between two tires. `Micras` tells it the fan's share of its
downforce each iteration, from the fan's current speed squared.

**Evidence.** The simulated wheels overrun the body by 0.17 % with the fan off and by
0.67 % with it on, at 0.34 N and 1.36 N on each tire. Both give 56 um of radius per
newton, whatever the timestep.

**On the robot.** The 56 um/N is the simulated band. Driving a known distance with the
fan on and with it off measures the real one; the wheel radius is then the one with the
fan off.

## Boost: 0.75 -> 0.65 of the traction

**File:** `config/constants.hpp` (`boost_utilization`)

A tuning constant, set from what the tires hold in a turn. In a turn the tires slide
sideways in proportion to the grip asked of them, and the pose estimate, which assumes
the wheels do not slip sideways, does not see it. At 0.75 a boosted turn at 30 m/s^2 put
the robot 10 to 20 mm off its path. On the ten saved maps, with the edge tracker and the
rolling radius above, the boosted fast run failed on:

| boost | fan, diagonal, boost | and risky turns |
|---|---|---|
| 0.75 | 3 of 10 | 5 of 10 |
| 0.70 | none | 1 touch (apec2018) |
| 0.65 | none | none |

At 0.65, boost is 3 to 5 % faster than the normal 0.6 (maze 1: 4.32 s with diagonals,
4.16 s with boost and risky turns).

**On the robot.** Part of the sideways slide in the simulation comes from MuJoCo: a
sliding contact pushes its surfaces apart, and the light inner wheel of a turn hops. The
real limit may be higher. Raise it step by step while watching the error across the path
at the exit of the turns.

## Localizer: a range is corrected for the angle it meets the wall at

**Files:** `micras_nav/include/micras/nav/wall_model.hpp`, `micras_nav/src/wall_model.cpp`
(new `WallModel::get_range`), `micras_nav/include/micras/nav/impl/localizer.tpp`

**Error.** A wall reflects diffusely, so the light a sensor gets back falls with the
cosine of the angle between its axis and the perpendicular of the wall. The proxy turns a
reading into a distance as if the wall were met at the angle of the calibration: square on
for the front sensors, at 45 degrees for the diagonal ones beside the walls of a
corridor. Along a diagonal the diagonal sensors meet the walls square on, and the reading
comes out short by the square root of the ratio of the cosines, 16 %. The localizer took
those ranges as they were. On apec2017, fan and diagonals, twelve readings of one sensor
in 6 ms, all short by 4 to 14 mm, pulled the estimate 12 mm off at 3 m/s, the controller
yanked the robot back, the wheels left the floor and the crash detection stopped the run.

**Change.** `WallModel::get_range` turns a reading back into the range along the axis,
from the cosine of the hit and the one of the calibration, and the localizer compares
that with the wall model.

**Evidence.** The largest error of the estimate along the path on the diagonals of the
fast runs of ten mazes went from 18 to 28 mm to 10 to 16 mm, and across it from 5 to
13 mm to 1 to 8 mm. With the routes of the turns below, which run long diagonals at
3 m/s, five of the ten mazes collided or stopped without it and none with it. The fast
runs of the eight turns take the same time as before.

**On the robot.** The real walls are matte, so the same is expected. A diagonal sensor
facing a wall square on and then at 45 degrees, at the same distance, should read about
16 % nearer square on; the ratio measures how diffuse the real walls are.

## Planner: labels, dropped only when another is as fast for every route

**Files:** `micras_nav/include/micras/nav/planner.hpp`,
`micras_nav/include/micras/nav/impl/planner.tpp`, `micras_nav/include/micras/nav/explorer.hpp`,
`micras_nav/include/micras/nav/impl/explorer.tpp`, `micras_nav/include/micras/nav/mission.hpp`,
`micras_nav/include/micras/nav/impl/mission.tpp`, `src/micras.cpp`, `config/constants.hpp`
(`plan_edges_per_iteration`, `search_edges_per_iteration`)

Not an error: asked for by the owner, to make room for the turns below without missing
the best route. The search held an array entry for every wall, heading and turn that could
reach it, 13,056 entries and 144 KB with eight turns, growing with each turn: 370 KB with
nineteen. It now holds labels, a way of reaching a node with its cost and the speed of the
turn that reached it, in a pool of 6144. A new label is dropped when another at the same
node is at least as fast for every continuation of the route, and a label waiting in the
queue is freed when a new one is. The test compares the time before the turn of each and
the speed of that turn, and for two different turns a bound on how much slower the first
edge after one can be than after the other, computed for every pair before the search. A
turn driven slower is compared as if at full speed less the time it lost, since it never
makes a route faster. If the pool fills, the costliest waiting labels are dropped and
`is_exact()` says so.

The search now advances by edges tried rather than nodes expanded, since a node tries
three to four times as many turns as before: 512 edges per iteration while planning a fast
run and 16 while searching.

**Evidence.** On 5,490 searches (the ten mazes whole and with 30 random parts of their
walls unknown, the nine profiles, both wall assumptions) the best route took the same time
as with a search that drops a label only for another of the same turn, in all 2,799 that
have a route. With the eight turns alone it also took the same time as the old planner,
and with every turn it was never slower and faster in 287. The most labels held at once
were 4,519. The fast runs of ten mazes with every switch on took the same time as before
the change; the searches were 14 s shorter in total, because the explorer, which visits
the walls of the few best routes, gets a slightly different list of them.

A search on a PC takes six times as long as with the old planner, with every turn. The
budget in edges keeps an iteration near what one node cost before, so the answers come
later: the searches of ten mazes took 4 % longer in total with 16 edges per iteration, 17 %
with 8 and 2 % with 32.

**On the robot.** The time of an iteration while searching is not simulated. Watch
`loop_worst_time_us` during a search and set `search_edges_per_iteration` from it. The
planner takes 151 KB, against 163 KB before.

## Turns of two bends: eight turns inside one cell

**Files:** `micras_nav/include/micras/nav/lattice.hpp` (eleven new `TurnId`,
`first_two_bend_turn`), `micras_nav/include/micras/nav/turn_table.hpp` (`TurnBend`,
`TurnShape` of up to two bends, `TwoBendDesign`, `TurnTable::place`, `check`, `clears`),
`micras_nav/include/micras/nav/segment.hpp`, `micras_nav/src/route_compiler.cpp`,
`micras_nav/include/micras/nav/impl/mission.tpp`, `config/two_bend_turns.hpp` (new, written by
the simulator's `just micras turn-designs`), `config/constants.hpp`

Not an error: asked for by the owner. The lattice had no way onto a diagonal inside one
cell, nor from one diagonal to the next, so short diagonals were driven as two turns of
90 degrees. The eight new turns enter a cell and leave it through one of its other three
walls, with one of the three headings that cross it: SD45E, SD45T, SD135T from a straight,
and DS45T, DD0E, DS45E, DD90E, DS135B from a diagonal. Each is two bends joined by
a straight, driven at one speed, the one its tighter bend allows: 1.4 to 1.8 m/s with the
fan at full traction.

Two more, DD90T and DD90B, turned 90 degrees from a diagonal inside the cell, with bends
of 37 mm at 1.2 m/s. No best route took them, of 2,799 on the ten mazes and 9,666 on 150
random ones, and without them every route took the same time and the planner a sixth less:
each only ends half a step before DD90 would, and the slower bends cost more than the half
step saves. They were taken out, and so was DD180B, for the reason below.

A turn of two bends is too long a search for the compiler, so the simulator's turn
designer searches a grid of the angle of the first bend, the curvature of each and the
straight before them, and keeps the fastest that the firmware's own check accepts. The
compiler places each design between its nodes, checks that its bends turn the angle of
the turn and, in a `static_assert`, that the robot clears every post and every wall the
turn does not cross by the margin, every 5 mm of the path. A change to the robot or the
maze that makes one no longer fit stops the build. The segment of a turn now carries the
length of its curve, negative for the turn to the right, since a turn of two bends can end
at the heading it started with.

The check of the turns also collected at most 48 obstacles around a turn and silently
dropped the rest. The eight turns of one bend come out the same with 96, but one of the
new ones did not: DD90E was designed against a list that missed walls.

**Evidence.** The fast runs of ten mazes with every switch on:

| | total | maze2 | apec2017 | apec2018 | japan2017ef |
|---|---|---|---|---|---|
| eight turns | 64.70 s | 4.11 s | 7.65 s | 6.21 s | 8.09 s |
| with the new turns | 62.42 s | 3.75 s | 7.16 s | 5.81 s | 7.63 s |

No collision in any of 60 runs with them: the ten mazes with seeds 1, 2 and 3, the fast
runs with the fan and diagonals, with and without boost, and the fan alone, which does not
use them and takes the same time as before.

DD180B, a U-turn of one cell from a diagonal, turned around the post at the corner of the
walls it enters and leaves through, arriving on a diagonal that leads away from that post.
If the robot came along that diagonal from farther than the cell before, it came through
the cell the diagonal it leaves on goes to next, and turning around there is shorter. If it
only swung onto that diagonal in the cell before, it could come in straight and turn 135
degrees (SD135), or swing toward the post and turn 90 (DD90), instead of 225 in all. On a
maze built for it, a corridor along a fence whose end leads into a diagonal back under the
fence, the planner took it only once seven other turns were forbidden, and the run was then
9.7 % slower than with SD135. Without it the planner took 9 % less time, and every route
the same.

Four turns that replace two turns in a row with one curve were also tried: on top of these
they gain 0.66 % of the planned time, and would take 4 gates per turn, a larger pool of
labels and a third more planning. They were left out.

**On the robot.** These are the tightest turns the robot drives, down to 50 mm of radius,
and the simulation tracked them within the margin only once the ranges on diagonals were
corrected (above). Try them first with the fan, diagonals and neither boost nor the risky
switch, and compare the pose at the exit of each with the estimate.

## Turns braked and accelerated through, as their curvature allows

**Files:** `micras_nav/include/micras/nav/curve_speed.hpp` and `impl/curve_speed.tpp`,
`micras_nav/src/curve_speed.cpp` (new), `micras_nav/include/micras/nav/motion_limits.hpp` and
`micras_nav/src/motion_limits.cpp` (`CurveLimits`, `Dynamics::get_curve_limits`),
`micras_nav/include/micras/nav/turn_table.hpp` (`Bending`, `bending_at`),
`micras_nav/src/velocity_planner.cpp`, `micras_nav/src/executor.cpp`,
`micras_nav/include/micras/nav/segment.hpp` (the unused `max_speed` is gone)

Not an error: asked for by the owner. A turn was driven at one speed, the one its tightest
point allows, so its start and end speeds were equal and two turns too close together were
both slowed to what the straight between them could change. Now a turn is sampled every
5 mm, each sample gets the speed its curvature and sharpness allow, and passes from the start
and from the end lower it to what can be reached and braked with the grip the curve leaves:
the lateral part takes its share as on a friction circle, the angular part from the push each
tire has left, and a change of speed on a curve also counts for the angular acceleration it
causes. The velocity planner gives a turn different speeds at its two ends, and the executor
plays the samples back with a constant acceleration between two of them. The planner still
prices a turn at one speed; the four best routes are then timed with the new rule, so the
chosen one is exactly timed.

A single bend gains nothing: its ramps are as long as the ratio of the angular to the lateral
grip makes them, so the entry is already at the speed of the arc. What gains is a turn of two
bends of different curvature, the straight between two bends, and a turn next to another one.

**Evidence.** Fast runs on the ten saved maps, seed 1, no collision in any of them:

| | before | after |
|---|---|---|
| fan (diagonals always on) | 65.04 s | 64.04 s |
| fan, boost | 63.25 s | 62.19 s |
| fan, boost, risky | 62.42 s | 61.27 s |

**On the robot.** Nothing new is asked of the tires: a sample is never faster than a turn's
old speed where that speed was the limit. Watch the time of the turns of two bends.

## Localizer and controller: the tires slide to the outside of a curve

**Files:** `micras_nav/include/micras/nav/robot_model.hpp` (`Traction::lateral_compliance`),
`config/targets/v1/robot.hpp`, `micras_nav/src/localizer.cpp`, `micras_nav/src/controller.cpp`

An error the simulation demonstrated. A tire only pushes sideways by slipping, so in a curve
the robot slides to the outside, and neither the wheels nor the gyroscope see it. On every
fast run the speed across the heading follows the lateral acceleration with a correlation of
0.99: 4.8 mm/s per m/s^2, 50 to 100 mm/s in the turns of a boosted run, which put the
estimate 8 to 16 mm off. The prediction of the localizer now slides the pose by that much,
and the controller points the robot into the curve by the compliance times the angular
speed, the angle at which it moves along the path while sliding, instead of letting an
error across the path build up to turn it there.

**Evidence.** On the ten saved maps, fan and boost, the routes of turns:

| | estimate across, rms / worst | controller across, rms |
|---|---|---|
| before | 2.4 / 16.3 mm | 2.8 mm |
| slide in the localizer only | 2.5 / 16.4 mm | 5.7 mm |
| and in the controller | 2.3 / 13.3 mm | 2.8 mm |

The two go together: a localizer that sees the slide alone makes the controller fight it.
The routes of turns gain little; the racing line below needs it. At 0.8 of the grip and
15 mm the line hit a wall on 4 of 10 mazes without it, and on none with it.

**On the robot.** The compliance is the simulation's, and the real tires will differ. Drive
a circle of known radius at a few speeds with the fan on and compare the end pose with the
odometry: the difference across, over the time, is the slide speed.

## Racing line instead of the diagonal switch

**Files:** `micras_nav/include/micras/nav/racing_line.hpp` and `impl/racing_line.tpp`,
`micras_nav/include/micras/nav/line.hpp`, `micras_nav/src/line.cpp` (new),
`micras_nav/include/micras/nav/segment.hpp` (`SegmentKind::LINE`),
`micras_nav/src/executor.cpp`, `micras_nav/include/micras/nav/mission.hpp` and `impl/mission.tpp`,
`micras_nav/include/micras/nav/motion_limits.hpp` (`RunProfile::racing_line`),
`micras_nav/include/micras/nav/impl/planner.tpp`, `include/micras/interface.hpp`,
`src/micras.cpp`, `config/constants.hpp`, `README.md`; in the simulator, the switch is named
`racing_line` and `scenarios/*_all.toml` turn it on

Not an error: asked for by the owner. Diagonals are always allowed now, so the map is complete
for four profiles instead of eight, and the second switch selects the racing line: once the
route of a fast run is planned, it is sampled every 10 mm, and each sample slides sideways,
within what keeps the outline of the robot 15 mm from every post and from every wall the
route does not cross, to make the sum of the squared curvatures smallest. It is solved in
windows of 64 samples by a projected Newton method on the banded normal equations, twelve
sweeps, with a check of every sample at the end. The speeds follow the rule of the turns
above, with 0.8 of the lateral grip of the run. If no line is found, or it is not faster,
the robot drives the route. With the risky switch, a line also goes through the route of the
risky turns, and the robot drives it only if it is the faster of the two; the line keeps
15 mm wherever it can move, and only the risky turns come closer.

The first line, at the full grip and at the 10 mm margin of the risky turns, hit a wall on
5 of 10 mazes: it holds its curves at the limit far longer than a turn, and trail brakes into
them, where the tires slide more than the compliance above says. At 0.8 of the grip and
15 mm it is clean. At 10 mm everywhere it hit walls on 4 to 6 of 10 mazes. Through the route of
the risky turns, keeping 15 mm elsewhere, the line was slower than the other on 7 of the 9 mazes
it finished and crashed on apec2017; choosing the faster of the two planned lines, every switch on
is clean on all ten, 58.68 s, the risky line driven on alljapan-033-2012-exp-fin and japan2013ef.
Its time is known before the run, since the run follows the planned reference in time; whether it
crashes is not. With risky on the planning takes two lines, and three when the risky one wins,
since only one line is kept.

**Evidence.** Fast runs on the ten saved maps, fan and boost, seeds 1, 2 and 3, no collision:

| | route | racing line |
|---|---|---|
| total | 62.19 s | 58.74 s (-5.5 %) |
| maze1 | 4.04 s | 3.72 s |
| japan2017ef | 7.34 s | 6.71 s |
| uk2016f | 4.64 s | 4.47 s |

The optimization runs in the PLAN state, 16 samples per iteration: 40 to 170 ms of a PC for
routes of 8 to 21 m, 2,500 to 12,000 iterations, the costliest 0.2 to 0.35 ms. On the
Cortex-M7 that is about 1 to 3 s before the run and a few milliseconds per iteration at
worst, inside the 10 ms of the watchdog; this was not measured on the robot. The line takes 57 KB
of RAM, 51 KB of it the samples of up to 25.6 m of line, and about 18 KB of flash for the
Cortex-M7; a turn's samples add 1 KB to the executor. The robot object grows from 153 KB to
212 KB.

**On the robot.** The line spends most of its time turning faster than 2 rad/s, where the
localizer takes no wall correction, so it leans on the odometry, the gyroscope and the
compliance above. Try it first on a short maze at a low utilization, and compare the pose at
the goal with the estimate.
