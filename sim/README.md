# Micras in the simulator

`sim/` runs this firmware in [micras-simulation](https://github.com/Team-Micras/micras-simulation): its
own `main`, its own proxies and its own configuration, over the host backend of micras-lib's HAL and
its SPI chip models, against a MuJoCo model of the robot in a contest maze. Every run writes a CSV of
the whole state and a metadata file, and two runs of the same binary with the same arguments are byte
identical.

The simulator's own README explains the engine, the command line, the output, the analysis and the
viewer; micras-lib's README explains the host HAL, the SPI device slot and the chip models. This file
is what is particular to Micras: how the firmware is built here, the recipes, what the simulation
relies on in the app, and the measurements behind the numbers in `robot.toml`.

## Building and running

Both submodules are needed, and GCC 15 (`gcc-15`/`g++-15`), Ninja and Python 3 with numpy and
matplotlib:

```bash
git submodule update --init                   # external/micras-lib and external/micras-simulation
cmake --preset host
cmake --build --preset host
cmake --build --preset host --target sim_check
```

| Preset | Build | For |
|---|---|---|
| `host` | RelWithDebInfo, in `build/host/` | runs, the contest, the baselines and the tests |
| `host-ci` | `host` with no viewer and no video, and every `-Werror` switch on | the CI |

Both configure the root `CMakeLists.txt` with `MICRAS_LIB_HAL_BACKEND=host`, which dispatches to this
folder before anything of the robot build. The compiler is pinned because another one may move the
last bits of a run.

The sources of this folder are formatted with the rest of the firmware (`make format` in the robot
build) and linted by this build, which compiles them: `cmake --build --preset host --target lint`.

An arbitrary run calls the binary directly. From the repository root, with the runs next to the
recipes' in the ignored build directory:

```bash
runs=build/host/sim/runs
build/host/sim/micras_sim --scenario sim/scenarios/explore.toml --out $runs/x
build/host/sim/micras_sim --scenario sim/scenarios/explore.toml --seconds 30 --viewer --out $runs/window
build/host/sim/micras_sim --scenario sim/scenarios/explore_solve.toml --maze apec2017 \
    --out $runs/apec2017 --flash $runs/apec2017/flash.bin
build/host/sim/micras_sim --scenario sim/scenarios/solve.toml --flash $runs/apec2017/flash.bin --out $runs/solve
build/host/sim/micras_sim --scenario sim/scenarios/explore_link.toml --monitor --out $runs/link
python3 external/micras-simulation/tools/analyze.py $runs/x   # report.json and the plots
```

Micras adds one option to the simulator's:

| Flag | Meaning |
|---|---|
| `--flash <file>` | load the flash from the file before the run and save it back after, so a map the firmware saved survives into the next run |

## Recipes

Each is a CMake target over a script of `scripts/` that takes its paths as arguments
(`cmake --build --preset host --target <recipe>`). Runs land in `build/host/sim/runs/`.

| Target | Does |
|---|---|
| `sim_run_idle` | the robot switched on and left alone for 4 s -> `runs/idle` |
| `sim_run_explore` | the first 60 s of an exploration, with its flash -> `runs/explore` |
| `sim_contest` | the whole contest (`explore_solve`) in every maze at once, and its health |
| `sim_contest_all` | the same with every switch of the fast run on (`explore_solve_all`) |
| `sim_compare_baseline` | `runs/idle` and `runs/explore` against `baseline/` |
| `sim_record_baseline` | the tests, both runs, and the baseline, over the previous one |
| `sim_check_flash` | an exploration, then a run booted from the flash it saved |
| `sim_check` | the Micras gate: the tests, the flash, both checked runs, the baseline and the runs' health |
| `sim_robot_report` | `robot.toml` against the firmware's `robot.hpp`, field by field |
| `sim_wall_calibration` | each wall sensor's gain for `robot.toml` (`build/host/sim/micras_wall_calibration --sweep` shows the distances) |
| `sim_serve` | an exploration started over the link, open to micras-monitor on `ws://localhost:8080` |
| `sim_watch` | an exploration in a window |

| Cache variable | Default | Effect |
|---|---|---|
| `MICRAS_SIM_CONTEST_MAZES` | all ten mazes | the mazes `sim_contest` and `sim_contest_all` run in |
| `MICRAS_SIM_EXACT` | `OFF` | the baseline comparison also requires each `data.csv` to be byte identical to the recorded one |

The mazes are maze1, maze2, apec2016 to apec2019, japan2013ef, japan2017ef, uk2016f and
alljapan-033-2012-exp-fin, from the simulator.

## The gate and the baseline

`sim_check` builds the simulator and the tests, runs the tests of `tests/` through CTest, checks that
the flash outlives a run, runs the two checked scenarios (idle, and the first 60 s of an exploration),
compares them with `baseline/` and checks their health: no warnings, no collision, no
non-finite sample, no unbound port, no watchdog expiry, no emergency stop, no dropped byte.

The baseline holds one `summary.json` per checked run: the hash of its `data.csv`, the state
timeline, and a handful of numbers with their tolerances. On the machine that recorded it the hash
matches; on another the compiler, libm and the MuJoCo build move the last bits, and the summary is what
is compared, within its tolerances. Two rules:

- **A refactoring must not move a byte.** On one machine it leaves the hash where it was; configure with
  `-DMICRAS_SIM_EXACT=ON` to make the gate check it. If it moves, the refactoring is wrong.
- **Never re-record to make a difference go away.** A change of behavior is recorded over the baseline
  with `sim_record_baseline`, with the reason in the commit; git keeps the earlier ones.

The variable-pool names the scenarios, the baselines and the analysis plugin read (`state`,
`reference/linear_speed`, `pose/linear_speed`, ...) are the CSV's columns: renaming one is a baseline
change.

The viewer, video and bridge invariance checks, which prove that a window, a recording and a bridge
nobody connects to change nothing, run on the simulator's own toy target, in its CI.

## Scenarios

| Scenario | What it plays |
|---|---|
| `idle` | the robot switched on and left alone |
| `explore` | an exploration started by the button |
| `explore_link` | an exploration started over the radio |
| `explore_solve` | the whole contest: explore, come back, then a long press for the fastest run, with the fan switch on |
| `explore_solve_all` | the same with every switch of the fast run on: fan, racing line, boost and risky turns |
| `solve` | the fastest run alone, from a map a previous run saved: pass its `--flash` |
| `solve_all` | the same with every switch on |

The inputs a scenario can press or set are `button` and the four DIP switches (`dip_fan`,
`dip_racing_line`, `dip_boost`, `dip_risky`); the link commands it can send are `explore`, `solve`,
`calibrate`, `save` and `reset`, framed by the firmware's own `comm` encoder.

## How the firmware is built here

| Target | What |
|---|---|
| `micras_host_board` | `cube/`: the Micras v1 Cube layer, written by hand; micras-lib's HAL links it as its board target |
| micras-lib | `micras::core`, `nav`, `comm`, `hal` (host backend), `proxy` (ST's LSM6DSV driver included, as C) and `proxy_models` |
| `micras_app` | the application of `src/` and `config/` (`cmake/app_sources.cmake`, the robot's own list), with `main` compiled as `micras_firmware_main` |
| `micras_sim_micras` | `src/`: the simulator's `Target`, the bindings and the pool variables; links `micras_app`, `micras::proxy_models` and `micras::sim_app` |
| `micras_sim` | `src/main.cpp`: one call to `micras::sim::run` |
| `micras_wall_calibration`, `micras_robot_report` | `tools/`, over the engine and the firmware's configuration |
| `micras_turn_designer` | `tools/turn_designer.cpp`, over `micras::nav` and the configuration's headers only; the build runs it into `generated/two_bend_turns.hpp` whenever it is rebuilt, and so does the robot build, with a host project of its own (`cmake/turn_designer/`) |
| `micras_sim_tests` | `tests/`: the host HAL on the Micras board, the SPI slot on its buses and the firmware's SPI chip proxies over the chip models (doctest) |

Everything of the firmware compiles unchanged. The simulator's firmware thread runs
`micras_firmware_main`, and the host timer hands every 100 us over to the world.

### The Cube layer

The Cube headers the firmware includes (`main.h`, `tim.h`, `adc.h`, ...) are written by hand in
`cube/`, with the handles configured as CubeMX configures them on the board (timer clock 275 MHz, TIM4
center-aligned at PSC 274 / ARR 250, and so on), because the host `Pwm` and `Timer` compute
frequencies and duties from those registers. Each value names the generated line it mirrors in the
generated `cube/Inc` and `cube/Src`. A CubeMX change that renames a handle or a pin is a compile error
here; a changed prescaler or period shows as a wrong frequency at initialization.

### The bindings

Every port is found through the firmware's own `target.hpp`: `.handle = &htim4` there and
`Board::pwm(&htim4, ...)` in a binding reach the same port, so no handle is named in `src/`. The motors,
encoders, wall sensors, battery, fan, current sensing, radio, button and DIP switches become the
simulator's devices; the LED, buzzer, ARGB, microcontroller and flash are marked bound and read by the
panel.

The SPI chips are micras-lib's models, one per chip (`MicrasChips`), attached to `hspi3` by the chip
selects `target.hpp` names. Each sample of the simulator's IMU device goes back to rad/s and m/s^2 and
into `Lsm6dsvModel::push_sample` at the rate the firmware configures (8 kHz), which encodes it with the
full scale the firmware wrote. The encoders' position reaches the firmware through the timer encoder,
as on the robot.

Two timings follow from the real drivers. The `Imu` reads the burst it started one `update()` earlier,
so a sample reaches the firmware one loop after the update that asked for it. Its constructor waits
10 ms, then 30 ms after the software power-on reset, so the robot exists, and INIT starts, 40 ms into
the run. The CSV's first row waits for the firmware's variables, so a Micras CSV starts at tick 320.

`Micras` is a static of the firmware's `main`, destroyed when the process exits, after the target and
its chips; the IMU's `Spi` ends its last transfer then. `MicrasTarget::unwire` therefore forgets every
port, which detaches the chips.

## What the simulation relies on in the app

The firmware stays free to change, but these are what `sim/` reads, and a change to one is a change here
in the same commit:

- `main` is compiled as `micras_firmware_main`, never returns, and reads the timer every loop: the timer
  read is where the host clock hands the step over to the world.
- `Micras::get_instance()`, `get_variables()` and `get_state()`, for the variable columns, the state
  events and the stop conditions; `Micras::Command`, for the link commands; `micras::State` and
  `NUMBER_OF_STATES`, named by `micras::state_names` in `include/micras/states/names.hpp` (a
  `static_assert` pins each name, because the runs and the baselines record them, and a state with no
  name fails the build); `loop_time_us` and `wall_sensors_frequency` of `constants.hpp`.
- The variable-pool names the scenarios, the baselines and the analysis plugin use (`state`,
  `reference/linear_speed`, `pose/linear_speed`, ...): renaming one is a baseline change.
- The objects of `target.hpp` the bindings bind: `locomotion_config`, `rotary_sensor_left_config`,
  `rotary_sensor_right_config`, `imu_config`, `wall_sensors_config`, `battery_config`, `fan_config`,
  `torque_sensors_config`, `bluetooth_config`, `button_config`, `dip_switch_config`, `led_config`,
  `buzzer_config` and `argb_config`; `robot.hpp` (`sim_robot_report`) and `turn_margins.hpp`
  (`sim_turn_designs`).
- No `noexcept` on the path from the loop to a timer read: the simulator ends a run by unwinding the
  firmware thread from inside a timer read.
- `main`'s `std::signal(SIGABRT, ...)` replaces the simulator's crash reporter: a firmware abort is
  reported through the host `Mcu::emergency_stop`, counted in `meta.json`'s `emergency_stops`.

## The robot description

`robot.toml` is the robot's physical truth, written by hand from the CAD, the board and the datasheets.
Every value is either a number or `{ value, source }` naming where it came from. The simulator generates
the MJCF from it and composes it with the arena; the composed model is saved to `<out>/model.xml`.

`robot.hpp` in the firmware is the firmware's *belief*, and the two are never forced equal.
`sim_robot_report` prints them side by side. Today they differ in the emitter half angle (3 deg
datasheet against 5.2 deg with mounting tolerance), the gyro noise (datasheet against a third more), the
maze wall thickness (12 mm arena against 12.6 mm, in the classic mazes) and the outline (the 53.5 by 25 mm board against
53.5 by 25.7 mm with the sensor caps), all on purpose, and in the static friction
voltage: 0.80 V of the simulated drive, from a free motor's measured draw, against the 2.5 V the
robot's wheel breaks away at, which also holds what the bridge loses of each pulse, for the drive
identification to settle. The tires hold 0.46 here, which is how the fast runs on
the robot slip, against the 0.54 it slides sideways at on a slope.

The home maze has its measured dimensions and surfaces beside its drawing, in `mazes/home4x4.toml`: 15.1 mm
walls of semi-gloss white melamine, which send a sensor back much more light square on than at an angle.
Their Minnaert exponent is the one `config/mazes/home/maze_config.hpp` gives the firmware, and the wall
sensors' gains in `robot.toml` are calibrated against those walls, where the robot calibrated them.
`mazes/race4x4.txt` is the same boards rearranged, with the same start and goal, for a route of four
sidesteps that the racing line drives in four fifths of the time of the turns.

Things in `robot.toml` that look arbitrary and are not:

- **The tires have no rolling friction** (condim 4, not 6). MuJoCo's convex contact separates two
  surfaces in proportion to how fast their friction is slipping, and a rolling wheel keeps a
  rolling-friction constraint slipping all the time. With it the tires lost the floor on 18 % of steps
  at 0.4 m/s and 55 % at 1.5 m/s, at every timestep tried (125, 62.5 and 31.25 us), and so did a bare
  sphere. The same separation happens where tires really slide, in a pivot; a tire soft enough
  (`contact_time_constant` 5 ms or more) absorbs it inside its own deflection. 11 ms is the estimate for
  the Kyosho MZW40-20 compound (Shore 20): the tire sinks 64 um under the 0.43 N it carries without the
  fan, against 75 to 105 um estimated from the compound. With 2 ms the hopping wheels hardly scrub,
  and a pivot at 0.6 V spins at 3.2 rad/s instead of 1.1.
- **The timestep is 100 us**, one firmware loop. Down to 31.25 us the contacts and the trace stay the same.
- **The chassis mass is 76.1 g**, not 87.3: the firmware's 87.3 g and 3.78e-5 kg m^2 are the whole
  robot with its fan, and the wheels are modeled separately.
- **Each wall sensor has a gain.** `sim_wall_calibration` places the robot where the firmware calibrates
  (centered in a corridor for the side sensors, facing a wall for the front ones) and sets each gain so
  that the simulated reading equals the robot's `reference_readings` in `target.hpp`. `--sweep` shows how
  the firmware's distances then follow the true ones.
- **Two PTFE skates, 0.5 mm off the floor with the tires unloaded.** The center of mass is over the
  axle, so the robot rocks onto its rear skate when it accelerates and onto the one under its nose when
  it brakes or the fan pulls. A wider swing slams the rear contact down at every change of
  acceleration, the tires lose the floor and slide, and the odometry runs long. The skates keep the
  swing to about 1 degree back and 0.6 forward. Two spheres stand in for them. Their
  contacts are frictionless, because a sliding frictional contact separates in MuJoCo like the tires
  above, and the simulator applies their friction, 0.15 for PTFE on a painted floor, as a force against
  their sliding (`friction` of each skid).
- **The fan pulls straight down**, not along the board's normal: its actuator's reference is a site
  fixed in the world. With the fan on, the board tilts onto its nose and a pull along its normal has a
  backward part, about 1 % of it. On frictionless skids it would roll the robot 4 mm into the back
  wall while it waits 5 s for the fan.
- **The fan's 1.47 N is measured**: 150 g on a scale at half of the battery, with the skirt
  (`nominal_voltage` 6.15 V). Under the sealed skirt its suction is centered 7.8 mm ahead of the axle,
  so the skate under the nose carries a share and each tire about 1.05 N instead of 0.43. The tires
  then roll about 70 um short of their radius instead of 29, which `robot.hpp` models as 77 um per
  newton, so calibrate the wheel radius with the fan running.

## State of the port

The firmware runs on the host HAL end to end, its SPI chip drivers included. Every proxy that
`Micras::check_initialization` checks comes up: the IMU's start-up waits take 40 ms, and INIT reaches
IDLE on the tick after it starts. A button press starts an exploration through the firmware's own
paths. The checked runs end with no unbound port, no watchdog expiry and no emergency stop. `--flash`
carries a saved map into the next run.

**The whole contest runs clean on ten mazes**, which `sim_contest` shows: a short press explores, the
firmware comes back to the start on its own and saves the map, and a long press then plans and runs the
fastest route with the fan. `sim_contest_all` does the same with every switch on (fan, racing line,
boost, risky), and there too every maze is clean. Diagonals are always allowed; the second switch selects
the racing line.

The search, at 0.3 m/s, takes 104 to 227 s before the fast run starts (explore and return together,
1746 s over the ten mazes). The fast run, at up to 1 m/s, takes with the fan 9.5 s on maze 1 and 9.6 to
22.7 s on the others, 151.1 s in all; with every switch on, 8.9 s on maze 1 and 9.1 to 21.6 s on the
others, 141.2 s in all.

What the firmware does that the fast modes depend on:

- **The edge tracker times a wall edge by the reading and only names it from the map.** An edge is the
  only reference along a corridor.
- **The odometry rolls on a radius the load flattens.** With the fan the tires carry four times the
  load.
- **The search asks for half of the traction, without the fan.** Accelerating tips the robot onto its
  rear skate, which takes load off the tires.
- **A fast run asks for 0.6 of the traction, and boost for 0.65; with the fan 0.5 and 0.55.** More
  slides the tires sideways in the turns. Up to 1 m/s on the straights.
- **An edge moves the pose by 3 mm at most, and the forward loop closes at 40 Hz.** At 3 m/s an edge
  timed a millisecond off is 3 mm off, and a larger correction with a stiffer loop takes the whole supply
  at once, a jolt the crash detection reads as a wall.
- **The wall observer keeps voting through the search turns**, so the search never stops in a cell to
  look.
- **A range is corrected for the angle it meets the wall at.** Along a diagonal the diagonal sensors
  meet the walls square on.
- **Turns of two bends inside one cell**, on a planner of labels that only drops one when another is as
  fast for every route. The planner advances by edges, and `sim_turn_designs` designs the turns, which
  the firmware's build checks.
- **Turns are braked and accelerated through as their curvature allows.**
- **The tires slide to the outside of a curve**, 4.8 mm/s per m/s^2 of lateral acceleration. The
  localizer predicts it and the controller points into it.
- **The racing line**, through the cells of the planned route, at 0.9 of the lateral grip and 12 mm from
  the walls. With risky on, the line goes through the route planned without the risky turns, since it
  keeps their margin, and replaces the risky route only when it plans faster.

## Findings worth checking on the robot

1. **Front sensor parallax.** Each front emitter sits 6.5 mm above its receiver and the receiver's lobe
   is 10 degrees, so the reading falls slower than 1/d^2. The firmware models it (`receiver_offset`,
   `receiver_half_angle`). A bench sweep toward a wall confirms both.
2. **The rolling radius.** 77 um less per newton on a tire in the simulation. Driving a known distance
   with the fan on and off measures the real one.
3. **The fan tips the robot onto its nose.** Under its sealed skirt the MicrasHardware fan study
   centers the suction 7.8 mm ahead of the axle, so the skate under the nose carries the share of the
   downforce that leaves it, and drags at its friction, 0.15 estimated for PTFE. Scales under the wheels
   and under the nose, with the fan running, measure the share, and tilting the robot on its skates the
   friction.
4. **A wall start is seen early.** Toward the start of a wall a diagonal sensor also lights the wall's
   end face, by the wall thickness times the slope of the beam.
5. **Past 120 mm the readings come out long**, because part of the beam lands on the floor. The
   localizer stops at 120 mm and the wall observer at 130 mm.
6. **The mass, inertia and tire friction are estimates**, and so is the fan's position.
   `sim_robot_report` lists what differs from the firmware's belief.
7. **The lateral compliance of the tires.** `robot.hpp` has the simulation's 4.8 mm/s per m/s^2. A
   circle of known radius driven at a few speeds with the fan on measures the real one, and the racing
   line depends on it.
8. **The robot rocks between its skates.** The center of mass is over the axle, so every change of
   acceleration moves the load from one skate to the other. Above about 10.9 m/s^2 even the fan no longer
   holds the nose down. A slow-motion video of a launch shows how far it swings.

## Known gaps

- **The bench programs of `tests/` are not built here.** The design supports them, since each is a
  program with its own `main` over the same HAL, but `test_imu` spins forever when the IMU is not
  initialized.
- **No "new run" from the panel.** The state machine, the maze map and the flash all carry state, and
  the robot is a static of `main`.

## Layout

```
sim/
├── CMakeLists.txt       the targets above and the recipes
├── cmake/               firmware_sha.cmake: the firmware commit meta.json records
├── cube/                the Micras v1 Cube layer, by hand
├── include/ src/        the target, the bindings, the pool variables, main
├── tools/               the analysis plugin, the wall calibration, the robot report, the turn designer
├── scripts/             the recipes' scripts
├── tests/               doctest, on the Micras board
├── robot.toml           the physical description
├── scenarios/           idle, explore, explore_link, explore_solve(_all), solve(_all)
└── baseline/            recorded summaries
```
