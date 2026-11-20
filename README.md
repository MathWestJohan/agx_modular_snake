# agx modular snake

Repository for modular snake project in Mechatronics and System Integration course MMA4004

run this command

``
$projectRoot = (Resolve-Path ..).Path
$env:PYTHONPATH = "$projectRoot;$env:PYTHONPATH"
& "$env:PYTHONHOME\python.exe" .\slope_scene.py
``

Fast run without the viewer, for tuning the gaits (from the project root; prints a summary per snake):

``
& "$env:PYTHONHOME\python.exe" -B .\environment\headless_course.py --configs PPYPP PYPYP --scene hills --time 600
``

Scenes: `hills`, `maze`, `course`, `flat`. Add `--wheels` to give YYPYY passive wheels (serpentine needs them).
Other options: `--design v3` (the earlier robot), `--head-mass` / `--tail-mass` (kg), `--friction` (track), `--cradles`,
`--sensing robot` (choose the gait from the robot's IMUs, see below), `--servo ideal` (the old servo model).

In the viewer, keys 1–5 switch scene: 1 hills, 2 maze, 3 hills and maze, 4 hills and maze with a normal
PYPYP next to one that slithers, 5 hills and maze on the flat-start track layout.

There are two hill track layouts (`TRACKS` in `environment/course_scene.py`):

- `plateau` (the default, the real course): an 11° hill 0.12 m high, then a 14° ramp up to a flat top
  0.8 m long and 0.15 m high, and down again;
- `flat-start`: a flat 0.8 m panel first, then the same two hills with a sharp second top.

Pick one with `TRACK_LAYOUT`, or `--track flat-start` in the headless runner. The log's `run.json`
records which one a run used.

The maze has four straight legs of 0.6 m, turning 30° left, then right, and so on, between walls 0.2 m
high and 0.30 m apart (`MAZE_WIDTH`, or `--maze-width`). With `--sensing robot` the snakes switch to the
maze gait when the camera would see the tape line, which starts at the maze entrance; the camera is
assumed to look 0.15 m ahead of the head (`CAMERA_VIEW_AHEAD`).

### How the wave runs

The wave runs on the clock at `WAVE_HZ` cycles per second (PPYPP 0.8, PYPYP 0.45, YYPYY 0.6), like
the robot's motor sketch, and every servo follows its moving target all the time. Earlier the wave only
moved on when every joint had reached its target. That made the snakes jitter: each servo jumped to its
next target and stopped about 4 times a second, and joints overshot and were pulled back. The jitter that
is left comes from the servos at their torque limit (the middle pitch joints at 0.75 kg), where the load
pushes the joint back.

### The servo model

The servos are simulated as DC motors at the 5 V the Servo2 gives them (`SERVOS` in
`snakeapp/snake_control.py`): the speed falls in a straight line with the load, from 4.87 rad/s
(0.215 s per 60°) with no load to zero at the 0.224 N·m stall torque. So a faster wave leaves less
torque. The old model (full speed up to a fixed 0.268 N·m, rigid hold) is still there as `'ideal'`.
The 5 V figures are worked out from the datasheet's 4.8 V and 6 V ones; measure a real servo and
put the numbers in `SERVOS`:

- no-load speed: command a 90° jump, film it in slow motion, count frames;
- speed under load and stall torque: a 10 cm arm on the horn, hang 50 / 100 / 150 g, film the lift;
  the heaviest weight it can still lift gives the stall torque;
- `full_power_error`: how far the arm sags from the commanded angle with a weight hanging.

Results on the real course at 0.75 kg (head-heavy, even, tail-heavy) and as bare plastic, no roll-overs:

| | Hills and maze | Maze |
|---|---|---|
| PPYPP | 330–378 s | 226–240 s |
| PYPYP | 453–583 s | 267–297 s |
| PYPYP+slither | 446–502 s | |
| YYPYY, `--wheels` | | 238 s |

### The robot's own gait choice

`--sensing robot` (or `SENSING` in `course_scene.py`) makes the snakes choose their gait the way the
robot's sensor ESP32 does: from simulated accelerometers in the head and tail and the commanded joint
angles, with the firmware's slope filter and its crest/valley rule, and the maze gait when the camera
sees the tape line (`snakeapp/robot_sensing.py`, a copy of the
firmware that `firmware/test/compare_with_sim.py` checks). The logs then also hold what the
world-based rule would have chosen (`wanted_sim`), and `analyze_run.py` compares the two.

The firmware's crest/valley rule compares the two halves of the body: the slope of the front half
(head IMU) minus that of the rear half (tail IMU), averaged over one wave cycle (`--bend-rule halves`,
the default). On the real course PPYPP and PYPYP finished in all 4 weight settings (PPYPP 381–434 s,
PYPYP 470–541 s). The older rule, the slope minus its 20 s average (`--bend-rule average`), missed most
valleys: PPYPP still finished in all 4 weight settings, but PYPYP in only 2. The robot's slope estimate is good
(about 3° rms) with either rule.

### Slither

Add `+slither` to a pitch snake (`PYPYP+slither`, `PPYPP+slither`) to make it weave from side to side
while it crawls up and down: its yaw joints run a small wave on top of the pitch wave
(`gait_controller.set_slither`, default 4°, `--slither <deg>` in the headless runner).

It slithers on flat ground and going downhill, and crawls plain while climbing, through the foot of a
ramp, over a hilltop and in the maze (`SLITHER_GAITS`). Slithering through the transitions rolled it
over, and slithering while climbing cost so much grip that it slid back down the steep 14° ramp. Like
this it is as fast as the plain snake (see the table above).
Bigger amplitudes (6°, 10°) roll it over. See the comments in `gait_controller.py`.

``
& "$env:PYTHONHOME\python.exe" -B .\environment\headless_course.py --configs PYPYP PYPYP+slither --scene course --time 800
``

### Data logs and quick analysis

`--log <folder>` writes one CSV per snake, 10 rows a second (position, gait, slope, steering, tilt, every
joint's angle and target, servo torque and time at the torque limit), plus `run.json` with the settings and
results. `analyze_run.py` reads them: distance and speed, time per gait, every stall, servo load, roll-over,
and with `--plot` a picture next to each log. Give it several folders to get a table comparing the runs.

``
& "$env:PYTHONHOME\python.exe" -B .\environment\headless_course.py --configs PYPYP --scene course --time 750 --log runs\test1
& "$env:PYTHONHOME\python.exe" -B .\environment\analyze_run.py runs\test1 --plot
& "$env:PYTHONHOME\python.exe" -B .\environment\analyze_run.py runs\test1 runs\test2 --brief
& "$env:PYTHONHOME\python.exe" -B .\environment\analyze_run.py runs\test1 --from 300 --to 400
``

For the viewer, set `LOG_DIR` in `environment/course_scene.py`. The analyser needs no AGX.

### Charts for comparing runs

`plot_runs.py` draws charts that compare runs, for reports (PNG, SVG or PDF; needs matplotlib, not AGX).
Each run keeps its colour in every chart, up to 8 runs per chart:

- `summary`: finish time of every run, or where it got stuck or rolled over
- `progress`: how far along the course each run was over time, with the hills, maze and finish marked
- `stalls`: the course profile, and where each run stopped for 15 s or more
- `gaits`: which gait each run used over time (for `--sensing robot` runs also the world-based choice)
- `servo_error`, `servo_torque`: how far each joint lagged behind its target, and its mean torque
- `sensing`, `choice`: for `--sensing robot` runs, the robot's slope and bend estimates against the true
  values, and how often its gait choice matched the world-based one

``
& "$env:PYTHONHOME\python.exe" -B .\environment\plot_runs.py runs\even runs\tail-heavy
& "$env:PYTHONHOME\python.exe" -B .\environment\plot_runs.py runs\a runs\b --snakes PYPYP --labels "firmware rule" "proposed rule" --format png pdf --out report\figures
``

## Robot design

The simulation builds the V5 design (`environment/assets/ssv5`) by default; set `DESIGN = 'v3'` in
`snakeapp/snake_module.py` for the earlier bare modules. Assembly, from the front:

- head (`SS_V5_head`) bolted to the first module's head arm (`SS_V5_arm_head`), so the snake leads with an arm
- five modules: cradle (`SS_V5_krybbe_kort`, or `_lang` per module with `cradles`), Parallax servo, arm (`SS_V5_arm_vanlig`)
- a plate (`SS_V5_mellomplate`) between each pair of modules, with a tooth under each bottom corner
- tail (`SS_V5_tail`) bolted to the last module's cradle
- teeth (`SS_V5_tenner`) under the four bottom corners of the head and of the tail; the notched
  `_head_L` / `_head_R` ones at the head's front corners

The snake stands on the tips of the teeth, 5 mm below the modules. After changing the CAD files, run
`python snakeapp/convert_snake_to_mesh.py v5` to remake the meshes the simulation loads.

The teeth are printed in a rubber-like filament (TPU) for grip. In the simulation they have their own
material (`RUBBER_TEETH` in `snake_module.py`) with friction 0.75 on every surface (`TEETH_FRICTION` in
`course_scene.py`; an estimate from a TPU 95A-on-hardwood figure of 0.70–0.80), against 0.5–0.6 for the
PLA parts. The plates between the modules get the same material, since only their teeth touch the
ground. Measure the real value: put the snake on a board of the track material and tilt it until it
slides; the friction is tan(angle). `--teeth-friction <mu>` sets it for a run, `0` makes the teeth PLA.

The robot weighs about 0.75 kg. The extra weight above the printed plastic (ESP32, camera and IR sensors in
the head; ESP32 and Servo2 in the tail) is split evenly between head and tail by default
(`HEAD_MASS = 0.177`, `TAIL_MASS = 0.153` in `snake_module.py`). Weigh the real head and tail and put the
values there; `--head-mass 0 --tail-mass 0` gives the bare plastic snake (0.61 kg).

## Robot firmware

The robot's own sketches are in `esp32_code/` (motor ESP32, sensor ESP32 and the camera). `firmware/`
holds the motor and sensor sketches with the simulation's gaits added, see `firmware/README.md`.
After tuning gaits here, run `python firmware/sync_from_sim.py` to copy them into the firmware.
