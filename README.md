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
Other options: `--design v3` (the earlier robot), `--head-mass` / `--tail-mass` (kg), `--friction` (track), `--cradles`.

In the viewer, keys 1–4 switch scene: 1 hills, 2 maze, 3 hills and maze, 4 hills and maze with a normal
PYPYP next to one that slithers.

The hill track (`TRACK` in `environment/course_scene.py`) starts with a flat 0.8 m panel, then an 11° hill
(0.12 m high) and a 14° hill (0.15 m) with a sharp top.

### How the wave runs

The wave runs on the clock at `WAVE_HZ` cycles per second (PPYPP 0.8, PYPYP 0.55, YYPYY 0.6), like
the robot's motor sketch, and every servo follows its moving target all the time. Earlier the wave only
moved on when every joint had reached its target. That made the snakes jitter: each servo jumped to its
next target and stopped about 4 times a second, and joints overshot and were pulled back. The jitter that
is left comes from the servos at their torque limit (the middle pitch joints at 0.75 kg), where the load
pushes the joint back.

Results at 0.75 kg, every weight split (head-heavy, even, tail-heavy), no roll-overs:

| | Hills and maze | Hills | Maze |
|---|---|---|---|
| PPYPP | 299–305 s | 136 s (even) | 167–175 s |
| PYPYP | 390–404 s | 195 s (even) | 195–202 s |
| PYPYP+slither | 394–413 s | 195 s (even) | |
| YYPYY, `--wheels` | 481 s (only with `WHEEL_ROLLING_FRICTION` 0.02) | | 273 s |

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

The robot weighs about 0.75 kg. The extra weight above the printed plastic (ESP32, camera and IR sensors in
the head; ESP32 and Servo2 in the tail) is split evenly between head and tail by default
(`HEAD_MASS = 0.177`, `TAIL_MASS = 0.153` in `snake_module.py`). Weigh the real head and tail and put the
values there; `--head-mass 0 --tail-mass 0` gives the bare plastic snake (0.61 kg).

## Robot firmware

The robot's own sketches are in `esp32_code/` (motor ESP32, sensor ESP32 and the camera). `firmware/`
holds the motor and sensor sketches with the simulation's gaits added, see `firmware/README.md`.
After tuning gaits here, run `python firmware/sync_from_sim.py` to copy them into the firmware.
