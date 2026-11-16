# Snake robot firmware with the simulation's gaits

These are the robot's own sketches from `esp32_code/`, plus what the simulation needs. `esp32_code/` is left as it was. Every change in the two sketches here is marked `SIM` in the code.

| Board | Sketch | Compared with `esp32_code/` |
|---|---|---|
| Motor ESP32 | `snake_motor/` | Wave formula from the simulation, pitch offset, wave direction setting, pitch test command |
| Sensor ESP32 | `snake_sensors/` | "Sim gait": gait table from the simulation, gait chosen from the slope the IMUs measure |
| Camera (Timer Camera X) | use `esp32_code/snake_camera_line.ino` | No change. The simulation has no tape line, so nothing in it concerns the camera. |

Wiring, Wi-Fi dashboard, link format and speeds, the Servo2 driver, the BOOT button and auto-start are all unchanged. With "Sim gait" off, the sensor board behaves as before.

## What the simulation needed that the robot did not have

The robot is pitch-yaw-pitch-yaw-pitch, which is the simulation's `PYPYP` snake. In the simulation it crawls in caterpillar mode and steers with the yaw joints, as the robot does. These things were missing or different:

| | `esp32_code/` | Simulation | Here |
|---|---|---|---|
| Caterpillar wave direction | head → tail | tail → head moves the snake head first | tail → head by default (`CATERPILLAR_WAVE` in `snake_motor.ino`) |
| Amplitude | 20° slider default | 35° flat and uphill, 30° downhill, 20° in the maze | from the simulation's table when "Sim gait" is on |
| Hilltops | nothing | "crest" gait: 30° wave plus every pitch joint bent 10° down | pitch offset in the order to the motor board |
| Choosing the gait | by hand | from the slope of the body | from both IMUs and the joint angles |

Phase lag (120°) already matched. Speed stays on the slider: the simulated servos, carrying the snake's weight, managed about 0.35 wave cycles per second, so 0.3–0.5 Hz is the range to try.

The wave direction is the one change that alters what the robot does with "Sim gait" off. **Check it first** (below).

## Files

```
firmware/
  snake_motor/
    snake_motor.ino     esp32_code/snake_motor.ino + SIM changes
    sim_gait.h          the wave formula (plain C++, checked against the simulation)
  snake_sensors/
    snake_sensors.ino   esp32_code/snake_sensors.ino + SIM changes
    sim_gaits.h         gait tables, generated from the simulation: do not edit
    sim_terrain.h       body slope from the IMUs + joint angles, gait choice (plain C++)
  sync_from_sim.py      simulation -> snake_sensors/sim_gaits.h
  test/                 PC tests (no ESP32 needed)
```

Open each sketch folder in the Arduino IDE as usual (board "ESP32 Dev Module", library "WebSockets" for the sensor sketch). Both compile with ESP32 core 3.3.12.

The order to the motor board has one more number at the end, the pitch offset:

```
$S,mode,amp,freq,lag,dir,steer,pitchOffset*CS
```

Old and new sketches still understand each other: the old motor sketch ignores the extra number, and the new one treats a missing number as 0. Use the two from this folder together to get the simulation's gaits.

## Checks on the robot, in this order

### 1. Pitch joints bend the right way (motor board, USB)

The simulation's gaits and the slope measurement both need this sign: a **negative** pitch angle bends the snake like a hilltop, with both ends down.

1. Stop the snake and lay it flat and straight.
2. Type `p` in the motor board's Serial Monitor. Every pitch joint goes to −15° for 3 s.
3. Both ends of the snake must go **down**. A pitch joint that lifts its end instead gets `-1` in `DIRECTION`.

### 2. Caterpillar crawls head first

1. "Sim gait" off, direction "forward", press Caterpillar.
2. The snake must crawl **head first**.
3. If it crawls tail first, set `CATERPILLAR_WAVE = -1` in `snake_motor.ino`. Please tell us if that happens: the real robot then moves the opposite way from the simulated one, and the simulation needs correcting.

### 3. IMUs calibrated

"Sim gait" only measures the slope with calibrated IMUs. Calibrate as before: snake flat, straight and still, STOP, "Calibrate IMUs". Then lift the head end by hand: the dashboard's slope must go **positive**. The front IMU must be in the head and the rear IMU in the tail.

### 4. Sim gait

1. Tick "Sim gait" (or USB `g` on the sensor board), then press Caterpillar.
2. The line under the checkboxes shows the gait in use, the slope and the bend. The amplitude slider follows the table.
3. On a ramp the gait should change to `uphill`; over the top to `crest` for a while, then `flat` or `downhill`.

For a course run without a phone, set `START_SIM_GAIT = true` in `snake_sensors.ino`.

## How the gait is chosen

Four times a second, as in the simulation:

1. **Slope.** Each IMU gives the tilt of its own end, which rocks with the wave. The joint angles from the motor board turn that into the slope of the whole body: the line from the last joint to the first, which is what the simulation measures. Two IMUs give two answers, which are averaged.
2. **Bend.** The simulation feels a hilltop through the ground contacts under the snake. The robot cannot, so it uses how much the slope has changed: the slope minus its own average over about 20 s (`SIM_BEND_TAU_S`).
3. **Choice.** The simulation's thresholds: uphill or downhill beyond ±8°, flat within ±4°, crest or valley when the bend passes ±8°. A gait is kept for at least 3 s.
4. **Maze.** With "Auto-steer (IR)" on and flat ground, the simulation's maze gait is used: a 20° wave, because the bigger one rolled the snake over between the walls.

## Not tested, and known gaps

- **Nothing here has run on the robot.** It compiles, and the PC tests pass.
- **Crest and valley detection** (step 2 above) is new and has not been tried in the simulation. It reacts with a delay, and stops reacting if the snake is stuck on the hilltop for longer than about 20 s. Set `SIM_CREST_VALLEY = false` to switch it off. In the simulation the crest gait mattered for getting over the second hilltop.
- **Tilt while crawling** comes from accelerometers alone. Knocks show up as tilt errors. The slope is averaged over several seconds, which should cover it.
- **The simulation's robot is not weighed.** Its head and tail masses are guesses without the camera, batteries and boards. Heavier ends change how well it climbs.
- **Serpentine and sidewind** are not from the simulation: it has no such gaits for this joint layout. They are unchanged.
- **Line following** is not in the simulation, which steers at known positions instead. The steering itself (a bend on the yaw joints) is the same.

## Changing the gaits

Tune the gaits in the simulation (`snakeapp/gait_controller.py`), then run this from the project root and upload `snake_sensors` again:

```
python firmware/sync_from_sim.py
```

## PC tests

These check the plain C++ parts on the PC. No ESP32 is needed.

With Visual Studio (from a "x64 Native Tools Command Prompt"), or the same file with `g++ -std=c++17 -o host_test`:
```
cl /std:c++17 /EHsc /Fe:host_test.exe firmware\test\host_test.cpp
```

Then:
```
host_test checks                                          slope from the IMUs, crest detection, tables, wave direction
python firmware/test/compare_with_sim.py host_test.exe    same joint angles and gait choice as the simulation (needs AGX)
```
