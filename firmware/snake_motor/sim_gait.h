// SIM: the joint angles of a gait, made the way the simulation makes them
// (SnakeControl.control_group_init in snakeapp/snake_control.py and the gait tables in
// snakeapp/gait_controller.py). Plain C++ with no Arduino calls, so firmware/test can check it
// against the simulation on a PC.
#pragma once
#include <math.h>

// same numbers as "enum Mode" in the sketches
const int SIM_MODE_VERTICAL = 1;   // caterpillar: wave on the pitch joints
const int SIM_MODE_LATERAL  = 2;   // serpentine: wave on the yaw joints
const int SIM_MODE_SIDEWIND = 3;

// Angle (degrees) of one joint.
//   joint            0 = the joint next to the head
//   ampDeg, lagDeg   amplitude, and phase lag per TWO joints along the body (the simulation's rule).
//                    For pitch-yaw-pitch-yaw-pitch that is the lag from one pitch joint to the next,
//                    the same as "index among joints of the same type" in the first sketches.
//   phase            rad, grows with time when going forward
//   pitchOffsetDeg   caterpillar only: constant bend added to every pitch joint.
//                    Negative = convex (both ends down, like the top of a hill). The simulation's
//                    crest gait uses this to reach down the far side of a hilltop.
//   caterpillarWave  +1: the pitch wave runs from the tail to the head, which moves the snake head
//                    first in the simulation. -1: from the head to the tail.
// The serpentine and sidewind waves run from the head to the tail, as in the simulation.
inline float simGaitAngle(int mode, bool isPitch, int joint, float ampDeg, float lagDeg, float phase,
                          float pitchOffsetDeg, int caterpillarWave) {
  const float halfPi = 1.57079633f;
  // position along the body in steps of two joints: i/2 for a pitch joint, (i-1)/2 for a yaw joint
  const float k = isPitch ? joint / 2.0f : (joint - 1) / 2.0f;
  const float lag = lagDeg * (halfPi / 90.0f) * k;
  switch (mode) {
    case SIM_MODE_VERTICAL:
      return isPitch ? ampDeg * sinf(phase + caterpillarWave * lag) + pitchOffsetDeg : 0.0f;
    case SIM_MODE_LATERAL:
      return isPitch ? 0.0f : ampDeg * sinf(phase - lag);
    case SIM_MODE_SIDEWIND:
      return isPitch ? 0.5f * ampDeg * sinf(phase - lag - halfPi) : ampDeg * sinf(phase - lag);
    default:
      return 0.0f;
  }
}
