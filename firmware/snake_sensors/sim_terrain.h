// SIM: what the simulation reads straight from the world (GaitController.measure_terrain and
// choose_situation in snakeapp/gait_controller.py), worked out on the robot from the IMUs and the
// joint angles. Plain C++ with no Arduino calls, so firmware/test can check it on a PC.
#pragma once
#include <math.h>
#include <string.h>

#include "sim_gaits.h"

namespace simterrain {

const int MAX_JOINTS = 8;

struct V3 { float x, y, z; };
struct M3 { float m[3][3]; };

inline M3 mul(const M3& a, const M3& b) {
  M3 r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
  return r;
}

inline M3 transpose(const M3& a) {
  M3 r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[j][i];
  return r;
}

inline V3 apply(const M3& a, V3 v) {
  return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
          a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
          a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}

// How the body part behind a joint is turned relative to the part in front of it.
// Pitch + lifts the far end of the rear part (concave, U shape); - makes a hilltop shape.
inline M3 jointRotation(bool pitch, float angleDeg) {
  const float a = angleDeg * 0.0174532925f, c = cosf(a), s = sinf(a);
  if (pitch) return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
  return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

// Slope (degrees) of the line from the trailing joint to the leading joint: + = leading end higher.
// This is the simulation's "slope", which it measures from the positions of the first and last module.
//   types      one letter per joint from the head: 'P' pitch, 'Y' yaw (as the motor board reports)
//   jointDeg   joint angles in degrees, as the motor board reports them
//   up         which way is up, in the frame of the body part that carries the IMU
//              (x toward the head, y to the left, z up); any length
//   imuPart    that body part: 0 = the head (in front of joint 0), k = between joint k-1 and joint k,
//              n = the tail
//   travelDir  +1 = crawling head first, -1 = tail first (the tail is then the leading end)
// Returns false if the input cannot be used.
//   first, last  chainSlopeDeg only: the line from joint `last` to joint `first` instead of the whole
//                body (bodySlopeDeg = joints 0 .. n-1)
inline bool chainSlopeDeg(const char* types, const float* jointDeg, int n, const float up[3], int imuPart,
                          int travelDir, int first, int last, float& slopeDeg) {
  if (n < 1 || n > MAX_JOINTS || imuPart < 0 || imuPart > n) return false;
  if (first < 0 || last > n - 1 || (n > 1 && first >= last)) return false;
  const float un = sqrtf(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
  if (un < 1e-3f) return false;
  const V3 u = {up[0] / un, up[1] / un, up[2] / un};

  // how every body part is turned, seen from the IMU's part
  M3 part[MAX_JOINTS + 1];
  part[imuPart] = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  for (int k = imuPart + 1; k <= n; ++k)
    part[k] = mul(part[k - 1], jointRotation(types[k - 1] == 'P', jointDeg[k - 1]));
  for (int k = imuPart - 1; k >= 0; --k)
    part[k] = mul(part[k + 1], transpose(jointRotation(types[k] == 'P', jointDeg[k])));

  // joint positions: part k (1..n-1) runs from joint k-1 back to joint k. The modules are equally
  // long, so the length does not matter for the angle.
  V3 line = {0, 0, 0};   // from joint `last` to joint `first`
  for (int k = first + 1; k <= last; ++k) {
    const V3 back = apply(part[k], {-1, 0, 0});
    line = {line.x - back.x, line.y - back.y, line.z - back.z};
  }
  if (n == 1) line = apply(part[0], {1, 0, 0});   // one joint: use the head's direction
  if (travelDir < 0) line = {-line.x, -line.y, -line.z};

  const float rise = line.x * u.x + line.y * u.y + line.z * u.z;
  const V3 flat = {line.x - rise * u.x, line.y - rise * u.y, line.z - rise * u.z};
  slopeDeg = atan2f(rise, sqrtf(flat.x * flat.x + flat.y * flat.y + flat.z * flat.z)) * 57.2957795f;
  return true;
}

inline bool bodySlopeDeg(const char* types, const float* jointDeg, int n, const float up[3], int imuPart,
                         int travelDir, float& slopeDeg) {
  return chainSlopeDeg(types, jointDeg, n, up, imuPart, travelDir, 0, n - 1, slopeDeg);
}

// The crest/valley "bend" from the two halves of the body (the default, SIM_BEND_FROM_HALVES):
// the slope of the front half (head IMU through joints 0 .. middle) minus the slope of the rear half
// (tail IMU through joints middle .. last). Front higher than the rear line = the ground bends up under
// the snake = a valley (> 0); lower = a hilltop (< 0). The bend of the ground does not depend on the
// way the snake crawls, so both halves are measured toward the head whatever travelDir is.
// Returns false if either IMU's answer cannot be used.
inline bool halvesBendDeg(const char* types, const float* jointDeg, int n, const float upHead[3],
                          const float upTail[3], float& bendDeg) {
  if (n < 2) return false;
  const int mid = n / 2;
  float front, rear;
  if (!chainSlopeDeg(types, jointDeg, n, upHead, 0, 1, 0, mid, front)) return false;
  if (!chainSlopeDeg(types, jointDeg, n, upTail, n, 1, mid, n - 1, rear)) return false;
  bendDeg = front - rear;
  return true;
}

// The wave bends the body up and down as it runs, which adds to the halves' bend. Averaged over
// exactly one wave cycle that bending cancels and the ground's bend is left. This keeps the last
// cycle of 0.25 s averages (simulation: robot_sensing.CycleAverage).
struct CycleAverage {
  static const int MAX = 64;   // 16 s of 0.25 s samples: a wave cycle at 0.0625 Hz
  float v[MAX];
  int n = 0, head = 0;

  void reset() { n = head = 0; }

  // add one sample and keep the last `keep` of them
  void add(float x, int keep) {
    if (keep < 1) keep = 1;
    if (keep > MAX) keep = MAX;
    v[head] = x;
    head = (head + 1) % MAX;
    if (n < MAX) ++n;
    if (n > keep) n = keep;
  }

  bool valid() const { return n > 0; }

  float mean() const {
    float sum = 0.0f;
    for (int i = 1; i <= n; ++i) sum += v[(head - i + MAX) % MAX];
    return n ? sum / n : 0.0f;
  }
};

// How many samples, dt seconds apart, make one wave cycle at waveHz
inline int cycleSamples(float waveHz, float dt) {
  if (waveHz <= 0.0f || dt <= 0.0f) return 1;
  const int k = (int)(1.0f / waveHz / dt + 0.5f);
  return k < 1 ? 1 : k;
}

// Filtered slope and ground bend, and the gait choice.
struct Terrain {
  float slope = 0.0f;   // degrees, + = uphill
  float bend = 0.0f;    // degrees: < 0 the ground falls away (hilltop), > 0 it rises (valley)
  bool valid = false;

  void reset() {
    valid = false;
    slope = bend = slow_ = 0.0f;
  }

  // Call every 0.25 s with the slope measured since the last call.
  // This also gives the older bend (SIM_BEND_FROM_HALVES = false): how much the slope has changed,
  // the slope minus its own slow average (time constant bendTau seconds). Tested in the simulation it
  // missed most valleys: the snake climbs onto a ramp so slowly that the average keeps up, and while
  // it is stuck the bend fades to zero. With the halves rule the caller overwrites `bend`.
  void update(float measuredSlopeDeg, float dt, float bendTau) {
    if (!valid) {
      slope = slow_ = measuredSlopeDeg;
      bend = 0.0f;
      valid = true;
      return;
    }
    slope = 0.8f * slope + 0.2f * measuredSlopeDeg;   // as measure_terrain
    float k = dt / bendTau;
    if (k > 1.0f) k = 1.0f;
    slow_ += (slope - slow_) * k;
    bend = slope - slow_;
  }

  // GaitController.choose_situation
  int choose(int now, bool crestAndValley) const {
    const float s = slope, b = bend;
    if (crestAndValley) {
      if (b < -SIM_BEND_ON_DEG || (now == SIT_CREST && b < -SIM_BEND_OFF_DEG)) return SIT_CREST;
      if (b > SIM_BEND_ON_DEG || (now == SIT_VALLEY && b > SIM_BEND_OFF_DEG)) return SIT_VALLEY;
    }
    if (s > SIM_SLOPE_ON_DEG) return SIT_UPHILL;
    if (s < -SIM_SLOPE_ON_DEG) return SIT_DOWNHILL;
    if (fabsf(s) < SIM_SLOPE_OFF_DEG) return SIT_FLAT;
    if (now == SIT_FLAT || now == SIT_UPHILL || now == SIT_DOWNHILL) return now;
    return s > 0 ? SIT_UPHILL : SIT_DOWNHILL;
  }

 private:
  float slow_ = 0.0f;
};

// gaits_for(): the table for a joint layout such as "PYPYP", else the closest one
inline const SimGaitTable* tableFor(const char* types) {
  if (types == nullptr || types[0] == 0) return nullptr;
  for (int i = 0; i < SIM_GAIT_TABLE_COUNT; ++i)
    if (strcmp(SIM_GAIT_TABLES[i].config, types) == 0) return &SIM_GAIT_TABLES[i];
  int pitch = 0, yaw = 0;
  for (const char* c = types; *c; ++c) {
    if (*c == 'P') ++pitch;
    if (*c == 'Y') ++yaw;
  }
  const char* closest = yaw > pitch ? "YYPYY" : "PPYPP";
  for (int i = 0; i < SIM_GAIT_TABLE_COUNT; ++i)
    if (strcmp(SIM_GAIT_TABLES[i].config, closest) == 0) return &SIM_GAIT_TABLES[i];
  return nullptr;
}

}  // namespace simterrain
