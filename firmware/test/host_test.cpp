// PC tests for the simulation parts of the firmware (the plain C++ headers). No ESP32 needed.
// Build: see firmware/README.md.
//   host_test checks   body slope from the IMUs, crest detection, gait tables, wave direction
//   host_test gait     joint angles of every gait in the table   } compare_with_sim.py runs the same
//   host_test choose   gait choice over a grid of slope and bend } on the simulation and compares
//   host_test slope    body slope from random joint angles and tilts }  (snakeapp/robot_sensing.py
//   host_test terrain  filtered slope and bend over a slope sequence }   runs these in the simulation)
//   host_test halves   front/rear-halves bend, and its average over a wave cycle }
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../snake_motor/sim_gait.h"
#include "../snake_sensors/sim_gaits.h"
#include "../snake_sensors/sim_terrain.h"

namespace {

const double kPi = 3.14159265358979323846;
float rad(double deg) { return (float)(deg * kPi / 180.0); }

int failures = 0;

void check(bool ok, const char* what) {
    printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) ++failures;
}

void near(double got, double want, double tolerance, const char* what) {
    const bool ok = fabs(got - want) <= tolerance;
    printf("%s  %s  (got %.3f, want %.3f)\n", ok ? "ok  " : "FAIL", what, got, want);
    if (!ok) ++failures;
}

// up vector as the sensor sketch builds it from an IMU's pitch (+ = nose up) and roll, degrees
void upFrom(double pitch, double roll, float up[3]) {
    up[0] = (float)sin(rad(pitch));
    up[1] = (float)(cos(rad(pitch)) * sin(rad(roll)));
    up[2] = (float)(cos(rad(pitch)) * cos(rad(roll)));
}

void slopeChecks() {
    const char* types = "PYPYP";
    const float straight[5] = {0, 0, 0, 0, 0};
    float up[3], slope = 0;

    upFrom(14, 0, up);
    simterrain::bodySlopeDeg(types, straight, 5, up, 0, 1, slope);
    near(slope, 14.0, 1e-3, "slope: straight snake, head IMU tilted up 14 deg");
    simterrain::bodySlopeDeg(types, straight, 5, up, 5, 1, slope);
    near(slope, 14.0, 1e-3, "slope: straight snake, tail IMU tilted up 14 deg");
    simterrain::bodySlopeDeg(types, straight, 5, up, 0, -1, slope);
    near(slope, -14.0, 1e-3, "slope: same, crawling tail first -> the leading end is lower");

    upFrom(0, 25, up);
    simterrain::bodySlopeDeg(types, straight, 5, up, 0, 1, slope);
    near(slope, 0.0, 1e-3, "slope: straight snake rolled 25 deg stays level");

    // middle pitch joint bent like a valley by 23 deg, head level: the tail half rises
    const float valley[5] = {0, 0, 23, 0, 0};
    upFrom(0, 0, up);
    simterrain::bodySlopeDeg(types, valley, 5, up, 0, 1, slope);
    near(slope, -11.5, 1e-3, "slope: middle pitch joint +23 (valley shape), head level -> tail end higher");
    const float hilltop[5] = {0, 0, -23, 0, 0};
    simterrain::bodySlopeDeg(types, hilltop, 5, up, 0, 1, slope);
    near(slope, 11.5, 1e-3, "slope: middle pitch joint -23 (hilltop shape), head level -> tail end lower");

    // a bent snake: the head IMU and the tail IMU must give the same slope
    const float bent[5] = {-17, 10, 12, -8, -14};
    float fromHead = 0, fromTail = 0;
    upFrom(6, 3, up);
    simterrain::bodySlopeDeg(types, bent, 5, up, 0, 1, fromHead);
    // what the tail IMU reads then: the same "up", seen from the tail
    simterrain::M3 tail = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    for (int i = 0; i < 5; ++i) tail = simterrain::mul(tail, simterrain::jointRotation(types[i] == 'P', bent[i]));
    const simterrain::V3 upTail = simterrain::apply(simterrain::transpose(tail), {up[0], up[1], up[2]});
    const float upT[3] = {upTail.x, upTail.y, upTail.z};
    simterrain::bodySlopeDeg(types, bent, 5, upT, 5, 1, fromTail);
    near(fromTail, fromHead, 1e-3, "slope: bent snake, head IMU and tail IMU agree");

    const float zero[3] = {0, 0, 0};
    check(!simterrain::bodySlopeDeg(types, straight, 5, zero, 0, 1, slope), "slope: no IMU reading -> no slope");
    check(!simterrain::bodySlopeDeg(types, straight, 0, up, 0, 1, slope), "slope: no joints reported -> no slope");
}

void terrainChecks() {
    // climb a 14 deg ramp, then come over the top onto flat ground in 15 s
    simterrain::Terrain t;
    const float dt = 0.25f, tau = 20.0f;
    int now = SIT_FLAT;
    bool sawUphill = false, sawCrest = false;
    float crestSeconds = 0;
    for (float time = 0; time < 120; time += dt) {
        float slope = 14.0f;
        if (time > 60) slope = 14.0f * (1.0f - (time - 60) / 15.0f);
        if (time > 75) slope = 0.0f;
        t.update(slope, dt, tau);
        now = t.choose(now, true);
        if (time < 60 && now == SIT_UPHILL) sawUphill = true;
        if (time > 60 && now == SIT_CREST) {
            sawCrest = true;
            crestSeconds += dt;
        }
    }
    check(sawUphill, "terrain: steady 14 deg slope -> uphill gait");
    char what[96];
    snprintf(what, sizeof what, "terrain: slope falling 14 -> 0 deg in 15 s -> crest gait (for %.0f s)", crestSeconds);
    check(sawCrest, what);
    check(now == SIT_FLAT, "terrain: back to the flat gait afterwards");
}

void tableChecks() {
    const SimGaitTable* t = simterrain::tableFor("PYPYP");
    check(t != nullptr && strcmp(t->config, "PYPYP") == 0, "tables: PYPYP has its own table");
    t = simterrain::tableFor("PYP");
    check(t != nullptr && strcmp(t->config, "PPYPP") == 0, "tables: an unknown pitch layout uses PPYPP");
    t = simterrain::tableFor("YYY");
    check(t != nullptr && strcmp(t->config, "YYPYY") == 0, "tables: an unknown yaw layout uses YYPYY");
    check(simterrain::tableFor("") == nullptr, "tables: no layout yet -> no table");
}

void waveChecks() {
    // caterpillar, CATERPILLAR_WAVE = +1: the hump must reach the tail pitch joint first, then the
    // middle one, then the head one (the simulation: tail -> head moves the snake head first)
    const float lag = 120.0f;
    float peak[5] = {0, 0, 0, 0, 0};   // phase (deg, 0..360) at which each joint is at its top
    for (int j = 0; j < 5; j += 2) {
        float best = -1e9f;
        for (int p = 0; p < 360; ++p) {
            const float a = simGaitAngle(SIM_MODE_VERTICAL, true, j, 30.0f, lag, rad(p), 0.0f, 1);
            if (a > best) {
                best = a;
                peak[j] = (float)p;
            }
        }
    }
    const float tailToMiddle = fmodf(peak[2] - peak[4] + 360.0f, 360.0f);
    const float middleToHead = fmodf(peak[0] - peak[2] + 360.0f, 360.0f);
    check(fabsf(tailToMiddle - 120.0f) < 1.5f && fabsf(middleToHead - 120.0f) < 1.5f,
          "wave: caterpillar hump goes tail -> middle -> head, 120 deg apart");
    check(simGaitAngle(SIM_MODE_VERTICAL, false, 1, 30.0f, lag, 1.0f, -10.0f, 1) == 0.0f,
          "wave: caterpillar leaves the yaw joints straight (steering is added by the motor sketch)");
    near(simGaitAngle(SIM_MODE_VERTICAL, true, 0, 0.0f, lag, 1.0f, -10.0f, 1), -10.0, 1e-5,
         "wave: the pitch offset is added to the pitch joints");
    // serpentine: head -> tail
    float y1 = 0, y3 = 0, bestY1 = -1e9f, bestY3 = -1e9f;
    for (int p = 0; p < 360; ++p) {
        const float a1 = simGaitAngle(SIM_MODE_LATERAL, false, 1, 30.0f, lag, rad(p), 0.0f, 1);
        const float a3 = simGaitAngle(SIM_MODE_LATERAL, false, 3, 30.0f, lag, rad(p), 0.0f, 1);
        if (a1 > bestY1) { bestY1 = a1; y1 = (float)p; }
        if (a3 > bestY3) { bestY3 = a3; y3 = (float)p; }
    }
    check(fabsf(fmodf(y3 - y1 + 360.0f, 360.0f) - 120.0f) < 1.5f, "wave: serpentine bend goes head -> tail");
}

// What the robot does with one gait of the table: the sensor sketch sends |amplitude|, |lag| and the
// pitch offset, the motor sketch makes the wave with CATERPILLAR_WAVE = +1.
// keep in step with python_gait() in compare_with_sim.py
void gait() {
    for (int t = 0; t < SIM_GAIT_TABLE_COUNT; ++t) {
        const SimGaitTable& table = SIM_GAIT_TABLES[t];
        const int n = (int)strlen(table.config);
        for (int s = 0; s < SIT_COUNT; ++s) {
            const SimGait& g = table.gaits[s];
            if (!g.valid) continue;
            const bool pitchWave = table.gaits[SIT_FLAT].amP != 0.0f;   // simMode() in the sensor sketch
            const int mode = pitchWave ? SIM_MODE_VERTICAL : SIM_MODE_LATERAL;
            const float amp = fabsf(pitchWave ? g.amP : g.amY);
            const float lag = fabsf(pitchWave ? g.phiP : g.phiY);
            // a negative amplitude in the table is the same wave half a cycle later
            const double shift = (pitchWave ? g.amP : g.amY) < 0 ? kPi : 0.0;
            for (int k = 0; k < 12; ++k) {
                printf("%s %s %d", table.config, SIM_SITUATION_NAMES[s], k);
                for (int j = 0; j < n; ++j)
                    printf(" %.4f", simGaitAngle(mode, table.config[j] == 'P', j, amp, lag,
                                                 (float)(2 * kPi * k / 12 + shift), g.offsetP, 1));
                printf("\n");
            }
        }
    }
}

// keep in step with python_choose() in compare_with_sim.py
void choose() {
    const char* configs[] = {"PPYPP", "YYPYY"};
    for (const char* config : configs) {
        const SimGaitTable* table = simterrain::tableFor(config);
        const bool crestValley = table->gaits[SIT_CREST].valid && table->gaits[SIT_VALLEY].valid;
        simterrain::Terrain t;
        for (int now = 0; now < SIT_COUNT; ++now) {
            if (!table->gaits[now].valid) continue;
            for (int i = 0; i < 42; ++i) {
                for (int j = 0; j < 30; ++j) {
                    t.slope = -20.5f + i;
                    t.bend = -14.5f + j;
                    printf("%s %s %.1f %.1f %s\n", config, SIM_SITUATION_NAMES[now], t.slope, t.bend,
                           SIM_SITUATION_NAMES[t.choose(now, crestValley)]);
                }
            }
        }
    }
}

// Same pseudo-random numbers as _Lcg in compare_with_sim.py
unsigned long long lcgState = 12345;
double rnd() {
    lcgState = (lcgState * 1103515245ULL + 12345ULL) % 2147483648ULL;
    return (double)lcgState / 2147483648.0;
}

// keep in step with python_slope() in compare_with_sim.py
void slope() {
    const char* configs[] = {"PYPYP", "PPYPP", "YYPYY"};
    lcgState = 12345;
    for (const char* config : configs) {
        const int n = (int)strlen(config);
        for (int k = 0; k < 200; ++k) {
            float joints[simterrain::MAX_JOINTS];
            for (int j = 0; j < n; ++j) joints[j] = (float)(-60.0 + 120.0 * rnd());
            const double p = (-40.0 + 80.0 * rnd()) * kPi / 180.0, r = (-30.0 + 60.0 * rnd()) * kPi / 180.0;
            const float up[3] = {(float)sin(p), (float)(cos(p) * sin(r)), (float)(cos(p) * cos(r))};
            const int part = k % 2 ? n : 0, dir = (k / 2) % 2 ? -1 : 1;
            float out = 0;
            simterrain::bodySlopeDeg(config, joints, n, up, part, dir, out);
            printf("%s %d %.4f\n", config, k, out);
        }
    }
}

// keep in step with python_terrain() in compare_with_sim.py
void terrain() {
    simterrain::Terrain t;
    for (int k = 0; k < 400; ++k) {
        const float s = (float)(12.0 * sin(k * 0.05) + (k > 200 ? 8.0 : 0.0));
        t.update(s, 0.25f, 20.0f);
        printf("%d %.4f %.4f\n", k, t.slope, t.bend);
    }
}

// keep in step with python_halves() in compare_with_sim.py
void halves() {
    const char* configs[] = {"PYPYP", "PPYPP", "YYPYY"};
    lcgState = 54321;
    for (const char* config : configs) {
        const int n = (int)strlen(config);
        for (int k = 0; k < 200; ++k) {
            float joints[simterrain::MAX_JOINTS];
            for (int j = 0; j < n; ++j) joints[j] = (float)(-50.0 + 100.0 * rnd());
            float up[2][3];
            for (int i = 0; i < 2; ++i) {
                const double p = (-30.0 + 60.0 * rnd()) * kPi / 180.0, r = (-20.0 + 40.0 * rnd()) * kPi / 180.0;
                up[i][0] = (float)sin(p);
                up[i][1] = (float)(cos(p) * sin(r));
                up[i][2] = (float)(cos(p) * cos(r));
            }
            float out = 0;
            simterrain::halvesBendDeg(config, joints, n, up[0], up[1], out);
            printf("%s %d %.4f\n", config, k, out);
        }
    }
    // the cycle average: a bend that steps and a wave rate that changes
    simterrain::CycleAverage c;
    for (int k = 0; k < 300; ++k) {
        const float hz = k < 100 ? 0.45f : (k < 200 ? 0.8f : 0.1f);
        const float x = (float)(20.0 * sin(k * 0.7) + (k > 150 ? 12.0 : 0.0));
        c.add(x, simterrain::cycleSamples(hz, 0.25f));
        printf("c %d %.4f\n", k, c.mean());
    }
}

void halvesChecks() {
    // a straight snake lying on flat ground: no bend
    const float straight[5] = {0, 0, 0, 0, 0};
    const float level[3] = {0, 0, 1};
    float bend = 99;
    check(simterrain::halvesBendDeg("PYPYP", straight, 5, level, level, bend) && fabs(bend) < 1e-3,
          "halves: straight and level -> no bend");
    // the same snake bent up at the middle joint (U shape, both ends up 10 deg): a valley, bend 20 deg.
    // Head part tilted 10 deg nose up, tail part 10 deg tail up (that is nose down, seen from the tail).
    const float vee[5] = {0, 0, 20, 0, 0};
    const float headUp[3] = {sinf(rad(10)), 0, cosf(rad(10))};
    const float tailUp[3] = {sinf(rad(-10)), 0, cosf(rad(-10))};
    check(simterrain::halvesBendDeg("PYPYP", vee, 5, headUp, tailUp, bend) && bend > 15 && bend < 25,
          "halves: body bent up at the middle -> valley (bend > 0)");
    near(bend, 20.0, 1.0, "halves: valley of 2 x 10 deg");
    simterrain::CycleAverage c;
    for (int k = 0; k < 9; ++k) c.add(k % 2 ? 10.0f : -10.0f, 2);
    near(c.mean(), 0.0, 1e-4, "cycle average: keeps only the last cycle");
    check(simterrain::cycleSamples(0.45f, 0.25f) == 9, "cycle average: 0.45 Hz = 9 samples of 0.25 s");
}

}  // namespace

int main(int argc, char** argv) {
    const char* mode = argc > 1 ? argv[1] : "checks";
    if (strcmp(mode, "gait") == 0) {
        gait();
        return 0;
    }
    if (strcmp(mode, "choose") == 0) {
        choose();
        return 0;
    }
    if (strcmp(mode, "slope") == 0) {
        slope();
        return 0;
    }
    if (strcmp(mode, "terrain") == 0) {
        terrain();
        return 0;
    }
    if (strcmp(mode, "halves") == 0) {
        halves();
        return 0;
    }
    slopeChecks();
    terrainChecks();
    halvesChecks();
    tableChecks();
    waveChecks();
    printf("%s\n", failures == 0 ? "all checks passed" : "SOME CHECKS FAILED");
    return failures == 0 ? 0 : 1;
}
