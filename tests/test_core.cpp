// g++ -std=c++17 -O2 -I../src/core test_core.cpp -o test_core && ./test_core
// The "reality" simulator uses slightly different physics than the pilot's model,
// to exercise drift detection + replanning.
#include <cstdio>
#include <functional>
#include <string>

#include "Pilot.hpp"
using namespace pf;

static Obj solid(float x, float y, float w, float h) { return {Kind::Solid, x, y, w / 2, h / 2, 0}; }
static Obj spike(float x, float y) { return {Kind::Hazard, x, y, 4.f, 8.f, 0}; }

struct Result { bool ok; int attempts; float maxX; float maxDrift; double sec; };

static Result run(const std::string& name, std::vector<Obj> objs, State start, float goal,
                  bool platformer, float realGravityScale, int maxAttempts = 8) {
    Pilot pilot;
    pilot.cfg.platformer = platformer;
    if (platformer) pilot.cfg.hWeight = 1.4f;
    pilot.goalX = goal;
    pilot.world.build(objs);

    Config real = pilot.cfg;  // reality differs slightly
    real.g *= realGravityScale; real.cubeJump *= 1.01f; real.shipUp *= realGravityScale;
    real.shipDown *= realGravityScale;

    Result r{false, 0, 0, 0, 0};
    for (int att = 1; att <= maxAttempts && !r.ok; att++) {
        r.attempts = att;
        State s = start;
        pilot.reset();
        pilot.preplan(s, 400);
        Act a;
        for (int dec = 0; dec < 20000; dec++) {
            pilot.noteActual(s);
            a = pilot.decide(s, 500);
            for (int k = 0; k < DECISION_TICKS && !s.dead; k++) step(s, pilot.world, real, a);
            r.maxX = std::max(r.maxX, s.x);
            if (s.dead) break;
            if (s.x >= goal) { r.ok = true; break; }
        }
        if (!r.ok) pilot.onDeath();
    }
    r.maxDrift = pilot.maxDrift;
    std::printf("%-28s %s  attempts=%d  maxX=%.0f  drift=%.1f\n", name.c_str(), r.ok ? "PASS" : "FAIL",
                r.attempts, r.maxX, r.maxDrift);
    return r;
}

int main() {
    int fails = 0;
    auto chk = [&](Result r) { if (!r.ok) fails++; };

    // 1) cube: ground + spikes + a block
    {
        std::vector<Obj> o;
        for (int i = 0; i < 400; i++) o.push_back(solid(i * 30.f, 60.f, 30, 60));  // ground top = 90
        for (float x : {600.f, 660.f, 900.f, 930.f, 1300.f}) o.push_back(spike(x, 90 + 8));
        o.push_back(solid(1100, 105, 30, 30));
        State s; s.x = 0; s.y = 90 + 15;
        chk(run("cube spikes+block", o, s, 2000, false, 1.00f));
        chk(run("cube spikes (g+1.5%)", o, s, 2000, false, 1.015f));
    }
    // 2) ship corridor with a staircase of gaps
    {
        std::vector<Obj> o;
        for (int i = 0; i < 400; i++) o.push_back(solid(i * 30.f, 60.f, 30, 60));
        o.push_back({Kind::PortalMode, 200, 240, 15, 45, uint8_t(Mode::Ship)});
        for (int i = 0; i < 30; i++) {
            float x = 600 + i * 120.f;
            float gapY = 240 + 50 * std::sin(i * 0.9f);
            o.push_back(solid(x, gapY + 90 + 60, 30, 120));  // upper pillar
            o.push_back(solid(x, gapY - 90 - 45, 30, 90));   // lower pillar
        }
        State s; s.x = 0; s.y = 105;
        chk(run("ship pillars", o, s, 4000, false, 1.00f));
    }
    // 3) wave zigzag
    {
        std::vector<Obj> o;
        for (int i = 0; i < 400; i++) o.push_back(solid(i * 30.f, 60.f, 30, 60));
        o.push_back({Kind::PortalMode, 200, 240, 15, 45, uint8_t(Mode::Wave)});
        for (int i = 0; i < 25; i++) {
            float x = 500 + i * 150.f;
            bool up = i % 2;
            o.push_back(solid(x, up ? 120 : 360, 30, up ? 60 : 60));  // alternating blocks
        }
        State s; s.x = 0; s.y = 105;
        chk(run("wave alternating blocks", o, s, 3800, false, 1.00f));
    }
    // 4) platformer: wall + spikes
    {
        std::vector<Obj> o;
        for (int i = 0; i < 300; i++) o.push_back(solid(i * 30.f, 60.f, 30, 60));
        o.push_back(solid(400, 105, 30, 30));
        o.push_back(spike(560, 98)); o.push_back(spike(590, 98));
        o.push_back(solid(800, 105, 30, 30)); o.push_back(solid(800, 135, 30, 30));
        State s; s.x = 0; s.y = 105;
        chk(run("platformer wall+spikes", o, s, 1200, true, 1.00f));
    }
    std::printf("%s\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return fails;
}
