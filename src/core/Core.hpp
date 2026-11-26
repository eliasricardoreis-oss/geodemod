// Core.hpp — Geode-independent world model + physics simulator.
// Everything here is plain C++17 so it can be unit-tested without Geometry Dash.
// All constants live in Config and are meant to be CALIBRATED against the real game
// (see README "Calibration"). Units: GD units (30 = 1 block), seconds.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace pf {

constexpr float TICK = 1.f / 240.f;  // physics substep (GD 2.2 runs 240 TPS)
constexpr int DECISION_TICKS = 4;    // one planner decision = 4 ticks = 1/60 s

enum class Mode : uint8_t { Cube, Ship, Ball, Ufo, Wave, Robot, Spider, Swing };

enum class Kind : uint8_t {
    Solid, Hazard,
    PadYellow, PadPink, PadRed, PadBlue,
    OrbYellow, OrbPink, OrbRed, OrbBlue, OrbGreen, OrbBlack,
    PortalMode,     // arg = Mode
    PortalGravity,  // arg = 1 flipped / 0 normal
    PortalSize,     // arg = 1 mini / 0 normal
    PortalSpeed     // arg = 0..4  (0.5x, 1x, 2x, 3x, 4x)
};

struct Obj {
    Kind kind;
    float x, y, hw, hh;  // centre + half extents (axis aligned)
    uint8_t arg = 0;
};

struct Config {
    // --- horizontal speed (units / s) ---
    float speedX[5] = {251.16f, 311.58f, 387.42f, 468.0f, 576.0f};
    // --- vertical physics (units/s, units/s^2) ---
    float g = 3449.5f;
    float cubeJump = 670.8f, miniJumpScale = 0.85f, termFall = 900.f;
    float shipUp = 2760.f, shipDown = 2070.f, shipMaxUp = 480.f, shipMaxDown = 384.f;
    float ballG = 2070.f, ballKick = 120.f, ballTerm = 900.f;
    float ufoG = 1725.f, ufoJump = 450.f, ufoTerm = 400.f;
    float robotG = 3449.5f, robotJump = 540.f, robotBoostT = 0.25f, robotBoostGrav = 0.2f;
    float spiderG = 3449.5f;
    float swingG = 2400.f, swingTerm = 480.f, swingFlipDamp = 0.5f;
    float waveSlope = 1.f, miniWaveSlope = 2.f;
    // --- pads / orbs (launch speeds) ---
    float padYellow = 960.f, padPink = 600.f, padRed = 1200.f, padBlueKick = 240.f;
    float orbYellow = 700.f, orbPink = 450.f, orbRed = 900.f, orbBlueKick = 240.f,
          orbGreen = 620.f, orbBlack = 1000.f;
    // --- hitboxes ---
    float hitHalf = 15.f, miniHalf = 9.f, innerHalf = 4.5f, hazScale = 0.85f, waveHazHalf = 5.f;
    float snapTol = 6.f;  // forgiveness when landing on a block edge
    // --- level ---
    float floorY = 90.f;
    float deathY = 3000.f;
    // --- platformer ---
    bool platformer = false;
    float platScale = 0.8f, platAccel = 3000.f;

    // --- planner ---
    int horizonTicks = 720;      // look-ahead per segment (3 s)
    int maxNodes = 40000;        // node cap per segment
    float hWeight = 1.0f;        // 1.0 classic, ~1.4 platformer
    float clickCost = 0.6f;      // prefer fewer clicks
    float safeGap = 8.f, gapCost = 0.4f;  // prefer clearance from hazards
    float driftTol = 14.f;       // replan when real != predicted by this much
    int lowWater = 30;           // start planning next segment when this few decisions remain
};

struct Act {
    uint8_t bits = 0;  // 1 = jump/hold, 2 = left, 4 = right
    bool jump() const { return bits & 1; }
    int dir() const { return ((bits >> 2) & 1) - ((bits >> 1) & 1); }
};

struct State {
    float x = 0, y = 105, vy = 0, vx = 0;
    Mode mode = Mode::Cube;
    bool flip = false, mini = false, hold = false, onGround = true, dead = false, hasCeil = false;
    uint8_t speed = 1;
    float boost = 0;
    float floorY = 90, ceilY = 1e9f;
    float minGap = 1e9f;
    int32_t used[4] = {-1, -1, -1, -1};
    uint8_t usedHead = 0;
    uint32_t tick = 0;
};

inline float half(const State& s, const Config& c) { return s.mini ? c.miniHalf : c.hitHalf; }
inline bool overlap(float ax, float ay, float ahw, float ahh, float bx, float by, float bhw, float bhh) {
    return std::fabs(ax - bx) < ahw + bhw && std::fabs(ay - by) < ahh + bhh;
}

class World {
public:
    void build(std::vector<Obj> v) {
        objs = std::move(v);
        std::sort(objs.begin(), objs.end(), [](const Obj& a, const Obj& b) { return a.x < b.x; });
        maxHW = 0;
        for (auto& o : objs) maxHW = std::max(maxHW, o.hw);
    }
    const std::vector<Obj>& all() const { return objs; }
    bool empty() const { return objs.empty(); }

    // visit every object whose centre may overlap [x0, x1]
    template <class F> void near(float x0, float x1, F&& f) const {
        auto it = std::lower_bound(objs.begin(), objs.end(), x0 - maxHW,
                                   [](const Obj& o, float v) { return o.x < v; });
        for (; it != objs.end() && it->x <= x1 + maxHW; ++it) f(*it, int(it - objs.begin()));
    }

    // nearest solid face from y0 going in direction dir (+1 up / -1 down) over [x-hw, x+hw]
    bool rayY(float x, float hw, float y0, int dir, float& faceY) const {
        bool found = false;
        float best = dir > 0 ? 1e18f : -1e18f;
        near(x - hw, x + hw, [&](const Obj& o, int) {
            if (o.kind != Kind::Solid) return;
            if (std::fabs(o.x - x) >= hw + o.hw - 0.5f) return;
            if (dir > 0) {
                float f = o.y - o.hh;
                if (f >= y0 - 0.5f && f < best) { best = f; found = true; }
            } else {
                float f = o.y + o.hh;
                if (f <= y0 + 0.5f && f > best) { best = f; found = true; }
            }
        });
        faceY = best;
        return found;
    }

    // index of the last mode portal at or before x (-1 if none)
    int lastModePortal(float x) const {
        auto it = std::upper_bound(objs.begin(), objs.end(), x,
                                   [](float v, const Obj& o) { return v < o.x; });
        int n = 0;
        while (it != objs.begin() && n++ < 20000) {
            --it;
            if (it->kind == Kind::PortalMode) return int(it - objs.begin());
        }
        return -1;
    }

private:
    std::vector<Obj> objs;
    float maxHW = 0;
};

// ---------------------------------------------------------------- simulator

inline void setBounds(State& s, const Config& c, float py) {
    float lo = std::max(c.floorY, std::round(py / 30.f) * 30.f - 150.f);
    s.floorY = lo; s.ceilY = lo + 300.f; s.hasCeil = true;
}
inline void clearBounds(State& s, const Config& c) { s.floorY = c.floorY; s.ceilY = 1e9f; s.hasCeil = false; }

inline bool isUsed(const State& s, int idx) {
    for (int i = 0; i < 4; i++) if (s.used[i] == idx) return true;
    return false;
}
inline void markUsed(State& s, int idx) { s.used[s.usedHead] = idx; s.usedHead = (s.usedHead + 1) & 3; }

inline void flipGravity(State& s) { s.flip = !s.flip; s.onGround = false; }

inline void applyObject(State& s, const Config& c, const Obj& o, int idx, bool click) {
    const float sgn = s.flip ? -1.f : 1.f;
    switch (o.kind) {
        case Kind::PadYellow: s.vy = sgn * c.padYellow; s.onGround = false; break;
        case Kind::PadPink:   s.vy = sgn * c.padPink;   s.onGround = false; break;
        case Kind::PadRed:    s.vy = sgn * c.padRed;    s.onGround = false; break;
        case Kind::PadBlue:   flipGravity(s); s.vy = (s.flip ? 1.f : -1.f) * -c.padBlueKick; break;
        case Kind::OrbYellow: if (!click) return; s.vy = sgn * c.orbYellow; s.onGround = false; break;
        case Kind::OrbPink:   if (!click) return; s.vy = sgn * c.orbPink;   s.onGround = false; break;
        case Kind::OrbRed:    if (!click) return; s.vy = sgn * c.orbRed;    s.onGround = false; break;
        case Kind::OrbBlue:   if (!click) return; flipGravity(s); s.vy = (s.flip ? 1.f : -1.f) * -c.orbBlueKick; break;
        case Kind::OrbGreen:  if (!click) return; flipGravity(s); s.vy = (s.flip ? 1.f : -1.f) * -c.orbGreen; break;
        case Kind::OrbBlack:  if (!click) return; s.vy = -sgn * c.orbBlack; s.onGround = false; break;
        case Kind::PortalMode: {
            Mode m = Mode(o.arg);
            s.mode = m; s.boost = 0;
            if (m == Mode::Cube || m == Mode::Robot) clearBounds(s, c); else setBounds(s, c, o.y);
            break;
        }
        case Kind::PortalGravity: s.flip = o.arg != 0; s.vy *= 0.5f; s.onGround = false; break;
        case Kind::PortalSize:    s.mini = o.arg != 0; break;
        case Kind::PortalSpeed:   s.speed = std::min<uint8_t>(o.arg, 4); break;
        default: return;
    }
    markUsed(s, idx);
}

// Advance exactly one physics tick.
inline void step(State& s, const World& w, const Config& c, Act a) {
    if (s.dead) return;
    const bool hold = a.jump();
    const bool click = hold && !s.hold;
    s.hold = hold;
    s.minGap = 1e9f;
    const float ph = half(s, c);

    // 1) pads / orbs / portals
    w.near(s.x - ph, s.x + ph, [&](const Obj& o, int idx) {
        if (o.kind == Kind::Solid || o.kind == Kind::Hazard) return;
        if (!overlap(s.x, s.y, ph, ph, o.x, o.y, o.hw, o.hh) || isUsed(s, idx)) return;
        applyObject(s, c, o, idx, click);
    });

    // 2) horizontal velocity
    const float spd = c.speedX[s.speed];
    float dx;
    if (c.platformer) {
        float target = a.dir() * spd * c.platScale;
        float acc = c.platAccel * TICK;
        s.vx += std::clamp(target - s.vx, -acc, acc);
        dx = s.vx * TICK;
    } else {
        dx = spd * TICK;
    }
    const float xSpeed = c.platformer ? std::fabs(s.vx) : spd;

    // 3) vertical dynamics (written in world coords, sgn = +1 normal gravity)
    const float sgn = s.flip ? -1.f : 1.f;
    auto clampLocal = [&](float lo, float hi) {  // clamp vy in gravity-local frame
        float l = std::clamp(s.vy * sgn, lo, hi);
        s.vy = l * sgn;
    };
    switch (s.mode) {
        case Mode::Cube:
            if (s.onGround && hold) { s.vy = c.cubeJump * (s.mini ? c.miniJumpScale : 1.f) * sgn; s.onGround = false; }
            s.vy -= c.g * sgn * TICK; clampLocal(-c.termFall, 1e9f);
            break;
        case Mode::Ship:
            s.vy += sgn * (hold ? c.shipUp : -c.shipDown) * TICK;
            clampLocal(-c.shipMaxDown, c.shipMaxUp);
            break;
        case Mode::Ball:
            if (s.onGround && click) { flipGravity(s); s.vy = (s.flip ? 1.f : -1.f) * -c.ballKick; }
            else { s.vy -= c.ballG * sgn * TICK; }
            clampLocal(-c.ballTerm, c.ballTerm);
            break;
        case Mode::Ufo:
            if (click) s.vy = sgn * c.ufoJump;
            s.vy -= c.ufoG * sgn * TICK; clampLocal(-c.ufoTerm, c.ufoTerm);
            break;
        case Mode::Wave: {
            float slope = s.mini ? c.miniWaveSlope : c.waveSlope;
            s.vy = sgn * (hold ? 1.f : -1.f) * xSpeed * slope;
            break;
        }
        case Mode::Robot: {
            if (s.onGround && click) { s.vy = c.robotJump * sgn; s.boost = c.robotBoostT; s.onGround = false; }
            float gg = c.robotG;
            if (!hold) s.boost = 0;
            if (s.boost > 0) { gg *= c.robotBoostGrav; s.boost -= TICK; }
            s.vy -= gg * sgn * TICK; clampLocal(-c.termFall, 1e9f);
            break;
        }
        case Mode::Spider: {
            if (s.onGround && click) {
                float face;
                if (w.rayY(s.x, ph, s.y, int(sgn), face)) {
                    flipGravity(s);
                    s.y = face - sgn * ph;
                    s.vy = 0; s.onGround = true;
                    break;
                }
                if (s.hasCeil) {  // fall back to ceiling/floor bound
                    flipGravity(s);
                    s.y = (sgn > 0 ? s.ceilY : s.floorY) - sgn * ph;
                    s.vy = 0; s.onGround = true;
                    break;
                }
            }
            s.vy -= c.spiderG * sgn * TICK; clampLocal(-c.termFall, 1e9f);
            break;
        }
        case Mode::Swing:
            if (click) { flipGravity(s); s.vy *= c.swingFlipDamp; }
            {
                float sg2 = s.flip ? -1.f : 1.f;
                s.vy -= c.swingG * sg2 * TICK;
                float l = std::clamp(s.vy * sg2, -c.swingTerm, c.swingTerm);
                s.vy = l * sg2;
            }
            break;
    }

    // 4) integrate
    if (s.mode != Mode::Spider || !s.onGround) s.onGround = false;  // re-established by collisions
    s.x += dx;
    s.y += s.vy * TICK;

    // 5) bounds
    const float sg = s.flip ? -1.f : 1.f;
    if (s.y - ph < s.floorY) {
        s.y = s.floorY + ph;
        if (s.vy < 0) s.vy = 0;
        if (sg > 0) s.onGround = true;
    }
    if (s.hasCeil && s.y + ph > s.ceilY) {
        s.y = s.ceilY - ph;
        if (s.vy > 0) s.vy = 0;
        if (sg < 0) s.onGround = true;
    }
    if (s.y > c.deathY) { s.dead = true; return; }

    // 6) solids + hazards
    const bool wave = s.mode == Mode::Wave;
    const float hp = wave ? c.waveHazHalf : ph * c.hazScale;
    w.near(s.x - ph, s.x + ph, [&](const Obj& o, int) {
        if (s.dead) return;
        if (o.kind == Kind::Solid) {
            float px = (ph + o.hw) - std::fabs(s.x - o.x);
            float py = (ph + o.hh) - std::fabs(s.y - o.y);
            if (px <= 0 || py <= 0) return;
            if (py <= std::max(px, c.snapTol)) {
                float out = (s.y >= o.y) ? 1.f : -1.f;
                s.y = o.y + out * (o.hh + ph);
                if (s.vy * out < 0) s.vy = 0;
                if (out == sg) s.onGround = true;
            } else {
                if (overlap(s.x, s.y, c.innerHalf, c.innerHalf, o.x, o.y, o.hw, o.hh)) { s.dead = true; return; }
                s.x = o.x + ((s.x < o.x) ? -1.f : 1.f) * (ph + o.hw);
            }
        } else if (o.kind == Kind::Hazard) {
            if (overlap(s.x, s.y, hp, hp, o.x, o.y, o.hw, o.hh)) { s.dead = true; return; }
            float gx = std::fabs(s.x - o.x) - (hp + o.hw);
            float gy = std::fabs(s.y - o.y) - (hp + o.hh);
            s.minGap = std::min(s.minGap, std::max({gx, gy, 0.f}));
        }
    });
    s.tick++;
}

}  // namespace pf
