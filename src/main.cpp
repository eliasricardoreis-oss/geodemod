// A* Pilot — Geode bridge (GD 2.2081, Geode v5).
// NOT compiled against the SDK by the author of this file: field/enum names marked
// "VERIFY" are the ones most likely to need a tweak against your Geode headers.
#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>

#include "Pilot.hpp"

using namespace geode::prelude;

namespace {

// ---- object classification ------------------------------------------------
// Portal / speed ids are stable GD object ids.  Pads/orbs use GameObjectType.
bool classify(GameObject* g, pf::Obj& o) {
    if (!g) return false;
    CCRect r = g->getObjectRect();  // VERIFY: bounding rect used for collisions
    o.x = r.getMidX(); o.y = r.getMidY();
    o.hw = r.size.width * 0.5f; o.hh = r.size.height * 0.5f;
    o.arg = 0;

    switch (g->m_objectID) {
        // mode portals
        case 12:   o.kind = pf::Kind::PortalMode; o.arg = uint8_t(pf::Mode::Cube);   return true;
        case 13:   o.kind = pf::Kind::PortalMode; o.arg = uint8_t(pf::Mode::Ship);   return true;
        case 47:   o.kind = pf::Kind::PortalMode; o.arg = uint8_t(pf::Mode::Ball);   return true;
        case 111:  o.kind = pf::Kind::PortalMode; o.arg = uint8_t(pf::Mode::Ufo);    return true;
        case 660:  o.kind = pf::Kind::PortalMode; o.arg = uint8_t(pf::Mode::Wave);   return true;
        case 745:  o.kind = pf::Kind::PortalMode; o.arg = uint8_t(pf::Mode::Robot);  return true;
        case 1331: o.kind = pf::Kind::PortalMode; o.arg = uint8_t(pf::Mode::Spider); return true;
        case 2926: o.kind = pf::Kind::PortalMode; o.arg = uint8_t(pf::Mode::Swing);  return true;  // VERIFY id
        // gravity / size
        case 10:   o.kind = pf::Kind::PortalGravity; o.arg = 0; return true;
        case 11:   o.kind = pf::Kind::PortalGravity; o.arg = 1; return true;
        case 99:   o.kind = pf::Kind::PortalSize; o.arg = 0; return true;
        case 101:  o.kind = pf::Kind::PortalSize; o.arg = 1; return true;
        // speed portals
        case 200:  o.kind = pf::Kind::PortalSpeed; o.arg = 0; return true;
        case 201:  o.kind = pf::Kind::PortalSpeed; o.arg = 1; return true;
        case 202:  o.kind = pf::Kind::PortalSpeed; o.arg = 2; return true;
        case 203:  o.kind = pf::Kind::PortalSpeed; o.arg = 3; return true;
        case 1334: o.kind = pf::Kind::PortalSpeed; o.arg = 4; return true;
        default: break;
    }
    switch (g->m_objectType) {  // VERIFY enum names against your headers
        case GameObjectType::Solid:        o.kind = pf::Kind::Solid;  return true;
        case GameObjectType::Slope:        o.kind = pf::Kind::Solid;  return true;  // approximation: box
        case GameObjectType::Hazard:       o.kind = pf::Kind::Hazard; return true;
        case GameObjectType::YellowJumpPad: o.kind = pf::Kind::PadYellow; return true;
        case GameObjectType::PinkJumpPad:   o.kind = pf::Kind::PadPink;   return true;
        case GameObjectType::RedJumpPad:    o.kind = pf::Kind::PadRed;    return true;
        case GameObjectType::GravityPad:    o.kind = pf::Kind::PadBlue;   return true;
        case GameObjectType::YellowJumpRing: o.kind = pf::Kind::OrbYellow; return true;
        case GameObjectType::PinkJumpRing:   o.kind = pf::Kind::OrbPink;   return true;
        case GameObjectType::RedJumpRing:    o.kind = pf::Kind::OrbRed;    return true;
        case GameObjectType::GravityRing:    o.kind = pf::Kind::OrbBlue;   return true;
        case GameObjectType::GreenRing:      o.kind = pf::Kind::OrbGreen;  return true;
        case GameObjectType::DropRing:       o.kind = pf::Kind::OrbBlack;  return true;
        default: return false;  // decoration, triggers, coins, dash/spider orbs: ignored
    }
}

pf::Mode readMode(PlayerObject* p) {
    if (p->m_isShip) return pf::Mode::Ship;
    if (p->m_isBird) return pf::Mode::Ufo;
    if (p->m_isBall) return pf::Mode::Ball;
    if (p->m_isDart) return pf::Mode::Wave;
    if (p->m_isRobot) return pf::Mode::Robot;
    if (p->m_isSpider) return pf::Mode::Spider;
    if (p->m_isSwing) return pf::Mode::Swing;
    return pf::Mode::Cube;
}

uint8_t readSpeed(float v) {  // m_playerSpeed: 0.7 / 0.9 / 1.1 / 1.3 / 1.6
    static const float t[5] = {0.7f, 0.9f, 1.1f, 1.3f, 1.6f};
    int best = 1; float bd = 1e9f;
    for (int i = 0; i < 5; i++) if (std::fabs(t[i] - v) < bd) { bd = std::fabs(t[i] - v); best = i; }
    return uint8_t(best);
}

}  // namespace

class $modify(PilotPlayLayer, PlayLayer) {
    struct Fields {
        pf::Pilot pilot;
        bool worldBuilt = false;
        bool jumpDown = false, leftDown = false, rightDown = false;
        float tickAcc = 0;
        uint32_t drawnVersion = 0;
        CCDrawNode* draw = nullptr;
        int attempts = 0;
    };

    bool enabled() { return Mod::get()->getSettingValue<bool>("enabled"); }

    // --- lifecycle -----------------------------------------------------------
    void setupHasCompleted() {
        PlayLayer::setupHasCompleted();
        auto f = m_fields.self();
        buildWorld();
        f->draw = CCDrawNode::create();
        m_objectLayer->addChild(f->draw, 1000);
        startAttempt();
    }

    void resetLevel() {
        PlayLayer::resetLevel();
        releaseAll();
        startAttempt();
    }

    void destroyPlayer(PlayerObject* p, GameObject* obj) {
        auto f = m_fields.self();
        if (p == m_player1 && f->worldBuilt && enabled() && !m_isTestMode) {
            f->pilot.noteActual(readState());
            f->pilot.onDeath();
            if (Mod::get()->getSettingValue<bool>("trace-log"))
                log::info("[A* Pilot] death at x={:.1f}; deaths={} maxDrift={:.1f}",
                          m_player1->getPositionX(), f->pilot.deaths, f->pilot.maxDrift);
        }
        PlayLayer::destroyPlayer(p, obj);
    }

    void postUpdate(float dt) {
        PlayLayer::postUpdate(dt);
        drive(dt);
    }

    // --- world ---------------------------------------------------------------
    void buildWorld() {
        auto f = m_fields.self();
        std::vector<pf::Obj> v;
        for (auto obj : CCArrayExt<GameObject*>(m_objects)) {
            pf::Obj o;
            if (classify(obj, o)) v.push_back(o);
        }
        f->pilot.world.build(std::move(v));
        f->worldBuilt = true;
        log::info("[A* Pilot] world built: {} relevant objects", f->pilot.world.all().size());
    }

    pf::State readState() {
        pf::State s;
        auto p = m_player1;
        auto f = m_fields.self();
        s.x = p->getPositionX();
        s.y = p->getPositionY();
        s.vy = float(p->m_yVelocity) * 60.f;  // VERIFY scale (units/frame@60 -> units/s)
        s.mode = readMode(p);
        s.flip = p->m_isUpsideDown;
        s.mini = p->m_vehicleSize < 0.9f;
        s.speed = readSpeed(p->m_playerSpeed);
        s.onGround = p->m_isOnGround;
        s.vx = float(p->m_platformerXVelocity) * 60.f;  // VERIFY name/scale
        s.floorY = f->pilot.cfg.floorY;
        if (s.mode != pf::Mode::Cube && s.mode != pf::Mode::Robot) {
            int i = f->pilot.world.lastModePortal(s.x);
            if (i >= 0) pf::setBounds(s, f->pilot.cfg, f->pilot.world.all()[i].y);
        }
        return s;
    }

    void startAttempt() {
        auto f = m_fields.self();
        if (!f->worldBuilt || !enabled()) return;
        auto& pilot = f->pilot;
        pilot.cfg.platformer = m_player1->m_isPlatformer;
        pilot.cfg.hWeight = pilot.cfg.platformer ? 1.4f : 1.0f;
        pilot.cfg.horizonTicks = int(Mod::get()->getSettingValue<double>("horizon-seconds") * 240.0);
        pilot.goalX = m_levelLength;  // VERIFY: end-of-level x (platformer: set to your end trigger)
        pilot.reset();
        f->tickAcc = 0;
        f->attempts++;
        pilot.preplan(readState(), double(Mod::get()->getSettingValue<int64_t>("preplan-ms")));
    }

    // --- control -------------------------------------------------------------
    void drive(float dt) {
        auto f = m_fields.self();
        if (!f->worldBuilt || !enabled() || m_player1->m_isDead || m_levelEndAnimationStarted) {
            return;
        }
        f->tickAcc += dt * 240.f;
        if (f->tickAcc < float(pf::DECISION_TICKS)) return;
        f->tickAcc -= float(pf::DECISION_TICKS);

        auto& pilot = f->pilot;
        pf::State actual = readState();
        pilot.noteActual(actual);
        pf::Act a = pilot.decide(actual, 800);
        press(a);

        if (Mod::get()->getSettingValue<bool>("trace-log")) {
            const auto& pr = pilot.predicted();
            if (pilot.cursor() > 0 && pilot.cursor() - 1 < pr.size())
                log::info("[A* Pilot] x={:.0f} real y={:.1f} pred y={:.1f}", actual.x, actual.y,
                          pr[pilot.cursor() - 1].y);
        }
        if (Mod::get()->getSettingValue<bool>("show-path") && f->drawnVersion != pilot.planVersion)
            redraw();
    }

    void press(pf::Act a) {
        auto f = m_fields.self();
        auto set = [&](bool want, bool& cur, PlayerButton b) {
            if (want == cur) return;
            cur = want;
            if (want) m_player1->pushButton(b); else m_player1->releaseButton(b);
        };
        set(a.jump(), f->jumpDown, PlayerButton::Jump);
        if (f->pilot.cfg.platformer) {
            set(a.dir() < 0, f->leftDown, PlayerButton::Left);
            set(a.dir() > 0, f->rightDown, PlayerButton::Right);
        }
    }

    void releaseAll() {
        auto f = m_fields.self();
        press(pf::Act{});
        f->jumpDown = f->leftDown = f->rightDown = false;
    }

    void redraw() {
        auto f = m_fields.self();
        f->drawnVersion = f->pilot.planVersion;
        if (!f->draw) return;
        f->draw->clear();
        const auto& pr = f->pilot.predicted();
        for (size_t i = f->pilot.cursor(); i + 1 < pr.size(); i++) {
            f->draw->drawSegment({pr[i].x, pr[i].y}, {pr[i + 1].x, pr[i + 1].y}, 1.f,
                                 {0.2f, 1.f, 0.4f, 0.9f});
        }
    }
};
