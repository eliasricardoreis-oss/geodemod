// Pilot.hpp — turns the planner into a controller.
//  * preplan(): plans (almost) the whole level before the attempt starts
//  * decide(): called once per decision (every 4 ticks) with the REAL state;
//    compares it with the prediction, replans from reality if it drifted,
//    and pipelines the next segment while the current one is executing.
//  * onDeath(): bans the region where the real game killed us so the next
//    attempt takes a different route (this is what keeps attempts low in classic).
#pragma once
#include <chrono>
#include <unordered_set>

#include "Planner.hpp"

namespace pf {

class Pilot {
public:
    World world;
    Config cfg;
    float goalX = 1e9f;
    int deaths = 0;
    float maxDrift = 0;
    uint32_t planVersion = 0;  // bumps whenever the predicted path changes (for drawing)

    const std::vector<State>& predicted() const { return pred; }
    size_t cursor() const { return idx; }

    void reset() {  // new attempt (death memory is kept on purpose)
        plan.clear(); pred.clear(); idx = 0; planning = false; goalReached = false;
        last = Act{}; planVersion++;
    }

    // plan as much of the level as possible before the attempt starts
    void preplan(const State& root, double budgetMs) {
        auto t0 = std::chrono::steady_clock::now();
        auto elapsed = [&] {
            return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        };
        State r = root; r.hold = false;
        planFrom(r, false);
        while (!goalReached && elapsed() < budgetMs) {
            if (pred.empty()) break;
            beginSegment(pred.back(), true);
        }
    }

    Act decide(State actual, int nodeBudget) {
        actual.hold = last.jump();
        if (idx <= plan.size() && !pred.empty() && idx < pred.size()) {
            const State& p = pred[idx];
            float d = std::fabs(p.y - actual.y) + std::fabs(p.x - actual.x) * 0.25f;
            maxDrift = std::max(maxDrift, d);
            if (p.mode != actual.mode || p.flip != actual.flip || d > cfg.driftTol) {
                plan.clear(); pred.clear(); idx = 0; planning = false; goalReached = false;
            }
        }
        if (plan.empty()) {
            planFrom(actual, false);  // synchronous: the real game is waiting on us
        } else if (!planning && !goalReached && plan.size() - std::min(idx, plan.size()) < size_t(cfg.lowWater)) {
            planner.begin(&world, &cfg, pred.back(), goalX, &banned);
            planning = true;
        }
        if (planning) {
            auto st = planner.step(nodeBudget);
            if (st != Planner::Status::Running) commitSegment(true);
        }
        if (idx < plan.size()) { last = plan[idx++]; return last; }
        return last;
    }

    void onDeath() {
        deaths++;
        // ban the last ~0.4 s of predicted positions before the real death
        size_t hi = std::min(idx, pred.size());
        size_t lo = hi > 24 ? hi - 24 : 0;
        for (size_t i = lo; i < hi; i++) banned.insert(coarseKey(pred[i].x, pred[i].y, pred[i].mode));
        if (!lastActual.dead) banned.insert(coarseKey(lastActual.x, lastActual.y, lastActual.mode));
        reset();
    }
    void noteActual(const State& s) { lastActual = s; }
    void clearMemory() { banned.clear(); deaths = 0; maxDrift = 0; }

private:
    void planFrom(const State& s, bool append) {
        beginSegment(s, append);
    }
    void beginSegment(const State& s, bool append) {
        planner.begin(&world, &cfg, s, goalX, &banned);
        while (planner.step(4000) == Planner::Status::Running) {}
        planning = true;
        commitSegment(append);
    }
    void commitSegment(bool append) {
        std::vector<Act> a; std::vector<State> p;
        planner.result(a, p);
        if (planner.reachedGoal()) goalReached = true;
        planning = false;
        if (a.empty()) {
            if (!append) { plan.clear(); pred = p; idx = 0; planVersion++; }
            else goalReached = true;  // nothing more to plan (dead end): stop pipelining
            return;
        }
        if (!append) { plan = a; pred = p; idx = 0; }
        else {
            plan.insert(plan.end(), a.begin(), a.end());
            pred.insert(pred.end(), p.begin() + 1, p.end());
        }
        planVersion++;
    }

    Planner planner;
    std::vector<Act> plan;
    std::vector<State> pred;
    size_t idx = 0;
    bool planning = false, goalReached = false;
    Act last;
    State lastActual;
    std::unordered_set<uint64_t> banned;
};

}  // namespace pf
