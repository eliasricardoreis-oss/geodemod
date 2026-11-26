// Planner.hpp — incremental (anytime) A* over input decisions.
// One graph edge = one Act held for DECISION_TICKS physics ticks.
//   g = elapsed ticks + penalties (clicks, low clearance)
//   h = remaining distance / speed   (ticks)
// Classic: every path takes the same time, so f is ~constant and ties are broken
// by depth => behaves like a memoised depth-first search that backtracks out of
// dead ends, preferring few clicks and safe clearance.
// Platformer: g really is time, so A* finds the fastest route.
#pragma once
#include <cstdint>
#include <queue>
#include <unordered_set>
#include <vector>

#include "Core.hpp"

namespace pf {

inline uint64_t mix64(uint64_t z) {
    z += 0x9e3779b97f4a7c15ULL;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

// fine key: dedupe inside the search
inline uint64_t fineKey(const State& s) {
    uint64_t h = mix64(uint64_t(int64_t(std::floor(s.x / 3.f))));
    h = mix64(h ^ uint64_t(int64_t(std::floor(s.y / 3.f))));
    h = mix64(h ^ uint64_t(int64_t(std::floor(s.vy / 30.f))));
    h = mix64(h ^ uint64_t(int64_t(std::floor(s.vx / 40.f))));
    uint64_t f = uint64_t(s.mode) | (uint64_t(s.flip) << 4) | (uint64_t(s.mini) << 5) |
                 (uint64_t(s.speed) << 6) | (uint64_t(s.hold) << 9) | (uint64_t(s.onGround) << 10) |
                 (uint64_t(s.boost > 0) << 11);
    return mix64(h ^ (f << 40));
}

// coarse key: used for the "death memory" (banned regions)
inline uint64_t coarseKey(float x, float y, Mode m) {
    uint64_t h = mix64(uint64_t(int64_t(std::floor(x / 16.f))));
    h = mix64(h ^ uint64_t(int64_t(std::floor(y / 16.f))));
    return mix64(h ^ (uint64_t(m) << 56));
}

class Planner {
public:
    enum class Status { Idle, Running, Done, Failed };

    void begin(const World* w, const Config* c, const State& root, float goalX,
               const std::unordered_set<uint64_t>* banned) {
        W = w; C = c; goal = goalX; ban = banned;
        nodes.clear(); seen.clear();
        open = decltype(open)();
        expansions = 0; goalNode = -1; best = 0; finalNode = -1; status_ = Status::Running;
        nodes.push_back({root, -1, 0, 0.f, 0});
        seen.insert(fineKey(root));
        open.push({h(root), 0, 0});
        acts.clear();
        if (C->platformer) { for (uint8_t b : {0, 1, 2, 3, 4, 5}) acts.push_back(b); }
        else { acts.push_back(0); acts.push_back(1); }
    }

    Status status() const { return status_; }
    bool reachedGoal() const { return goalNode >= 0; }
    int expanded() const { return expansions; }

    Status step(int budget) {
        if (status_ != Status::Running) return status_;
        while (budget-- > 0) {
            if (open.empty()) { finish(); break; }
            QItem q = open.top(); open.pop();
            const Node n = nodes[q.idx];  // copy: nodes may reallocate
            if (n.depth * DECISION_TICKS >= C->horizonTicks) { finalNode = q.idx; status_ = Status::Done; break; }
            if (++expansions > C->maxNodes) { finish(); break; }
            for (uint8_t b : acts) {
                Act a; a.bits = b;
                State t = n.s;
                float gap = 1e9f;
                for (int k = 0; k < DECISION_TICKS; k++) {
                    pf::step(t, *W, *C, a);
                    if (t.dead) break;
                    gap = std::min(gap, t.minGap);
                    if (t.x >= goal) break;
                }
                if (t.dead) continue;
                if (ban && ban->count(coarseKey(t.x, t.y, t.mode))) continue;
                if (!seen.insert(fineKey(t)).second) continue;
                float pen = 0;
                if (a.jump() && !n.s.hold) pen += C->clickCost;
                if (gap < C->safeGap) pen += (C->safeGap - gap) * C->gapCost;
                Node c{t, q.idx, b, n.cost + DECISION_TICKS + pen, n.depth + 1};
                int id = int(nodes.size());
                nodes.push_back(c);
                if (t.x >= goal) { goalNode = id; finalNode = id; status_ = Status::Done; return status_; }
                if (c.depth > nodes[best].depth ||
                    (c.depth == nodes[best].depth && c.s.x > nodes[best].s.x)) best = id;
                open.push({c.cost + C->hWeight * h(t), c.depth, id});
            }
        }
        return status_;
    }

    // result: actions + predicted state at the start of every action (+ final state)
    void result(std::vector<Act>& outActs, std::vector<State>& outStates) const {
        outActs.clear(); outStates.clear();
        if (finalNode < 0) return;
        std::vector<int> chain;
        for (int i = finalNode; i >= 0; i = nodes[i].parent) chain.push_back(i);
        std::reverse(chain.begin(), chain.end());
        for (size_t i = 0; i < chain.size(); i++) {
            outStates.push_back(nodes[chain[i]].s);
            if (i + 1 < chain.size()) { Act a; a.bits = nodes[chain[i + 1]].act; outActs.push_back(a); }
        }
    }

private:
    struct Node { State s; int parent; uint8_t act; float cost; int depth; };
    struct QItem {
        float f; int depth; int idx;
        bool operator<(const QItem& o) const { return f != o.f ? f > o.f : depth < o.depth; }
    };

    float h(const State& s) const {
        float spd = C->speedX[s.speed] * (C->platformer ? C->platScale : 1.f);
        return std::max(0.f, goal - s.x) / (spd * TICK);
    }
    void finish() {  // out of budget / dead ends: take the deepest survivor
        finalNode = best;
        status_ = (best == 0) ? Status::Failed : Status::Done;
    }

    const World* W = nullptr; const Config* C = nullptr;
    const std::unordered_set<uint64_t>* ban = nullptr;
    float goal = 0;
    std::vector<Node> nodes;
    std::priority_queue<QItem> open;
    std::unordered_set<uint64_t> seen;
    std::vector<uint8_t> acts;
    int expansions = 0, goalNode = -1, best = 0, finalNode = -1;
    Status status_ = Status::Idle;
};

}  // namespace pf
