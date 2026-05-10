// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// dstar_lite.cpp — D* Lite (Stentz & Likhachev 2002)
//   Backward incremental search from goal to start.
//   For static costmaps it behaves like a backward A*.
//   Designed for dynamic replanning; left extensible for future costmap updates.
// ──────────────────────────────────────────────────────────────────────────

#include <limits>
#include <queue>
#include <vector>
#include <functional>

#include "anhc_multi_planner/algorithm_base.hpp"

namespace anhc_multi_planner
{

// Key = pair<double,double>; min-heap on first then second element
using DKey = std::pair<double, double>;

struct DCell
{
  int  index;
  DKey key;
  bool operator>(const DCell & r) const { return key > r.key; }
};

static const double INF = std::numeric_limits<double>::infinity();

CellPath runDStarLite(
  int sx, int sy,
  int gx, int gy,
  nav2_costmap_2d::Costmap2D * costmap,
  const AlgoParams & p,
  std::function<bool()> cancel)
{
  const int W     = (int)costmap->getSizeInCellsX();
  const int H     = (int)costmap->getSizeInCellsY();
  const int TOTAL = W * H;
  const int S     = cellIndex(sx, sy, W);
  const int G     = cellIndex(gx, gy, W);

  // g[s] = best known cost from s to goal (backward)
  // rhs[s] = one-step lookahead of g
  std::vector<double> g(TOTAL, INF), rhs(TOTAL, INF);
  std::vector<bool>   inOpen(TOTAL, false);
  std::vector<int>    came(TOTAL, -1);  // next cell on path to goal

  rhs[G] = 0.0;

  auto calcKey = [&](int idx) -> DKey {
    int x = idx % W, y = idx / W;
    double mn = std::min(g[idx], rhs[idx]);
    return {mn + p.weight_heuristic * cellDist(x, y, sx, sy), mn};
  };

  std::priority_queue<DCell, std::vector<DCell>, std::greater<DCell>> U;
  U.push({G, calcKey(G)});
  inOpen[G] = true;

  int iter = 0;

  auto computeSP = [&]() {
    while (!U.empty()) {
      if (cancel && cancel()) return;
      if (++iter > p.dstar_max_iterations) return;

      auto top = U.top(); U.pop();
      int u = top.index;
      inOpen[u] = false;

      DKey ku = top.key;
      DKey ks = calcKey(u);
      if (ku < ks) {
        // Key outdated — re-insert
        U.push({u, ks}); inOpen[u] = true;
        continue;
      }

      if (g[u] > rhs[u]) {
        g[u] = rhs[u];
      } else {
        g[u] = INF;
        // Re-check self
        if (u != G) {
          // will be relaxed from neighbours below
        }
      }

      // Expand predecessors (for backward search, predecessors = 8-neighbours)
      int ux = u % W, uy = u / W;
      std::vector<std::pair<int,int>> nbrs;
      get8Neighbors(ux, uy, W, H, nbrs);

      for (auto [nx, ny] : nbrs) {
        int nidx = cellIndex(nx, ny, W);
        if (!isTraversable(nx, ny, costmap, p.allow_unknown)) continue;

        double c = edgeCost(nx, ny, ux, uy, costmap, p.cost_penalty_factor);
        if (rhs[nidx] > g[u] + c) {
          rhs[nidx] = g[u] + c;
          came[nidx] = u;
          if (!inOpen[nidx]) {
            U.push({nidx, calcKey(nidx)});
            inOpen[nidx] = true;
          }
        }
      }

      // Check start convergence
      if (g[S] == rhs[S]) break;
    }
  };

  computeSP();

  if (g[S] >= INF) return {};

  // Back-trace: follow came[] from start toward goal
  CellPath path;
  int idx = S;
  int safety = TOTAL;
  while (idx != G && --safety > 0) {
    path.emplace_back(idx % W, idx / W);
    int nxt = came[idx];
    if (nxt == -1) return {};
    idx = nxt;
  }
  path.emplace_back(gx, gy);
  return path;
}

}  // namespace anhc_multi_planner
