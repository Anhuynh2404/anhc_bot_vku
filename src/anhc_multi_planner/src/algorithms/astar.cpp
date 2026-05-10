// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// astar.cpp — Weighted 8-connected A* (ported from anhc_astar_planner)
// ──────────────────────────────────────────────────────────────────────────

#include <limits>
#include <queue>
#include <vector>
#include <functional>

#include "anhc_multi_planner/algorithm_base.hpp"

namespace anhc_multi_planner
{

struct PqCell
{
  int    index;
  double f;
  bool operator>(const PqCell & r) const { return f > r.f; }
};

CellPath runAstar(
  int sx, int sy,
  int gx, int gy,
  nav2_costmap_2d::Costmap2D * costmap,
  const AlgoParams & p,
  std::function<bool()> cancel)
{
  const int W     = static_cast<int>(costmap->getSizeInCellsX());
  const int H     = static_cast<int>(costmap->getSizeInCellsY());
  const int TOTAL = W * H;
  const int S_IDX = cellIndex(sx, sy, W);
  const int G_IDX = cellIndex(gx, gy, W);

  std::vector<double> g(TOTAL, std::numeric_limits<double>::infinity());
  std::vector<int>    came(TOTAL, -1);
  std::vector<bool>   closed(TOTAL, false);

  std::priority_queue<PqCell, std::vector<PqCell>, std::greater<PqCell>> open;
  g[S_IDX] = 0.0;
  open.push({S_IDX, p.weight_heuristic * cellDist(sx, sy, gx, gy)});

  int iter = 0;
  std::vector<std::pair<int,int>> nbrs;

  while (!open.empty()) {
    if (cancel && cancel()) { return {}; }
    if (++iter > p.max_iterations)   { return {}; }

    auto cur = open.top(); open.pop();
    if (closed[cur.index]) { continue; }
    closed[cur.index] = true;

    if (cur.index == G_IDX) {
      return backtrace(came, G_IDX, W);
    }

    int cx = cur.index % W;
    int cy = cur.index / W;
    get8Neighbors(cx, cy, W, H, nbrs);

    for (auto [nx, ny] : nbrs) {
      int nidx = cellIndex(nx, ny, W);
      if (closed[nidx]) { continue; }
      if (!isTraversable(nx, ny, costmap, p.allow_unknown)) { continue; }

      double tentative = g[cur.index] + edgeCost(cx, cy, nx, ny, costmap, p.cost_penalty_factor);
      if (tentative < g[nidx]) {
        g[nidx]    = tentative;
        came[nidx] = cur.index;
        double f   = tentative + p.weight_heuristic * cellDist(nx, ny, gx, gy);
        open.push({nidx, f});
      }
    }
  }
  return {};
}

}  // namespace anhc_multi_planner
