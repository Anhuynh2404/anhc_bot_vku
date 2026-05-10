// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// dijkstra.cpp — Uniform-cost search (A* with h=0)
// ──────────────────────────────────────────────────────────────────────────

#include <limits>
#include <queue>
#include <vector>
#include <functional>

#include "anhc_multi_planner/algorithm_base.hpp"

namespace anhc_multi_planner
{

struct DijkCell
{
  int    index;
  double g;
  bool operator>(const DijkCell & r) const { return g > r.g; }
};

CellPath runDijkstra(
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

  std::vector<double> dist(TOTAL, std::numeric_limits<double>::infinity());
  std::vector<int>    came(TOTAL, -1);
  std::vector<bool>   visited(TOTAL, false);

  std::priority_queue<DijkCell, std::vector<DijkCell>, std::greater<DijkCell>> pq;
  dist[S_IDX] = 0.0;
  pq.push({S_IDX, 0.0});

  int iter = 0;
  std::vector<std::pair<int,int>> nbrs;

  while (!pq.empty()) {
    if (cancel && cancel()) { return {}; }
    if (++iter > p.max_iterations)   { return {}; }

    auto cur = pq.top(); pq.pop();
    if (visited[cur.index]) { continue; }
    visited[cur.index] = true;

    if (cur.index == G_IDX) {
      return backtrace(came, G_IDX, W);
    }

    int cx = cur.index % W;
    int cy = cur.index / W;
    get8Neighbors(cx, cy, W, H, nbrs);

    for (auto [nx, ny] : nbrs) {
      int nidx = cellIndex(nx, ny, W);
      if (visited[nidx]) { continue; }
      if (!isTraversable(nx, ny, costmap, p.allow_unknown)) { continue; }

      double nd = dist[cur.index] + edgeCost(cx, cy, nx, ny, costmap, p.cost_penalty_factor);
      if (nd < dist[nidx]) {
        dist[nidx] = nd;
        came[nidx] = cur.index;
        pq.push({nidx, nd});
      }
    }
  }
  return {};
}

}  // namespace anhc_multi_planner
