// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// greedy_bfs.cpp — Greedy Best-First Search (h only, g ignored)
//   Fastest to compute; NOT guaranteed to find optimal path.
// ──────────────────────────────────────────────────────────────────────────

#include <queue>
#include <vector>
#include <functional>

#include "anhc_multi_planner/algorithm_base.hpp"

namespace anhc_multi_planner
{

struct GreedyCell
{
  int    index;
  double h;
  bool operator>(const GreedyCell & r) const { return h > r.h; }
};

CellPath runGreedyBFS(
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

  std::vector<int>  came(TOTAL, -1);
  std::vector<bool> visited(TOTAL, false);

  std::priority_queue<GreedyCell, std::vector<GreedyCell>, std::greater<GreedyCell>> pq;
  visited[S_IDX] = true;
  pq.push({S_IDX, cellDist(sx, sy, gx, gy)});

  int iter = 0;
  std::vector<std::pair<int,int>> nbrs;

  while (!pq.empty()) {
    if (cancel && cancel()) { return {}; }
    if (++iter > p.max_iterations)   { return {}; }

    auto cur = pq.top(); pq.pop();

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

      visited[nidx] = true;
      came[nidx]    = cur.index;
      pq.push({nidx, cellDist(nx, ny, gx, gy)});
    }
  }
  return {};
}

}  // namespace anhc_multi_planner
