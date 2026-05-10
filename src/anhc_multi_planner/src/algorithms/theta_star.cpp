// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// theta_star.cpp — Theta* (Any-Angle Path Planning)
//   Extension of A* that allows path segments along any angle by checking
//   line-of-sight from the grandparent, producing smoother, shorter paths.
// ──────────────────────────────────────────────────────────────────────────

#include <cmath>
#include <limits>
#include <queue>
#include <vector>
#include <functional>

#include "anhc_multi_planner/algorithm_base.hpp"

namespace anhc_multi_planner
{

/// Bresenham line-of-sight check on the costmap.
static bool hasLoS(
  int x0, int y0, int x1, int y1,
  nav2_costmap_2d::Costmap2D * costmap,
  bool allow_unknown)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  int cx = x0, cy = y0;
  const int W = static_cast<int>(costmap->getSizeInCellsX());
  const int H = static_cast<int>(costmap->getSizeInCellsY());

  while (true) {
    if (cx < 0 || cx >= W || cy < 0 || cy >= H) {
      return false;
    }
    if (!isTraversable(cx, cy, costmap, allow_unknown)) {
      return false;
    }
    if (cx == x1 && cy == y1) {
      return true;
    }
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; cx += sx; }
    if (e2 <  dx) { err += dx; cy += sy; }
  }
}

struct TsCell
{
  int    index;
  double f;
  bool operator>(const TsCell & r) const { return f > r.f; }
};

CellPath runThetaStar(
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

  std::priority_queue<TsCell, std::vector<TsCell>, std::greater<TsCell>> open;
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

      // ── Theta* parent relaxation ─────────────────────────────────────
      int par = came[cur.index];
      double tentative_g;

      if (par != -1) {
        // Try grandparent line-of-sight (the key Theta* step)
        int px = par % W, py = par / W;
        if (hasLoS(px, py, nx, ny, costmap, p.allow_unknown)) {
          tentative_g = g[par] + edgeCost(px, py, nx, ny, costmap, p.cost_penalty_factor);
          if (tentative_g < g[nidx]) {
            g[nidx]    = tentative_g;
            came[nidx] = par;
            double f   = tentative_g + p.weight_heuristic * cellDist(nx, ny, gx, gy);
            open.push({nidx, f});
            continue;
          }
        }
      }

      // Standard A* update
      tentative_g = g[cur.index] + edgeCost(cx, cy, nx, ny, costmap, p.cost_penalty_factor);
      if (tentative_g < g[nidx]) {
        g[nidx]    = tentative_g;
        came[nidx] = cur.index;
        double f   = tentative_g + p.weight_heuristic * cellDist(nx, ny, gx, gy);
        open.push({nidx, f});
      }
    }
  }
  return {};
}

}  // namespace anhc_multi_planner
