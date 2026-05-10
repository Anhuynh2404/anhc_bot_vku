// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// jps.cpp — Jump Point Search (JPS)
//   Accelerated A* for uniform grids using symmetry pruning.
//   JPS skips interior grid nodes by "jumping" to canonical successors,
//   dramatically reducing the open-set size in obstacle-sparse environments.
//
//   Implementation: JPS with 8-connectivity, costmap-aware obstacle check.
//   Note: JPS is designed for uniform-cost grids; cost_penalty_factor is
//   applied at jump endpoints (conservative safe approximation).
// ──────────────────────────────────────────────────────────────────────────

#include <cmath>
#include <limits>
#include <queue>
#include <vector>
#include <functional>

#include "anhc_multi_planner/algorithm_base.hpp"

namespace anhc_multi_planner
{

// ── Internal helpers ─────────────────────────────────────────────────────────

static inline bool inBounds(int x, int y, int W, int H)
{
  return x >= 0 && x < W && y >= 0 && y < H;
}

static inline bool blocked(int x, int y, int W, int H,
  nav2_costmap_2d::Costmap2D * costmap, bool allow_unknown)
{
  if (!inBounds(x, y, W, H)) { return true; }
  return !isTraversable(x, y, costmap, allow_unknown);
}

// ── JPS jump ─────────────────────────────────────────────────────────────────
/// Returns -1 if no jump point found, otherwise returns flat index of jump point.
static int jump(
  int cx, int cy, int dx, int dy,
  int gx, int gy,
  int W, int H,
  nav2_costmap_2d::Costmap2D * costmap,
  bool allow_unknown,
  int max_steps)
{
  int steps = 0;
  while (true) {
    cx += dx;
    cy += dy;

    if (!inBounds(cx, cy, W, H)) { return -1; }
    if (blocked(cx, cy, W, H, costmap, allow_unknown)) { return -1; }
    if (++steps > max_steps) { return -1; }

    if (cx == gx && cy == gy) { return cellIndex(cx, cy, W); }

    // ── Check forced neighbours ─────────────────────────────────────────
    if (dx != 0 && dy != 0) {
      // Diagonal move: check for forced horizontal/vertical neighbours
      if ((blocked(cx - dx, cy, W, H, costmap, allow_unknown) &&
           !blocked(cx - dx, cy + dy, W, H, costmap, allow_unknown)) ||
          (blocked(cx, cy - dy, W, H, costmap, allow_unknown) &&
           !blocked(cx + dx, cy - dy, W, H, costmap, allow_unknown)))
      {
        return cellIndex(cx, cy, W);
      }
      // Recursive horizontal/vertical jumps
      if (jump(cx, cy, dx, 0, gx, gy, W, H, costmap, allow_unknown, max_steps) != -1 ||
          jump(cx, cy, 0, dy, gx, gy, W, H, costmap, allow_unknown, max_steps) != -1)
      {
        return cellIndex(cx, cy, W);
      }
    } else if (dx != 0) {
      // Horizontal
      if ((!blocked(cx, cy + 1, W, H, costmap, allow_unknown) &&
            blocked(cx - dx, cy + 1, W, H, costmap, allow_unknown)) ||
          (!blocked(cx, cy - 1, W, H, costmap, allow_unknown) &&
            blocked(cx - dx, cy - 1, W, H, costmap, allow_unknown)))
      {
        return cellIndex(cx, cy, W);
      }
    } else {
      // Vertical
      if ((!blocked(cx + 1, cy, W, H, costmap, allow_unknown) &&
            blocked(cx + 1, cy - dy, W, H, costmap, allow_unknown)) ||
          (!blocked(cx - 1, cy, W, H, costmap, allow_unknown) &&
            blocked(cx - 1, cy - dy, W, H, costmap, allow_unknown)))
      {
        return cellIndex(cx, cy, W);
      }
    }
  }
}

// ── JPS successor identification ─────────────────────────────────────────────
static void identifySuccessors(
  int cur_idx, int par_idx,
  int gx, int gy,
  int W, int H,
  nav2_costmap_2d::Costmap2D * costmap,
  const AlgoParams & p,
  const std::vector<double> & g_cost,
  std::vector<int> & came,
  std::vector<double> & g_out,
  std::vector<std::pair<int,int>> & succs)  // {index, g}
{
  succs.clear();
  int cx = cur_idx % W, cy = cur_idx / W;

  // Compute natural move direction from parent (or all dirs if start)
  const int DIRS[8][2] = {
    {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
  int num_dirs = 8;

  int pdx = 0, pdy = 0;
  if (par_idx != -1) {
    int px = par_idx % W, py = par_idx / W;
    pdx = (cx > px) ? 1 : (cx < px) ? -1 : 0;
    pdy = (cy > py) ? 1 : (cy < py) ? -1 : 0;
    num_dirs = 1;
    // Expand only natural and forced
    // For simplicity, scan all 8 dirs but restrict to "natural" pruned set
    // (full JPS pruning rules applied inside jump())
    num_dirs = 8;
    (void)pdx; (void)pdy;
  }

  for (int k = 0; k < num_dirs; ++k) {
    int dx = DIRS[k][0], dy = DIRS[k][1];

    // Skip blocked immediate neighbour
    if (blocked(cx + dx, cy + dy, W, H, costmap, p.allow_unknown)) { continue; }
    // Diagonal: both cardinal dirs must be free
    if (dx != 0 && dy != 0) {
      if (blocked(cx + dx, cy, W, H, costmap, p.allow_unknown) ||
          blocked(cx, cy + dy, W, H, costmap, p.allow_unknown)) { continue; }
    }

    int jp = jump(cx, cy, dx, dy, gx, gy, W, H, costmap, p.allow_unknown, p.max_iterations);
    if (jp == -1) { continue; }

    int jx = jp % W, jy = jp / W;
    double ng = g_cost[cur_idx] + edgeCost(cx, cy, jx, jy, costmap, p.cost_penalty_factor);

    if (ng < g_out[jp]) {
      g_out[jp] = ng;
      came[jp]  = cur_idx;
      succs.emplace_back(jp, 0);
    }
  }
}

// ── Main JPS entry ────────────────────────────────────────────────────────────

struct JpsCell
{
  int    index;
  double f;
  bool operator>(const JpsCell & r) const { return f > r.f; }
};

CellPath runJPS(
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

  std::priority_queue<JpsCell, std::vector<JpsCell>, std::greater<JpsCell>> open;
  g[S_IDX] = 0.0;
  open.push({S_IDX, p.weight_heuristic * cellDist(sx, sy, gx, gy)});

  int iter = 0;
  std::vector<std::pair<int,int>> succs;

  while (!open.empty()) {
    if (cancel && cancel()) { return {}; }
    if (++iter > p.max_iterations)   { return {}; }

    auto cur = open.top(); open.pop();
    if (closed[cur.index]) { continue; }
    closed[cur.index] = true;

    if (cur.index == G_IDX) {
      return backtrace(came, G_IDX, W);
    }

    identifySuccessors(cur.index, came[cur.index],
      gx, gy, W, H, costmap, p, g, came, g, succs);

    for (auto [nidx, _] : succs) {
      if (closed[nidx]) { continue; }
      int nx = nidx % W, ny = nidx / W;
      double f = g[nidx] + p.weight_heuristic * cellDist(nx, ny, gx, gy);
      open.push({nidx, f});
    }
  }
  return {};
}

}  // namespace anhc_multi_planner
