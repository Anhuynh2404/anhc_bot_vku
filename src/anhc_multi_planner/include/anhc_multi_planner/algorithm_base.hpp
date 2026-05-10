// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// algorithm_base.hpp
// ──────────────────────────────────────────────────────────────────────────
// Shared types and parameter struct used by every algorithm implementation.
// Every algorithm function must conform to AlgorithmFn.
// ──────────────────────────────────────────────────────────────────────────

#ifndef ANHC_MULTI_PLANNER__ALGORITHM_BASE_HPP_
#define ANHC_MULTI_PLANNER__ALGORITHM_BASE_HPP_

#include <functional>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace anhc_multi_planner
{

// ── Output type ──────────────────────────────────────────────────────────────
/// Ordered list of (col, row) grid cells from start to goal.
using CellPath = std::vector<std::pair<int, int>>;

// ── Shared tunable parameters ────────────────────────────────────────────────
struct AlgoParams
{
  /// Allow planning through UNKNOWN cells.
  bool allow_unknown{true};

  /// Heuristic weight for A*, Theta*, JPS.  1.0 = admissible.
  double weight_heuristic{1.0};

  /// Multiplier for costmap-cost edge penalty.
  double cost_penalty_factor{3.0};

  /// Hard cap on grid-search iterations.
  int max_iterations{1000000};

  // ── RRT* specific ────────────────────────────────────────────────────────
  /// Maximum number of tree nodes before giving up.
  int rrt_star_max_nodes{5000};

  /// Step length (in grid cells) for random extension.
  double rrt_star_step_size{3.0};

  /// Rewire radius (in grid cells).
  double rrt_star_rewire_radius{5.0};

  // ── D* Lite specific ─────────────────────────────────────────────────────
  int dstar_max_iterations{2000000};
};

// ── Canonical algorithm function signature ───────────────────────────────────
/**
 * @brief Run a path search from (sx,sy) to (gx,gy) on `costmap`.
 *
 * @param sx       Start column
 * @param sy       Start row
 * @param gx       Goal column
 * @param gy       Goal row
 * @param costmap  Raw Nav2 costmap pointer (read-only access)
 * @param params   Shared tunables
 * @param cancel   Return `true` to abort early (Nav2 cancellation hook)
 * @return         Ordered cell path start→goal, or empty on failure/cancel.
 */
using AlgorithmFn = std::function<CellPath(
    int sx, int sy,
    int gx, int gy,
    nav2_costmap_2d::Costmap2D * costmap,
    const AlgoParams & params,
    std::function<bool()> cancel)>;

// ── Common inline helpers (header-only, used by all algorithms) ──────────────

/// Flat linear index from (col, row).
inline int cellIndex(int x, int y, int width)
{
  return y * width + x;
}

/// Euclidean distance between two cells.
inline double cellDist(int x0, int y0, int x1, int y1)
{
  double dx = static_cast<double>(x1 - x0);
  double dy = static_cast<double>(y1 - y0);
  return std::hypot(dx, dy);
}

/// Edge traversal cost: Euclidean distance weighted by costmap value.
inline double edgeCost(
  int x0, int y0, int x1, int y1,
  nav2_costmap_2d::Costmap2D * costmap,
  double cost_penalty_factor)
{
  double dist = cellDist(x0, y0, x1, y1);
  unsigned char c = costmap->getCost(x1, y1);
  double penalty = 1.0 + (static_cast<double>(c) / 255.0) * cost_penalty_factor;
  return dist * penalty;
}

/// True if a cell is traversable (not lethal, and unknown only if allowed).
inline bool isTraversable(
  int x, int y,
  nav2_costmap_2d::Costmap2D * costmap,
  bool allow_unknown)
{
  unsigned char cost = costmap->getCost(x, y);
  if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    return false;
  }
  if (!allow_unknown && cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }
  return true;
}

/// 8-connected neighbour offsets: {dx, dy}
inline void get8Neighbors(int cx, int cy, int W, int H,
  std::vector<std::pair<int,int>> & out)
{
  static const int NDX[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
  static const int NDY[8] = {-1, -1, -1,  0, 0,  1, 1, 1};
  out.clear();
  for (int k = 0; k < 8; ++k) {
    int nx = cx + NDX[k];
    int ny = cy + NDY[k];
    if (nx >= 0 && nx < W && ny >= 0 && ny < H) {
      out.emplace_back(nx, ny);
    }
  }
}

/// Back-trace the came_from array to build a cell path.
inline CellPath backtrace(
  const std::vector<int> & came_from,
  int goal_idx, int W)
{
  CellPath path;
  int idx = goal_idx;
  while (idx != -1) {
    path.emplace_back(idx % W, idx / W);
    idx = came_from[idx];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace anhc_multi_planner

#endif  // ANHC_MULTI_PLANNER__ALGORITHM_BASE_HPP_
