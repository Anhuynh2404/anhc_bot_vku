// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// anhc_astar_planner.hpp
// ──────────────────────────────────────────────────────────────────────────
// Nav2 GlobalPlanner plugin: from-scratch 8-connected weighted A* on the
// Nav2 global costmap.  No nav2_navfn or any other Nav2 planner dependency.
// ──────────────────────────────────────────────────────────────────────────

#ifndef ANHC_ASTAR_PLANNER__ANHC_ASTAR_PLANNER_HPP_
#define ANHC_ASTAR_PLANNER__ANHC_ASTAR_PLANNER_HPP_

#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/buffer.h"

namespace anhc_astar_planner
{

// ── A* node stored in the open / closed sets ────────────────────────────────
struct AstarCell
{
  int   index;   ///< flat cell index in the costmap
  double f;      ///< f = g + w * h  (priority key)

  // min-heap: smallest f on top
  bool operator>(const AstarCell & rhs) const { return f > rhs.f; }
};

// ── Main planner class ───────────────────────────────────────────────────────
class AnhcAstarPlanner : public nav2_core::GlobalPlanner
{
public:
  AnhcAstarPlanner() = default;
  ~AnhcAstarPlanner() override = default;

  // ── nav2_core::GlobalPlanner interface ──────────────────────────────────
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  // ── A* core ─────────────────────────────────────────────────────────────

  /// Run A* from (sx,sy) to (gx,gy) on the given costmap.
  /// Returns ordered world-frame waypoints, or empty vector on failure/cancellation.
  std::vector<geometry_msgs::msg::PoseStamped> runAstar(
    int sx, int sy,
    int gx, int gy,
    nav2_costmap_2d::Costmap2D * costmap,
    std::function<bool()> cancel_checker);

  /// Euclidean heuristic in grid cells.
  inline double heuristic(int x1, int y1, int x2, int y2) const
  {
    double dx = static_cast<double>(x1 - x2);
    double dy = static_cast<double>(y1 - y2);
    return std::hypot(dx, dy);
  }

  /// Edge cost: Euclidean distance * (1 + normalised costmap cost).
  /// This steers the planner away from inflated / near-obstacle cells.
  inline double edgeCost(
    int x0, int y0, int x1, int y1,
    nav2_costmap_2d::Costmap2D * costmap) const
  {
    double dist = std::hypot(
      static_cast<double>(x1 - x0),
      static_cast<double>(y1 - y0));
    unsigned char c = costmap->getCost(x1, y1);
    // Normalise cost to [0, 1] and add as a penalty multiplier.
    double penalty = 1.0 + (static_cast<double>(c) / 255.0) * cost_penalty_factor_;
    return dist * penalty;
  }

  /// Convert (mx, my) cell index → flat linear index.
  inline int toIndex(int x, int y, int width) const
  {
    return y * width + x;
  }

  /// Interpolate between consecutive waypoints at resolution_ spacing.
  std::vector<geometry_msgs::msg::PoseStamped> interpolatePath(
    const std::vector<std::pair<int, int>> & cell_path,
    nav2_costmap_2d::Costmap2D * costmap,
    const std_msgs::msg::Header & header) const;

  // ── ROS / Nav2 handles ───────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("AnhcAstarPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string global_frame_;

  // ── Configurable parameters ──────────────────────────────────────────────
  /// Distance between consecutive path waypoints [metres].
  double interpolation_resolution_{0.05};
  /// Allow planning through UNKNOWN cells (cost == 255 by default).
  bool allow_unknown_{true};
  /// Heuristic weight w.  1.0 → admissible (optimal); >1.0 → faster.
  double weight_heuristic_{1.0};
  /// Multiplier applied to costmap cost in the edge-cost formula.
  double cost_penalty_factor_{3.0};
  /// Hard cap on A* iterations to prevent runaway planning.
  int max_iterations_{1000000};
};

}  // namespace anhc_astar_planner

#endif  // ANHC_ASTAR_PLANNER__ANHC_ASTAR_PLANNER_HPP_
