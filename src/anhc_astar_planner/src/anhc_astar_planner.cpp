// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// anhc_astar_planner.cpp
// ──────────────────────────────────────────────────────────────────────────
// From-scratch 8-connected weighted A* global planner plugin for anhc_bot.
// Registered with Nav2's pluginlib as "anhc_astar_planner::AnhcAstarPlanner".
// ──────────────────────────────────────────────────────────────────────────

#include "anhc_astar_planner/anhc_astar_planner.hpp"

#include <algorithm>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Required by pluginlib so Nav2 can discover this class at runtime.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(anhc_astar_planner::AnhcAstarPlanner, nav2_core::GlobalPlanner)

namespace anhc_astar_planner
{

// ═══════════════════════════════════════════════════════════════════════════
// Lifecycle interface
// ═══════════════════════════════════════════════════════════════════════════

void AnhcAstarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_   = parent;
  name_   = name;
  tf_     = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"AnhcAstarPlanner: parent lifecycle node is expired"};
  }

  logger_ = node->get_logger();
  clock_  = node->get_clock();

  // ── Declare & read parameters ──────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution",
    rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".weight_heuristic",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_penalty_factor",
    rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations",
    rclcpp::ParameterValue(1000000));

  interpolation_resolution_ = node->get_parameter(
    name_ + ".interpolation_resolution").as_double();
  allow_unknown_ = node->get_parameter(
    name_ + ".allow_unknown").as_bool();
  weight_heuristic_ = node->get_parameter(
    name_ + ".weight_heuristic").as_double();
  cost_penalty_factor_ = node->get_parameter(
    name_ + ".cost_penalty_factor").as_double();
  max_iterations_ = node->get_parameter(
    name_ + ".max_iterations").as_int();

  RCLCPP_INFO(logger_,
    "[%s] AnhcAstarPlanner configured: "
    "interp_res=%.3f  allow_unknown=%s  w_h=%.2f  cost_penalty=%.2f  max_iter=%d",
    name_.c_str(),
    interpolation_resolution_,
    allow_unknown_ ? "true" : "false",
    weight_heuristic_,
    cost_penalty_factor_,
    max_iterations_);
}

void AnhcAstarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "[%s] Cleaning up AnhcAstarPlanner", name_.c_str());
}

void AnhcAstarPlanner::activate()
{
  RCLCPP_INFO(logger_, "[%s] Activating AnhcAstarPlanner", name_.c_str());
}

void AnhcAstarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "[%s] Deactivating AnhcAstarPlanner", name_.c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// createPlan — public entry point called by PlannerServer
// ═══════════════════════════════════════════════════════════════════════════

nav_msgs::msg::Path AnhcAstarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.stamp    = clock_->now();
  path.header.frame_id = global_frame_;

  // ── World → map cell conversion ─────────────────────────────────────────
  unsigned int sx_u, sy_u, gx_u, gy_u;
  if (!costmap_->worldToMap(
        start.pose.position.x, start.pose.position.y, sx_u, sy_u))
  {
    RCLCPP_WARN(logger_,
      "[%s] Start (%.3f, %.3f) is outside the costmap bounds.",
      name_.c_str(), start.pose.position.x, start.pose.position.y);
    return path;
  }
  if (!costmap_->worldToMap(
        goal.pose.position.x, goal.pose.position.y, gx_u, gy_u))
  {
    RCLCPP_WARN(logger_,
      "[%s] Goal (%.3f, %.3f) is outside the costmap bounds.",
      name_.c_str(), goal.pose.position.x, goal.pose.position.y);
    return path;
  }

  int sx = static_cast<int>(sx_u);
  int sy = static_cast<int>(sy_u);
  int gx = static_cast<int>(gx_u);
  int gy = static_cast<int>(gy_u);

  // ── Trivial same-cell case ───────────────────────────────────────────────
  if (sx == gx && sy == gy) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose   = goal.pose;
    path.poses.push_back(ps);
    return path;
  }

  // ── Run A* ──────────────────────────────────────────────────────────────
  RCLCPP_DEBUG(logger_,
    "[%s] Planning from cell (%d,%d) to (%d,%d)",
    name_.c_str(), sx, sy, gx, gy);

  auto waypoints = runAstar(sx, sy, gx, gy, costmap_, cancel_checker);

  if (waypoints.empty()) {
    RCLCPP_WARN(logger_,
      "[%s] A* found no path from (%.3f,%.3f) to (%.3f,%.3f).",
      name_.c_str(),
      start.pose.position.x, start.pose.position.y,
      goal.pose.position.x, goal.pose.position.y);
    return path;
  }

  // Stamp all waypoints
  for (auto & ps : waypoints) {
    ps.header = path.header;
  }

  // Override the orientation of the last waypoint with the goal orientation
  if (!waypoints.empty()) {
    waypoints.back().pose.orientation = goal.pose.orientation;
  }

  path.poses = std::move(waypoints);

  RCLCPP_INFO(logger_,
    "[%s] Path found: %zu poses", name_.c_str(), path.poses.size());

  return path;
}

// ═══════════════════════════════════════════════════════════════════════════
// runAstar — core algorithm
// ═══════════════════════════════════════════════════════════════════════════

std::vector<geometry_msgs::msg::PoseStamped> AnhcAstarPlanner::runAstar(
  int sx, int sy,
  int gx, int gy,
  nav2_costmap_2d::Costmap2D * costmap,
  std::function<bool()> cancel_checker)
{
  const int W = static_cast<int>(costmap->getSizeInCellsX());
  const int H = static_cast<int>(costmap->getSizeInCellsY());
  const int GOAL_IDX = toIndex(gx, gy, W);
  const int START_IDX = toIndex(sx, sy, W);
  const int TOTAL = W * H;

  // ── Per-cell state ───────────────────────────────────────────────────────
  std::vector<double> g_cost(TOTAL, std::numeric_limits<double>::infinity());
  std::vector<int>    came_from(TOTAL, -1);
  std::vector<bool>   closed(TOTAL, false);

  // ── Open set (min-heap on f) ─────────────────────────────────────────────
  std::priority_queue<AstarCell, std::vector<AstarCell>, std::greater<AstarCell>> open_set;

  g_cost[START_IDX] = 0.0;
  double h0 = weight_heuristic_ * heuristic(sx, sy, gx, gy);
  open_set.push({START_IDX, h0});

  // 8-connected neighbours: (dx, dy)
  const int NDX[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
  const int NDY[8] = {-1, -1, -1,  0, 0,  1, 1, 1};

  int iterations = 0;

  while (!open_set.empty()) {
    // Check for action cancellation from Nav2
    if (cancel_checker && cancel_checker()) {
      RCLCPP_INFO(logger_, "[%s] A* planning cancelled by Nav2.", name_.c_str());
      return {};
    }

    if (++iterations > max_iterations_) {
      RCLCPP_WARN(logger_,
        "[%s] A* hit max_iterations (%d). Aborting search.",
        name_.c_str(), max_iterations_);
      return {};
    }

    AstarCell current = open_set.top();
    open_set.pop();

    if (closed[current.index]) {
      continue;  // stale entry — skip
    }
    closed[current.index] = true;

    if (current.index == GOAL_IDX) {
      // ── Goal reached — back-trace path ──────────────────────────────────
      std::vector<std::pair<int, int>> cell_path;
      int idx = GOAL_IDX;
      while (idx != -1) {
        int cx = idx % W;
        int cy = idx / W;
        cell_path.emplace_back(cx, cy);
        idx = came_from[idx];
      }
      std::reverse(cell_path.begin(), cell_path.end());

      // Convert cells → interpolated world poses
      std_msgs::msg::Header hdr;   // header filled by caller
      return interpolatePath(cell_path, costmap, hdr);
    }

    // ── Expand neighbours ────────────────────────────────────────────────
    int cx = current.index % W;
    int cy = current.index / W;

    for (int k = 0; k < 8; ++k) {
      int nx = cx + NDX[k];
      int ny = cy + NDY[k];

      if (nx < 0 || nx >= W || ny < 0 || ny >= H) {
        continue;
      }

      int nidx = toIndex(nx, ny, W);
      if (closed[nidx]) {
        continue;
      }

      unsigned char cost = costmap->getCost(nx, ny);

      // ── Obstacle / unknown filtering ─────────────────────────────────
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      if (!allow_unknown_ && cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }

      double tentative_g = g_cost[current.index] + edgeCost(cx, cy, nx, ny, costmap);

      if (tentative_g < g_cost[nidx]) {
        g_cost[nidx]   = tentative_g;
        came_from[nidx] = current.index;
        double f = tentative_g + weight_heuristic_ * heuristic(nx, ny, gx, gy);
        open_set.push({nidx, f});
      }
    }
  }

  // Open set exhausted — no path found
  return {};
}

// ═══════════════════════════════════════════════════════════════════════════
// interpolatePath — cells → world-frame PoseStamped waypoints
// ═══════════════════════════════════════════════════════════════════════════

std::vector<geometry_msgs::msg::PoseStamped>
AnhcAstarPlanner::interpolatePath(
  const std::vector<std::pair<int, int>> & cell_path,
  nav2_costmap_2d::Costmap2D * costmap,
  const std_msgs::msg::Header & header) const
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  if (cell_path.empty()) {
    return poses;
  }

  const double res = costmap->getResolution();
  const double step = interpolation_resolution_;

  for (std::size_t i = 0; i + 1 < cell_path.size(); ++i) {
    // World coordinates of current and next cell centres
    double wx0, wy0, wx1, wy1;
    costmap->mapToWorld(
      cell_path[i].first, cell_path[i].second, wx0, wy0);
    costmap->mapToWorld(
      cell_path[i + 1].first, cell_path[i + 1].second, wx1, wy1);

    double seg_dx   = wx1 - wx0;
    double seg_dy   = wy1 - wy0;
    double seg_len  = std::hypot(seg_dx, seg_dy);
    double seg_yaw  = std::atan2(seg_dy, seg_dx);

    if (seg_len < 1e-6) {
      continue;
    }

    // Orientation quaternion for this segment
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, seg_yaw);

    // Insert waypoints at `step` metre intervals along the segment
    double t = 0.0;
    while (t < seg_len) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = header;
      ps.pose.position.x = wx0 + (seg_dx / seg_len) * t;
      ps.pose.position.y = wy0 + (seg_dy / seg_len) * t;
      ps.pose.position.z = 0.0;
      ps.pose.orientation = tf2::toMsg(q);
      poses.push_back(ps);
      t += step;
    }
  }

  // Always add the exact goal cell as the final waypoint
  double gwx, gwy;
  costmap->mapToWorld(
    cell_path.back().first, cell_path.back().second, gwx, gwy);

  geometry_msgs::msg::PoseStamped gps;
  gps.header = header;
  gps.pose.position.x = gwx;
  gps.pose.position.y = gwy;
  gps.pose.position.z = 0.0;
  // Orientation filled by createPlan() from the goal pose
  gps.pose.orientation.w = 1.0;
  poses.push_back(gps);

  // Avoid duplicate start-cell entry if step < resolution
  if (poses.size() > 1) {
    double sx0 = poses[0].pose.position.x;
    double sy0 = poses[0].pose.position.y;
    double sx1 = poses[1].pose.position.x;
    double sy1 = poses[1].pose.position.y;
    if (std::hypot(sx1 - sx0, sy1 - sy0) < res * 0.5) {
      poses.erase(poses.begin());
    }
  }

  return poses;
}

}  // namespace anhc_astar_planner
