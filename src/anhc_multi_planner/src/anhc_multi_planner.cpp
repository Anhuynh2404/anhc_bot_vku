// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// anhc_multi_planner.cpp
// ──────────────────────────────────────────────────────────────────────────
// Nav2 GlobalPlanner plugin: dispatches to one of 7 algorithms.
// Algorithm can be switched at runtime without node restart via:
//   /planning/set_algorithm  (std_msgs/String)
// ──────────────────────────────────────────────────────────────────────────

#include "anhc_multi_planner/anhc_multi_planner.hpp"

#include <algorithm>
#include <stdexcept>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(anhc_multi_planner::AnhcMultiPlanner, nav2_core::GlobalPlanner)

// ── Forward declarations for the 7 algorithm free-functions ──────────────────
namespace anhc_multi_planner
{
CellPath runAstar(int, int, int, int, nav2_costmap_2d::Costmap2D *, const AlgoParams &, std::function<bool()>);
CellPath runDijkstra(int, int, int, int, nav2_costmap_2d::Costmap2D *, const AlgoParams &, std::function<bool()>);
CellPath runGreedyBFS(int, int, int, int, nav2_costmap_2d::Costmap2D *, const AlgoParams &, std::function<bool()>);
CellPath runThetaStar(int, int, int, int, nav2_costmap_2d::Costmap2D *, const AlgoParams &, std::function<bool()>);
CellPath runJPS(int, int, int, int, nav2_costmap_2d::Costmap2D *, const AlgoParams &, std::function<bool()>);
CellPath runRRTStar(int, int, int, int, nav2_costmap_2d::Costmap2D *, const AlgoParams &, std::function<bool()>);
CellPath runDStarLite(int, int, int, int, nav2_costmap_2d::Costmap2D *, const AlgoParams &, std::function<bool()>);
}  // namespace anhc_multi_planner

namespace anhc_multi_planner
{

// ═══════════════════════════════════════════════════════════════════════════
// Lifecycle interface
// ═══════════════════════════════════════════════════════════════════════════

void AnhcMultiPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_         = parent;
  name_         = name;
  tf_           = tf;
  costmap_      = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"AnhcMultiPlanner: parent lifecycle node is expired"};
  }

  logger_ = node->get_logger();
  clock_  = node->get_clock();

  // ── Declare & read parameters ───────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".algorithm", rclcpp::ParameterValue(std::string("astar")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".weight_heuristic", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_penalty_factor", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(1000000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rrt_star_max_nodes", rclcpp::ParameterValue(5000));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rrt_star_step_size", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rrt_star_rewire_radius", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".dstar_max_iterations", rclcpp::ParameterValue(2000000));

  current_algorithm_ = node->get_parameter(name_ + ".algorithm").as_string();
  interpolation_resolution_ =
    node->get_parameter(name_ + ".interpolation_resolution").as_double();
  params_.allow_unknown       = node->get_parameter(name_ + ".allow_unknown").as_bool();
  params_.weight_heuristic    = node->get_parameter(name_ + ".weight_heuristic").as_double();
  params_.cost_penalty_factor = node->get_parameter(name_ + ".cost_penalty_factor").as_double();
  params_.max_iterations      = node->get_parameter(name_ + ".max_iterations").as_int();
  params_.rrt_star_max_nodes  = node->get_parameter(name_ + ".rrt_star_max_nodes").as_int();
  params_.rrt_star_step_size  = node->get_parameter(name_ + ".rrt_star_step_size").as_double();
  params_.rrt_star_rewire_radius = node->get_parameter(name_ + ".rrt_star_rewire_radius").as_double();
  params_.dstar_max_iterations = node->get_parameter(name_ + ".dstar_max_iterations").as_int();

  // ── Register all algorithms ─────────────────────────────────────────────
  registerAlgorithms();

  // Validate startup algorithm
  if (algorithms_.find(current_algorithm_) == algorithms_.end()) {
    RCLCPP_WARN(logger_,
      "[%s] Unknown startup algorithm '%s', falling back to 'astar'.",
      name_.c_str(), current_algorithm_.c_str());
    current_algorithm_ = "astar";
  }

  // ── Subscribe to runtime algorithm switch topic ─────────────────────────
  // Use the underlying rclcpp node handle from the lifecycle node
  algo_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/planning/set_algorithm",
    rclcpp::QoS(1).reliable(),
    std::bind(&AnhcMultiPlanner::onSetAlgorithm, this, std::placeholders::_1));

  RCLCPP_INFO(logger_,
    "[%s] AnhcMultiPlanner configured. Active algorithm: '%s'. "
    "Switch via: ros2 topic pub --once /planning/set_algorithm "
    "std_msgs/msg/String \"data: '<name>'\"",
    name_.c_str(), current_algorithm_.c_str());
}

void AnhcMultiPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "[%s] Cleaning up AnhcMultiPlanner", name_.c_str());
  algo_sub_.reset();
}

void AnhcMultiPlanner::activate()
{
  RCLCPP_INFO(logger_, "[%s] Activating AnhcMultiPlanner", name_.c_str());
}

void AnhcMultiPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "[%s] Deactivating AnhcMultiPlanner", name_.c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// Algorithm registry
// ═══════════════════════════════════════════════════════════════════════════

void AnhcMultiPlanner::registerAlgorithms()
{
  algorithms_["astar"]      = runAstar;
  algorithms_["dijkstra"]   = runDijkstra;
  algorithms_["greedy_bfs"] = runGreedyBFS;
  algorithms_["theta_star"] = runThetaStar;
  algorithms_["jps"]        = runJPS;
  algorithms_["rrt_star"]   = runRRTStar;
  algorithms_["dstar_lite"] = runDStarLite;
}

// ═══════════════════════════════════════════════════════════════════════════
// Runtime algorithm switch callback
// ═══════════════════════════════════════════════════════════════════════════

void AnhcMultiPlanner::onSetAlgorithm(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & requested = msg->data;

  if (algorithms_.find(requested) == algorithms_.end()) {
    RCLCPP_WARN(logger_,
      "[%s] /planning/set_algorithm: unknown algorithm '%s'. "
      "Valid options: astar | dijkstra | greedy_bfs | theta_star | "
      "jps | rrt_star | dstar_lite",
      name_.c_str(), requested.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(algo_mutex_);
    current_algorithm_ = requested;
  }

  RCLCPP_INFO(logger_,
    "[%s] Algorithm switched → '%s' (takes effect on next createPlan call).",
    name_.c_str(), requested.c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// createPlan — public entry point called by PlannerServer
// ═══════════════════════════════════════════════════════════════════════════

nav_msgs::msg::Path AnhcMultiPlanner::createPlan(
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

  // ── Dispatch to selected algorithm ──────────────────────────────────────
  std::string algo_name;
  AlgorithmFn algo_fn;
  {
    std::lock_guard<std::mutex> lock(algo_mutex_);
    algo_name = current_algorithm_;
    algo_fn   = algorithms_.at(algo_name);
  }

  RCLCPP_DEBUG(logger_,
    "[%s] Planning [%s]: cell (%d,%d) → (%d,%d)",
    name_.c_str(), algo_name.c_str(), sx, sy, gx, gy);

  CellPath cell_path = algo_fn(sx, sy, gx, gy, costmap_, params_, cancel_checker);

  if (cell_path.empty()) {
    RCLCPP_WARN(logger_,
      "[%s] [%s] found no path from (%.3f,%.3f) to (%.3f,%.3f).",
      name_.c_str(), algo_name.c_str(),
      start.pose.position.x, start.pose.position.y,
      goal.pose.position.x, goal.pose.position.y);
    return path;
  }

  // ── Convert cell path → world poses ─────────────────────────────────────
  auto waypoints = interpolatePath(cell_path, costmap_, path.header);

  for (auto & ps : waypoints) {
    ps.header = path.header;
  }
  if (!waypoints.empty()) {
    waypoints.back().pose.orientation = goal.pose.orientation;
  }

  path.poses = std::move(waypoints);

  RCLCPP_INFO(logger_,
    "[%s] [%s] Path found: %zu poses.",
    name_.c_str(), algo_name.c_str(), path.poses.size());

  return path;
}

// ═══════════════════════════════════════════════════════════════════════════
// interpolatePath — cells → world-frame PoseStamped waypoints
// ═══════════════════════════════════════════════════════════════════════════

std::vector<geometry_msgs::msg::PoseStamped>
AnhcMultiPlanner::interpolatePath(
  const CellPath & cell_path,
  nav2_costmap_2d::Costmap2D * costmap,
  const std_msgs::msg::Header & header) const
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  if (cell_path.empty()) {
    return poses;
  }

  const double res  = costmap->getResolution();
  const double step = interpolation_resolution_;

  for (std::size_t i = 0; i + 1 < cell_path.size(); ++i) {
    double wx0, wy0, wx1, wy1;
    costmap->mapToWorld(cell_path[i].first,     cell_path[i].second,     wx0, wy0);
    costmap->mapToWorld(cell_path[i+1].first,   cell_path[i+1].second,   wx1, wy1);

    double seg_dx  = wx1 - wx0;
    double seg_dy  = wy1 - wy0;
    double seg_len = std::hypot(seg_dx, seg_dy);
    double seg_yaw = std::atan2(seg_dy, seg_dx);

    if (seg_len < 1e-6) {
      continue;
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, seg_yaw);

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

  // Always include exact goal cell
  double gwx, gwy;
  costmap->mapToWorld(cell_path.back().first, cell_path.back().second, gwx, gwy);
  geometry_msgs::msg::PoseStamped gps;
  gps.header = header;
  gps.pose.position.x = gwx;
  gps.pose.position.y = gwy;
  gps.pose.position.z = 0.0;
  gps.pose.orientation.w = 1.0;
  poses.push_back(gps);

  // Remove near-duplicate first pose if start step < resolution
  if (poses.size() > 1) {
    double d = std::hypot(
      poses[1].pose.position.x - poses[0].pose.position.x,
      poses[1].pose.position.y - poses[0].pose.position.y);
    if (d < res * 0.5) {
      poses.erase(poses.begin());
    }
  }

  return poses;
}

}  // namespace anhc_multi_planner
