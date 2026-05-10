// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// anhc_multi_planner.hpp
// ──────────────────────────────────────────────────────────────────────────
// Nav2 GlobalPlanner plugin: single entry-point that dispatches to one of
// 7 hot-swappable path-search algorithms.
//
// Runtime algorithm switch:
//   ros2 topic pub --once /planning/set_algorithm std_msgs/msg/String
//     "data: 'dijkstra'"
//
// Valid names: astar | dijkstra | greedy_bfs | theta_star |
//              jps | rrt_star | dstar_lite
// ──────────────────────────────────────────────────────────────────────────

#ifndef ANHC_MULTI_PLANNER__ANHC_MULTI_PLANNER_HPP_
#define ANHC_MULTI_PLANNER__ANHC_MULTI_PLANNER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/buffer.h"

#include "anhc_multi_planner/algorithm_base.hpp"

namespace anhc_multi_planner
{

class AnhcMultiPlanner : public nav2_core::GlobalPlanner
{
public:
  AnhcMultiPlanner() = default;
  ~AnhcMultiPlanner() override = default;

  // ── nav2_core::GlobalPlanner interface ───────────────────────────────────
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
  // ── Algorithm registry ───────────────────────────────────────────────────
  /// Registered algorithm functions keyed by name.
  std::map<std::string, AlgorithmFn> algorithms_;

  /// Currently selected algorithm name.
  std::string current_algorithm_;
  mutable std::mutex algo_mutex_;

  // ── Internal helpers ─────────────────────────────────────────────────────
  void registerAlgorithms();

  /// Convert cell path → interpolated world-frame PoseStamped waypoints.
  std::vector<geometry_msgs::msg::PoseStamped> interpolatePath(
    const CellPath & cell_path,
    nav2_costmap_2d::Costmap2D * costmap,
    const std_msgs::msg::Header & header) const;

  /// Callback: /planning/set_algorithm
  void onSetAlgorithm(const std_msgs::msg::String::SharedPtr msg);

  // ── ROS / Nav2 handles ───────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("AnhcMultiPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string global_frame_;

  /// Subscriber for runtime algorithm switching.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr algo_sub_;

  // ── Shared algorithm parameters ──────────────────────────────────────────
  double interpolation_resolution_{0.05};
  AlgoParams params_;
};

}  // namespace anhc_multi_planner

#endif  // ANHC_MULTI_PLANNER__ANHC_MULTI_PLANNER_HPP_
