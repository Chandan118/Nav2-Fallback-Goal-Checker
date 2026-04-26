#ifndef NAV2_FALLBACK_GOAL_CHECKER__FALLBACK_GOAL_CHECKER_HPP_
#define NAV2_FALLBACK_GOAL_CHECKER__FALLBACK_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_fallback_goal_checker
{

/**
 * @class FallbackGoalChecker
 * @brief Goal Checker plugin that evaluates if the robot has reached its goal.
 *        It uses strict tolerances initially, but if the robot is stuck near
 *        the goal for too long, it falls back to looser tolerances to allow completion.
 */
class FallbackGoalChecker : public nav2_core::GoalChecker
{
public:
  FallbackGoalChecker() = default;
  ~FallbackGoalChecker() override = default;

  /**
   * @brief Initialize the goal checker
   * @param parent Pointer to parent node
   * @param plugin_name Name of the plugin
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Reset the internal state of the goal checker
   */
  void reset() override;

  /**
   * @brief Check if the goal has been reached
   * @param query_pose Current robot pose
   * @param goal_pose Target goal pose
   * @param velocity Current robot velocity
   * @return true if goal is reached, false otherwise
   */
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  /**
   * @brief Get the current goal tolerances
   * @param pose_tolerance Output parameter for pose tolerance
   * @param vel_tolerance Output parameter for velocity tolerance
   * @return true if successful
   */
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  // Strict Tolerances
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;

  // Fallback (Looser) Tolerances
  double fallback_xy_goal_tolerance_;
  double fallback_yaw_goal_tolerance_;
  
  // Timing parameters for fallback
  double fallback_timeout_sec_;

  // State
  rclcpp::Time first_near_goal_time_;
  bool near_goal_;
  std::string plugin_name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("FallbackGoalChecker")};

  /**
   * @brief Helper to calculate distance between poses
   */
  double getDistance(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2);
    
  /**
   * @brief Helper to calculate yaw difference
   */
  double getYawDiff(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2);
};

}  // namespace nav2_fallback_goal_checker

#endif  // NAV2_FALLBACK_GOAL_CHECKER__FALLBACK_GOAL_CHECKER_HPP_
