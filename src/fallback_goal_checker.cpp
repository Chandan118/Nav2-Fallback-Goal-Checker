#include "nav2_fallback_goal_checker/fallback_goal_checker.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_fallback_goal_checker
{

void FallbackGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in FallbackGoalChecker::initialize");
  }

  clock_ = node->get_clock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fallback_xy_goal_tolerance", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fallback_yaw_goal_tolerance", rclcpp::ParameterValue(0.50));
    
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".fallback_timeout_sec", rclcpp::ParameterValue(5.0));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".fallback_xy_goal_tolerance", fallback_xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".fallback_yaw_goal_tolerance", fallback_yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".fallback_timeout_sec", fallback_timeout_sec_);

  reset();

  RCLCPP_INFO(
    logger_,
    "FallbackGoalChecker initialized. Strict(xy:%.2f, yaw:%.2f). Fallback(xy:%.2f, yaw:%.2f) after %.1fs",
    xy_goal_tolerance_, yaw_goal_tolerance_, 
    fallback_xy_goal_tolerance_, fallback_yaw_goal_tolerance_, fallback_timeout_sec_);
}

void FallbackGoalChecker::reset()
{
  near_goal_ = false;
  first_near_goal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

double FallbackGoalChecker::getDistance(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2)
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  return std::hypot(dx, dy);
}

double FallbackGoalChecker::getYawDiff(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2)
{
  tf2::Quaternion q1, q2;
  tf2::fromMsg(p1.orientation, q1);
  tf2::fromMsg(p2.orientation, q2);

  double yaw1 = tf2::getYaw(q1);
  double yaw2 = tf2::getYaw(q2);

  double diff = std::fabs(yaw1 - yaw2);
  if (diff > M_PI) {
    diff = 2.0 * M_PI - diff;
  }
  return diff;
}

bool FallbackGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  double dist = getDistance(query_pose, goal_pose);
  double yaw_diff = getYawDiff(query_pose, goal_pose);

  // Check strict tolerances
  if (dist <= xy_goal_tolerance_ && yaw_diff <= yaw_goal_tolerance_) {
    return true;
  }

  // Check if we are within fallback tolerances
  if (dist <= fallback_xy_goal_tolerance_ && yaw_diff <= fallback_yaw_goal_tolerance_) {
    if (!near_goal_) {
      // Robot just entered the fallback tolerance zone
      near_goal_ = true;
      first_near_goal_time_ = clock_->now();
      RCLCPP_DEBUG(logger_, "Robot entered fallback tolerance zone. Waiting for timeout.");
    } else {
      // Robot is currently in fallback tolerance zone, check if timeout exceeded
      double time_elapsed = (clock_->now() - first_near_goal_time_).seconds();
      if (time_elapsed >= fallback_timeout_sec_) {
        RCLCPP_WARN(
          logger_, 
          "Goal fallback timeout exceeded (%.1fs). Forcing goal complete.", 
          time_elapsed);
        return true;
      }
    }
  } else {
    // Robot left the fallback tolerance zone
    near_goal_ = false;
  }

  return false;
}

bool FallbackGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  // Note: we return the strict tolerances to the planners so they aim for the best
  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_goal_tolerance_);
  pose_tolerance.orientation = tf2::toMsg(q);

  vel_tolerance.linear.x = 0.0;
  vel_tolerance.linear.y = 0.0;
  vel_tolerance.linear.z = 0.0;
  vel_tolerance.angular.x = 0.0;
  vel_tolerance.angular.y = 0.0;
  vel_tolerance.angular.z = 0.0;

  return true;
}

}  // namespace nav2_fallback_goal_checker

PLUGINLIB_EXPORT_CLASS(nav2_fallback_goal_checker::FallbackGoalChecker, nav2_core::GoalChecker)
