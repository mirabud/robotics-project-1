/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *  Contributor: Pham Cong Trang <phamcongtranghd@gmail.com>
 *  Contributor: Mitchell Sayer <mitchell4408@gmail.com>
 */

#include <algorithm>
#include <string>
#include <memory>
#include "angles/angles.h"


#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "turtlebot3/pose_controller.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_pose_controller
{

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

void PoseController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_position", rclcpp::ParameterValue(
      1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_orientation",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
      1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(
      1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
      0.1));

  node->get_parameter(plugin_name_ + ".kp_position", kp_position_);
  node->get_parameter(plugin_name_ + ".kp_orientation", kp_orientation_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void PoseController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type pose_controller::PoseController",
    plugin_name_.c_str());
  global_pub_.reset();
}

void PoseController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type pose_controller::PoseController\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
  global_pub_->on_activate();
}

void PoseController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type pose_controller::PoseController\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
  global_pub_->on_deactivate();
}

void PoseController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  (void) speed_limit;
  (void) percentage;
}

geometry_msgs::msg::TwistStamped PoseController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;
  
  if (global_plan_.poses.empty()){
    throw nav2_core::PlannerException("No valid global plan!");
  }

  auto transformed_plan = transformGlobalPlan(pose);
  RCLCPP_INFO(logger_, "Transformed plan size: %zu", transformed_plan.poses.size());

  // Target next waypoint in the transformed plan
  const auto &target_pose = global_plan_.poses.front();
  
 
  
  // compute positional error
  // Calculate the distance to the goal
  double dx = target_pose.pose.position.x - pose.pose.position.x;  // x_goal - x_robot
  double dy = target_pose.pose.position.y - pose.pose.position.y;  // y_goal - y_robot

  double dist_to_goal = hypot(dx, dy);  // sqrt(dx^2 + dy^2)

  // Calculate the angle to the goal (target direction)
  double angle_to_goal = atan2(dy, dx);

  // Get the current orientation (yaw) of the robot
  double ori_robot = tf2::getYaw(pose.pose.orientation);
  // Function to normalize an angle to the range [-pi, pi]
 

  // Calculate the orientation error (angle difference between the robot's current orientation and the goal direction)
  double angle= angle_to_goal - ori_robot;
  double orientation_error = atan2(sin(angle),cos(angle));

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;


// Proportional control for orientation
  double angular_vel = kp_orientation_ * orientation_error;  // Scale angular velocity based on error
  angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);  // Clamp angular velocity

  // If orientation error is large, prioritize rotation over linear motion
  if (fabs(orientation_error) > 0.3) {
    cmd_vel.twist.linear.x = 0.0;  // Stop forward motion
    cmd_vel.twist.angular.z = angular_vel;  // Rotate proportionally
  } else {
    // Proportional control for linear velocity
    double linear_vel = kp_position_ * dist_to_goal;
    linear_vel = std::max(linear_vel, 0.1);  // Ensure a minimum velocity
    linear_vel = std::clamp(linear_vel, 0.0, max_linear_vel_); 

    cmd_vel.twist.linear.x = linear_vel;  // Move forward
    cmd_vel.twist.angular.z = angular_vel;  // Small corrections for orientation
  }
   RCLCPP_INFO(logger_, "Linear velocity: %f, Angular velocity: %f", cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
  
  return cmd_vel;
}

void PoseController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;
}

nav_msgs::msg::Path
PoseController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Original mplementation taken fron nav2_dwb_controller

  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }







  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
    min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // From the closest point, look for the first point that's further then dist_threshold from the
  // robot. These points are definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
    });

  // Helper function for the transform below. Transforms a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, costmap_ros_->getBaseFrameID(),
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PoseController::transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance
) const
{
  // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    auto transform = tf->lookupTransform(
      frame,
      in_pose.header.frame_id,
      tf2::TimePointZero
    );
    if (
      (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Transform data too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(),
        frame.c_str()
      );
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Data time: %ds %uns, Transform time: %ds %uns",
        in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec,
        transform.header.stamp.sec,
        transform.header.stamp.nanosec
      );
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
    return false;
  }
  return false;
}

}  // namespace nav2_pose_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_pose_controller::PoseController, nav2_core::Controller)
