#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_smc_controller/smc_controller.hpp"

namespace nav2_smc_controller
{

void SMCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_        = parent;
  plugin_name_ = name;
  tf_buffer_   = tf_buffer;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SMCController: failed to lock lifecycle node");
  }

  logger_ = node->get_logger();
  clock_  = node->get_clock();

  // ── robot_type ────────────────────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_type",
    rclcpp::ParameterValue(std::string("diff")));

  std::string robot_type_str;
  node->get_parameter(plugin_name_ + ".robot_type", robot_type_str);

  if (robot_type_str == "omni") {
    robot_type_ = RobotType::OMNI;
  } else if (robot_type_str == "diff") {
    robot_type_ = RobotType::DIFF;
  } else {
    RCLCPP_WARN(logger_,
      "SMCController: unknown robot_type '%s', defaulting to 'diff'",
      robot_type_str.c_str());
    robot_type_ = RobotType::DIFF;
  }

  // ── base_frame_id ─────────────────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".base_frame_id",
    rclcpp::ParameterValue(std::string("base_link")));
  node->get_parameter(plugin_name_ + ".base_frame_id", base_frame_id_);

  // ── Helper: declare + get one double parameter ─────────────────────────
  auto param = [&](const std::string & p, double def) -> double {
    nav2_util::declare_parameter_if_not_declared(
      node, plugin_name_ + "." + p, rclcpp::ParameterValue(def));
    double v;
    node->get_parameter(plugin_name_ + "." + p, v);
    return v;
  };

  // ── Shared parameters ─────────────────────────────────────────────────
  lambda_               = param("lambda",               1.0);
  k_linear_             = param("k_linear",             0.5);
  k_angular_            = param("k_angular",            1.0);
  eta_linear_           = param("eta_linear",           0.2);
  eta_angular_          = param("eta_angular",          0.5);
  boundary_layer_       = param("boundary_layer",       0.1);
  max_linear_velocity_  = param("max_linear_velocity",  0.3);
  max_angular_velocity_ = param("max_angular_velocity", 1.0);
  step_size_            = param("step_size",            0.2);

  base_max_linear_velocity_  = max_linear_velocity_;
  base_max_angular_velocity_ = max_angular_velocity_;

  // ── Omni-only parameters ──────────────────────────────────────────────
  if (robot_type_ == RobotType::OMNI) {
    k_lateral_            = param("k_lateral",            0.5);
    eta_lateral_          = param("eta_lateral",          0.2);
    max_lateral_velocity_ = param("max_lateral_velocity", 0.3);
    base_max_lateral_velocity_ = max_lateral_velocity_;
  }

  // ── Publishers ─────────────────────────────────────────────────────────
  next_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "smc/next_pose", 1);
  sliding_surface_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "smc/sliding_surfaces", 1);

  RCLCPP_INFO(logger_,
    "SMCController configured [%s]: frame=%s lambda=%.2f k_lin=%.2f k_ang=%.2f v_max=%.2f w_max=%.2f step=%.2f",
    robot_type_str.c_str(), base_frame_id_.c_str(),
    lambda_, k_linear_, k_angular_, max_linear_velocity_, max_angular_velocity_, step_size_);
}

void SMCController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up SMCController");
  next_pose_pub_.reset();
  sliding_surface_pub_.reset();
}

void SMCController::activate()
{
  RCLCPP_INFO(logger_, "Activating SMCController");
  next_pose_pub_->on_activate();
  sliding_surface_pub_->on_activate();
  last_cycle_time_ = clock_->now();
  prev_e_y_ = 0.0;
  prev_e_x_ = 0.0;
  is_new_plan_ = true;
}

void SMCController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating SMCController");
  next_pose_pub_->on_deactivate();
  sliding_surface_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped SMCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped zero;
  zero.header.frame_id = base_frame_id_;
  zero.header.stamp    = clock_->now();

  if (global_plan_.poses.empty()) {
    RCLCPP_ERROR(logger_, "SMCController: empty plan — stopping");
    return zero;
  }

  if (!transformPlan(robot_pose.header.frame_id)) {
    RCLCPP_ERROR(logger_, "SMCController: plan transform failed — stopping");
    return zero;
  }

  return (robot_type_ == RobotType::OMNI)
    ? computeOmni(robot_pose)
    : computeDiff(robot_pose);
}

geometry_msgs::msg::TwistStamped SMCController::computeDiff(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_id_;
  cmd.header.stamp    = clock_->now();

  auto next_pose = getNextPose(robot_pose);
  next_pose_pub_->publish(next_pose);

  tf2::Transform robot_tf, next_tf, rel_tf;
  tf2::fromMsg(robot_pose.pose, robot_tf);
  tf2::fromMsg(next_pose.pose,  next_tf);
  rel_tf = robot_tf.inverse() * next_tf;

  const double e_x = rel_tf.getOrigin().getX();
  const double e_y = rel_tf.getOrigin().getY();

  // Prevent derivative kick on new plans
  if (is_new_plan_) {
    prev_e_y_ = e_y;
    prev_e_x_ = e_x;
    is_new_plan_ = false;
  }

  double dt = (clock_->now() - last_cycle_time_).seconds();
  if (dt < 1e-4) { dt = 1e-4; }

  const double e_y_dot = (e_y - prev_e_y_) / dt;
  const double s_v     = e_x;
  const double s_w     = e_y_dot + lambda_ * e_y;

  // u = +eta * s  +  k * sat(s / phi) [Corrected signs for tracking]
  cmd.twist.linear.x  = std::clamp(
    eta_linear_  * s_v + k_linear_  * sat(s_v, boundary_layer_),
    -max_linear_velocity_,  max_linear_velocity_);
  cmd.twist.angular.z = std::clamp(
    eta_angular_ * s_w + k_angular_ * sat(s_w, boundary_layer_),
    -max_angular_velocity_, max_angular_velocity_);
  cmd.twist.linear.y  = 0.0;   

  std_msgs::msg::Float64MultiArray surf;
  surf.data = {s_v, s_w};
  sliding_surface_pub_->publish(surf);

  last_cycle_time_ = clock_->now();
  prev_e_y_ = e_y;
  prev_e_x_ = e_x;

  return cmd;
}

geometry_msgs::msg::TwistStamped SMCController::computeOmni(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_id_;
  cmd.header.stamp    = clock_->now();

  auto next_pose = getNextPose(robot_pose);
  next_pose_pub_->publish(next_pose);

  tf2::Transform robot_tf, next_tf, rel_tf;
  tf2::fromMsg(robot_pose.pose, robot_tf);
  tf2::fromMsg(next_pose.pose,  next_tf);
  rel_tf = robot_tf.inverse() * next_tf;

  const double e_x  = rel_tf.getOrigin().getX();
  const double e_y  = rel_tf.getOrigin().getY();
  const double e_th = wrapAngle(tf2::getYaw(rel_tf.getRotation()));

  if (is_new_plan_) {
    prev_e_y_ = e_y;
    prev_e_x_ = e_x;
    is_new_plan_ = false;
  }

  const double s_v   = e_x;
  const double s_lat = e_y;
  const double s_w   = e_th;

  // u = +eta * s  +  k * sat(s / phi) [Corrected signs for tracking]
  cmd.twist.linear.x  = std::clamp(
    eta_linear_   * s_v   + k_linear_   * sat(s_v,   boundary_layer_),
    -max_linear_velocity_,   max_linear_velocity_);

  cmd.twist.linear.y  = std::clamp(
    eta_lateral_  * s_lat + k_lateral_  * sat(s_lat, boundary_layer_),
    -max_lateral_velocity_,  max_lateral_velocity_);

  cmd.twist.angular.z = std::clamp(
    eta_angular_  * s_w   + k_angular_  * sat(s_w,   boundary_layer_),
    -max_angular_velocity_,  max_angular_velocity_);

  std_msgs::msg::Float64MultiArray surf;
  surf.data = {s_v, s_lat, s_w};
  sliding_surface_pub_->publish(surf);

  last_cycle_time_ = clock_->now();
  prev_e_y_ = e_y;
  prev_e_x_ = e_x;

  return cmd;
}

void SMCController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_     = path;
  prev_e_y_        = 0.0;
  prev_e_x_        = 0.0;
  is_new_plan_     = true; // Arm the flag to prevent derivative kick
  last_cycle_time_ = clock_->now();
}

void SMCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  const double factor = percentage ? (speed_limit / 100.0)
                                   : (base_max_linear_velocity_ > 1e-9
                                        ? speed_limit / base_max_linear_velocity_
                                        : 1.0);

  max_linear_velocity_  = percentage ? base_max_linear_velocity_  * factor : speed_limit;
  max_angular_velocity_ = base_max_angular_velocity_ * factor;

  if (robot_type_ == RobotType::OMNI) {
    max_lateral_velocity_ = base_max_lateral_velocity_ * factor;
  }
}

geometry_msgs::msg::PoseStamped SMCController::getNextPose(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  size_t closest_idx = 0;
  double min_dist    = std::numeric_limits<double>::max();

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    const double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    const double d  = std::hypot(dx, dy);
    if (d < min_dist) { min_dist = d; closest_idx = i; }
  }

  for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
    const double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    const double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    if (std::hypot(dx, dy) >= step_size_) {
      return global_plan_.poses[i];
    }
  }

  return global_plan_.poses.back();
}

bool SMCController::transformPlan(const std::string & frame)
{
  if (global_plan_.header.frame_id == frame) { return true; }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      frame, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_,
      "SMCController: TF '%s'→'%s': %s",
      global_plan_.header.frame_id.c_str(), frame.c_str(), ex.what());
    return false;
  }

  for (auto & pose : global_plan_.poses) {
    tf2::doTransform(pose, pose, transform);
  }
  global_plan_.header.frame_id = frame;
  return true;
}

}  // namespace nav2_smc_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smc_controller::SMCController, nav2_core::Controller)