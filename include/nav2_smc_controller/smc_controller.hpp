#ifndef SMC_CONTROLLER_HPP
#define SMC_CONTROLLER_HPP

#include <string>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_smc_controller
{

class SMCController : public nav2_core::Controller
{
public:
  SMCController() = default;
  ~SMCController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  // ── Utility functions (made public for testing) ──────────────────────
  /// Boundary-layer saturation – suppresses chattering near s = 0
  static double sat(double s, double phi)
  {
    if (phi < 1e-9) { return (s >= 0.0) ? 1.0 : -1.0; }
    return std::clamp(s / phi, -1.0, 1.0);
  }

  /// Wrap angle to [-pi, pi]
  static double wrapAngle(double a)
  {
    while (a >  M_PI) { a -= 2.0 * M_PI; }
    while (a < -M_PI) { a += 2.0 * M_PI; }
    return a;
  }

protected:
  // ── Control branches ─────────────────────────────────────────────────
  geometry_msgs::msg::TwistStamped computeDiff(
    const geometry_msgs::msg::PoseStamped & robot_pose);

  geometry_msgs::msg::TwistStamped computeOmni(
    const geometry_msgs::msg::PoseStamped & robot_pose);

  // ── Helpers ──────────────────────────────────────────────────────────
  geometry_msgs::msg::PoseStamped getNextPose(
    const geometry_msgs::msg::PoseStamped & robot_pose);

  bool transformPlan(const std::string & frame);

  // ── ROS handles ──────────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr  clock_;
  rclcpp::Logger            logger_ {rclcpp::get_logger("SMCController")};
  std::string               plugin_name_;

  std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseStamped>>     next_pose_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
    std_msgs::msg::Float64MultiArray>>    sliding_surface_pub_;

  // ── Robot type and frames ────────────────────────────────────────────
  enum class RobotType { DIFF, OMNI };
  RobotType robot_type_ {RobotType::DIFF};
  std::string base_frame_id_ {"base_link"};

  // ── Shared SMC parameters ────────────────────────────────────────────
  double lambda_          {1.0};   
  double k_linear_        {0.5};
  double k_angular_       {1.0};
  double eta_linear_      {0.2};
  double eta_angular_     {0.5};
  double boundary_layer_  {0.1};   
  double max_linear_velocity_  {0.3};
  double max_angular_velocity_ {1.0};
  double step_size_       {0.2};

  // ── Omni-only parameters ─────────────────────────────────────────────
  double k_lateral_           {0.5};
  double eta_lateral_         {0.2};
  double max_lateral_velocity_{0.3};
  double base_max_lateral_velocity_ {0.3};

  // ── Speed-limit originals ────────────────────────────────────────────
  double base_max_linear_velocity_  {0.3};
  double base_max_angular_velocity_ {1.0};

  // ── Controller state ─────────────────────────────────────────────────
  rclcpp::Time last_cycle_time_;
  double prev_e_y_  {0.0};   
  double prev_e_x_  {0.0};
  bool   is_new_plan_ {true}; ///< Prevents derivative kick on first cycle

  nav_msgs::msg::Path global_plan_;
};

}  // namespace nav2_smc_controller

#endif  // SMC_CONTROLLER_HPP
