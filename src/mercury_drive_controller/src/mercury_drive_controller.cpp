// ════════════════════════════════════════════════════════════════════
//  MercuryDriveController — implementation
//  Custom 4-wheel skid-steer controller for Mercury UGV (ROS2 Jazzy)
// ════════════════════════════════════════════════════════════════════

#include "mercury_drive_controller/mercury_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace mercury_drive_controller
{

// ── Constructor ───────────────────────────────────────────────────────
MercuryDriveController::MercuryDriveController()
: controller_interface::ControllerInterface() {}

// ── Interface configuration ───────────────────────────────────────────

controller_interface::InterfaceConfiguration
MercuryDriveController::command_interface_configuration() const
{
  // We write velocity commands to all four wheel joints.
  std::vector<std::string> ifaces;
  for (const auto & j : LEFT_WHEEL_JOINTS)
    ifaces.push_back(std::string(j) + "/" + hardware_interface::HW_IF_VELOCITY);
  for (const auto & j : RIGHT_WHEEL_JOINTS)
    ifaces.push_back(std::string(j) + "/" + hardware_interface::HW_IF_VELOCITY);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, ifaces};
}

controller_interface::InterfaceConfiguration
MercuryDriveController::state_interface_configuration() const
{
  // We read velocity from all four wheels (odometry).
  // We read position from both suspension joints (geometry awareness).
  std::vector<std::string> ifaces;
  for (const auto & j : LEFT_WHEEL_JOINTS)
    ifaces.push_back(std::string(j) + "/" + hardware_interface::HW_IF_VELOCITY);
  for (const auto & j : RIGHT_WHEEL_JOINTS)
    ifaces.push_back(std::string(j) + "/" + hardware_interface::HW_IF_VELOCITY);
  for (const auto & j : SUSPENSION_JOINTS)
    ifaces.push_back(std::string(j) + "/" + hardware_interface::HW_IF_POSITION);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, ifaces};
}

// ── Lifecycle callbacks ───────────────────────────────────────────────

controller_interface::CallbackReturn MercuryDriveController::on_init()
{
  // Declare all ROS parameters with defaults.
  // Values can be overridden in controllers.yaml.
  try {
    auto_declare<double>("wheel_radius",    p_wheel_radius_);
    auto_declare<double>("wheel_track",     p_wheel_track_);
    auto_declare<double>("cmd_vel_timeout", p_cmd_timeout_);
    auto_declare<std::string>("base_frame_id", p_base_frame_);
    auto_declare<std::string>("odom_frame_id", p_odom_frame_);
    auto_declare<bool>  ("enable_odom_tf",  p_pub_tf_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "[MercuryDrive] on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MercuryDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // ── Read parameters ───────────────────────────────────────────────
  p_wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  p_wheel_track_  = get_node()->get_parameter("wheel_track").as_double();
  p_cmd_timeout_  = get_node()->get_parameter("cmd_vel_timeout").as_double();
  p_base_frame_   = get_node()->get_parameter("base_frame_id").as_string();
  p_odom_frame_   = get_node()->get_parameter("odom_frame_id").as_string();
  p_pub_tf_       = get_node()->get_parameter("enable_odom_tf").as_bool();

  if (p_wheel_radius_ <= 0.0 || p_wheel_track_ <= 0.0) {
    RCLCPP_ERROR(logger,
      "[MercuryDrive] wheel_radius (%.4f) and wheel_track (%.4f) must be positive.",
      p_wheel_radius_, p_wheel_track_);
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger,
    "[MercuryDrive] Configured — radius=%.3fm  track=%.3fm  timeout=%.2fs",
    p_wheel_radius_, p_wheel_track_, p_cmd_timeout_);

  // ── Command subscriber (non-RT, fills RealtimeBox) ────────────────
  cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
      if (!subscriber_active_) return;
      cmd_box_.set(*msg);
    });

  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom", rclcpp::SystemDefaultsQoS());
  rt_odom_pub_ = std::make_shared<
    realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);

  // ── Initialise odometry message ───────────────────────────────────
  rt_odom_pub_->msg_.header.frame_id = p_odom_frame_;
  rt_odom_pub_->msg_.child_frame_id  = p_base_frame_;
  rt_odom_pub_->msg_.pose.pose.orientation.w = 1.0;
  // Diagonal covariance — tunable later
  rt_odom_pub_->msg_.pose.covariance[0]  = 1e-3;
  rt_odom_pub_->msg_.pose.covariance[7]  = 1e-3;
  rt_odom_pub_->msg_.pose.covariance[35] = 1e-2;
  rt_odom_pub_->msg_.twist.covariance[0]  = 1e-3;
  rt_odom_pub_->msg_.twist.covariance[7]  = 1e-3;
  rt_odom_pub_->msg_.twist.covariance[35] = 1e-2;

  // ── TF publisher ──────────────────────────────────────────────────
  if (p_pub_tf_) {
    tf_pub_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
    rt_tf_pub_ = std::make_shared<
      realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_pub_);
    rt_tf_pub_->msg_.transforms.resize(1);
    rt_tf_pub_->msg_.transforms[0].header.frame_id = p_odom_frame_;
    rt_tf_pub_->msg_.transforms[0].child_frame_id  = p_base_frame_;
  }

  // Zero the cmd_vel box
  geometry_msgs::msg::TwistStamped zero;
  zero.header.stamp = get_node()->now();
  cmd_box_.set(zero);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MercuryDriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Map state/command interface loaned references into our handle arrays.
  if (!register_wheel_handles())      return controller_interface::CallbackReturn::ERROR;
  if (!register_suspension_handles()) return controller_interface::CallbackReturn::ERROR;

  subscriber_active_ = true;
  odom_inited_       = false;

  RCLCPP_INFO(get_node()->get_logger(),
    "[MercuryDrive] Activated — wheels and suspension handles acquired.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MercuryDriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_active_ = false;
  halt();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MercuryDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MercuryDriveController::on_error(
  const rclcpp_lifecycle::State &)
{
  reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

// ── Main control loop (called at controller_manager update rate) ──────

controller_interface::return_type MercuryDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  const double dt = period.seconds();

  // ── 1. Get latest command from the subscriber thread ─────────────
  geometry_msgs::msg::TwistStamped cmd;
  if (auto opt = cmd_box_.try_get(); opt.has_value()) {
    cmd = *opt;
  } else {
    cmd = last_cmd_;
  }

  // ── 2. Command timeout — brake if stale ──────────────────────────
  const double age = (time - cmd.header.stamp).seconds();
  if (p_cmd_timeout_ > 0.0 && age > p_cmd_timeout_) {
    cmd.twist.linear.x  = 0.0;
    cmd.twist.angular.z = 0.0;
  }
  last_cmd_ = cmd;

  const double linear  = cmd.twist.linear.x;
  const double angular = cmd.twist.angular.z;

  // ── 3. Skid-steer kinematics → wheel angular velocities ──────────
  //   ω_left  = (v  -  ω × track/2) / r
  //   ω_right = (v  +  ω × track/2) / r
  const double half_track  = p_wheel_track_ / 2.0;

  // URDF swap: left_wheels_ are physically on the right side, right_wheels_ are physically on the left side.
  // Wheel joint axes: axis is 0 1 0 (pointing left).
  // To drive forward, both sides require positive velocity. CCW rotation requires positive on right and negative on left.
  const double omega_left_wheels  =  (linear + angular * half_track) / p_wheel_radius_;
  const double omega_right_wheels =  (linear - angular * half_track) / p_wheel_radius_;

  RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 500,
    "[MercuryDriveDebug] linear: %.3f, angular: %.3f, age: %.3f, time: %.3f, cmd_L: %.3f, cmd_R: %.3f",
    linear, angular, age, time.seconds(), omega_left_wheels, omega_right_wheels);

  // ── 4. Command all wheels on each side ───────────────────────────
  for (auto & wh : left_wheels_) {
    if (wh.velocity_cmd && !wh.velocity_cmd->set_value(omega_left_wheels)) {
      RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
        "[MercuryDrive] Failed to set left wheel velocity.");
    }
  }
  for (auto & wh : right_wheels_) {
    if (wh.velocity_cmd && !wh.velocity_cmd->set_value(omega_right_wheels)) {
      RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
        "[MercuryDrive] Failed to set right wheel velocity.");
    }
  }

  // ── 5. Read actual wheel velocities for odometry ─────────────────
  //   Average front + rear on each side for a more accurate estimate.
  auto read_vel = [&](WheelHandle & wh) -> double {
    if (!wh.velocity_state) return 0.0;
    auto opt = wh.velocity_state->get_optional();
    return opt.has_value() ? *opt : 0.0;
  };

  const double right_vel_mean = (read_vel(left_wheels_[0])  + read_vel(left_wheels_[1]))  / 2.0;
  const double left_vel_mean  = (read_vel(right_wheels_[0]) + read_vel(right_wheels_[1])) / 2.0;

  // Convert to linear contact velocity (positive velocity is forward for both sides)
  const double v_left  =  left_vel_mean  * p_wheel_radius_;
  const double v_right =  right_vel_mean * p_wheel_radius_;

  // ── 6. Read suspension angles (for logging / future geo-correction)
  auto read_pos = [&](SuspensionHandle & sh) -> double {
    if (!sh.position_state) return 0.0;
    auto opt = sh.position_state->get_optional();
    return opt.has_value() ? *opt : 0.0;
  };
  const double theta_left  = read_pos(suspension_[0]);  // left arm angle [rad]
  const double theta_right = read_pos(suspension_[1]);  // right arm angle (= -theta_left via mimic)

  // Log suspension seesaw angle for diagnostics (throttled)
  RCLCPP_DEBUG_THROTTLE(logger, *get_node()->get_clock(), 500,
    "[MercuryDrive] Suspension — left: %.3f rad  right: %.3f rad  (sum should ≈ 0)",
    theta_left, theta_right);

  // ── 7. Integrate odometry ─────────────────────────────────────────
  if (!odom_inited_) {
    // Skip first cycle — no valid dt yet
    odom_inited_ = true;
  } else if (std::isfinite(v_left) && std::isfinite(v_right)) {
    integrate_odometry(v_left, v_right, dt);
  }

  // ── 8. Publish /odom and /tf ──────────────────────────────────────
  publish_odometry_and_tf(time);

  return controller_interface::return_type::OK;
}

// ── Odometry integration (Euler method) ──────────────────────────────

void MercuryDriveController::integrate_odometry(
  double v_left, double v_right, double dt)
{
  const double v_robot_x = (v_left + v_right) / 2.0;
  const double w_robot   = (v_right - v_left) / p_wheel_track_;

  // Center of the wheels is offset from base_link origin by d_x = 0.3976 m.
  // Since the robot rotates around the center of the wheels, rotation with angular speed w_robot
  // induces a lateral linear velocity of the base_link origin: v_robot_y = -w_robot * d_x.
  const double d_x = 0.3976;
  const double v_robot_y = -w_robot * d_x;

  // Integrate in world frame
  const double delta_x   = (v_robot_x * std::cos(odom_theta_) - v_robot_y * std::sin(odom_theta_)) * dt;
  const double delta_y   = (v_robot_x * std::sin(odom_theta_) + v_robot_y * std::cos(odom_theta_)) * dt;
  const double delta_th  = w_robot * dt;

  odom_x_     += delta_x;
  odom_y_     += delta_y;
  odom_theta_ += delta_th;

  // Keep heading in [-π, π]
  while (odom_theta_ >  M_PI) odom_theta_ -= 2.0 * M_PI;
  while (odom_theta_ < -M_PI) odom_theta_ += 2.0 * M_PI;
}

// ── Publish odometry and TF ───────────────────────────────────────────

void MercuryDriveController::publish_odometry_and_tf(const rclcpp::Time & time)
{
  // Build quaternion from heading angle
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_theta_);

  // ── /odom ─────────────────────────────────────────────────────────
  if (rt_odom_pub_ && rt_odom_pub_->trylock()) {
    auto & msg = rt_odom_pub_->msg_;
    msg.header.stamp                 = time;
    msg.pose.pose.position.x         = odom_x_;
    msg.pose.pose.position.y         = odom_y_;
    msg.pose.pose.position.z         = 0.0;
    msg.pose.pose.orientation.x      = q.x();
    msg.pose.pose.orientation.y      = q.y();
    msg.pose.pose.orientation.z      = q.z();
    msg.pose.pose.orientation.w      = q.w();
    // Reconstruct velocities from state
    // (use the last commanded values as a proxy, including lateral velocity from rotation)
    msg.twist.twist.linear.x  = last_cmd_.twist.linear.x;
    msg.twist.twist.linear.y  = -last_cmd_.twist.angular.z * 0.3976;
    msg.twist.twist.angular.z = last_cmd_.twist.angular.z;
    rt_odom_pub_->unlockAndPublish();
  }

  // ── /tf ───────────────────────────────────────────────────────────
  if (p_pub_tf_ && rt_tf_pub_ && rt_tf_pub_->trylock()) {
    auto & t = rt_tf_pub_->msg_.transforms[0];
    t.header.stamp               = time;
    t.transform.translation.x    = odom_x_;
    t.transform.translation.y    = odom_y_;
    t.transform.translation.z    = 0.0;
    t.transform.rotation.x       = q.x();
    t.transform.rotation.y       = q.y();
    t.transform.rotation.z       = q.z();
    t.transform.rotation.w       = q.w();
    rt_tf_pub_->unlockAndPublish();
  }
}

// ── Register hardware interface handles ───────────────────────────────

bool MercuryDriveController::register_wheel_handles()
{
  auto logger = get_node()->get_logger();

  // Helper: find a state interface by prefix+name
  auto find_state = [&](const std::string & joint, const std::string & iface)
    -> hardware_interface::LoanedStateInterface *
  {
    auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & si) {
        return si.get_prefix_name() == joint && si.get_interface_name() == iface;
      });
    if (it == state_interfaces_.end()) {
      RCLCPP_ERROR(logger,
        "[MercuryDrive] State interface '%s/%s' not found.",
        joint.c_str(), iface.c_str());
      return nullptr;
    }
    return &(*it);
  };

  // Helper: find a command interface by prefix+name
  auto find_cmd = [&](const std::string & joint, const std::string & iface)
    -> hardware_interface::LoanedCommandInterface *
  {
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
      [&](const auto & ci) {
        return ci.get_prefix_name() == joint && ci.get_interface_name() == iface;
      });
    if (it == command_interfaces_.end()) {
      RCLCPP_ERROR(logger,
        "[MercuryDrive] Command interface '%s/%s' not found.",
        joint.c_str(), iface.c_str());
      return nullptr;
    }
    return &(*it);
  };

  // Left wheels
  for (std::size_t i = 0; i < LEFT_WHEEL_JOINTS.size(); ++i) {
    const std::string jname(LEFT_WHEEL_JOINTS[i]);
    auto * vs = find_state(jname, hardware_interface::HW_IF_VELOCITY);
    auto * vc = find_cmd  (jname, hardware_interface::HW_IF_VELOCITY);
    if (!vs || !vc) return false;
    left_wheels_[i] = WheelHandle{vs, vc};
  }

  // Right wheels
  for (std::size_t i = 0; i < RIGHT_WHEEL_JOINTS.size(); ++i) {
    const std::string jname(RIGHT_WHEEL_JOINTS[i]);
    auto * vs = find_state(jname, hardware_interface::HW_IF_VELOCITY);
    auto * vc = find_cmd  (jname, hardware_interface::HW_IF_VELOCITY);
    if (!vs || !vc) return false;
    right_wheels_[i] = WheelHandle{vs, vc};
  }

  return true;
}

bool MercuryDriveController::register_suspension_handles()
{
  auto logger = get_node()->get_logger();

  for (std::size_t i = 0; i < SUSPENSION_JOINTS.size(); ++i) {
    const std::string jname(SUSPENSION_JOINTS[i]);
    auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & si) {
        return si.get_prefix_name() == jname &&
               si.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (it == state_interfaces_.end()) {
      RCLCPP_WARN(logger,
        "[MercuryDrive] Suspension joint '%s/position' not found. "
        "Suspension geometry correction disabled.", jname.c_str());
      // Non-fatal: suspension reading is a bonus, not required for driving.
      // Use a dummy placeholder so the handle is still valid.
      // The get_optional() call will return nullopt for a missing interface.
      // We skip registering so index access won't crash.
      continue;
    }
    suspension_[i] = SuspensionHandle{&*it};
  }

  return true;   // always succeed — suspension reading is advisory only
}

// ── Safety helpers ────────────────────────────────────────────────────

void MercuryDriveController::halt()
{
  for (auto & wh : left_wheels_) {
    if (wh.velocity_cmd) {
      (void)wh.velocity_cmd->set_value(0.0);
    }
  }
  for (auto & wh : right_wheels_) {
    if (wh.velocity_cmd) {
      (void)wh.velocity_cmd->set_value(0.0);
    }
  }
}

bool MercuryDriveController::reset()
{
  subscriber_active_ = false;
  odom_inited_       = false;
  odom_x_            = 0.0;
  odom_y_            = 0.0;
  odom_theta_        = 0.0;
  cmd_sub_.reset();
  geometry_msgs::msg::TwistStamped zero;
  cmd_box_.set(zero);
  return true;
}

}  // namespace mercury_drive_controller

// ── Plugin export ─────────────────────────────────────────────────────
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mercury_drive_controller::MercuryDriveController,
  controller_interface::ControllerInterface)
