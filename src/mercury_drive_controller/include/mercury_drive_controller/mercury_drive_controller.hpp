#pragma once

// ════════════════════════════════════════════════════════════════════
//  MercuryDriveController
//  Custom 4-wheel skid-steer drive controller for Mercury UGV.
//
//  ROBOT ARCHITECTURE THIS CONTROLLER IS BUILT FOR:
//  ─────────────────────────────────────────────────
//  base_link
//    ├── left_suspension_joint  [revolute, Y-axis, passive, ±60°]
//    │     └── left_suspension_link
//    │           ├── front_left_wheel_joint  [continuous]
//    │           └── rear_left_wheel_joint   [continuous]
//    └── right_suspension_joint [revolute, Y-axis, mimic of left × -1]
//          └── right_suspension_link
//                ├── front_right_wheel_joint [continuous]
//                └── rear_right_wheel_joint  [continuous]
//
//  WHY NOT standard diff_drive_controller:
//  ─────────────────────────────────────────
//  1. Wheels live on suspension arms that rotate passively.
//     The effective lateral wheel track changes as suspension tilts.
//  2. We need 4-wheel odometry (average front+rear per side) rather
//     than assuming 2 virtual wheels.
//  3. We read suspension joint positions for geometry-aware odometry
//     and future terrain-adaptive control.
//  4. The team needs a codebase they can read, own and modify.
//
//  KINEMATICS (skid-steer):
//  ─────────────────────────────────────────
//  Given cmd: linear v [m/s], angular ω [rad/s]
//    ω_left  = (v - ω × track/2) / wheel_radius
//    ω_right = (v + ω × track/2) / wheel_radius
//  Both front and rear wheels on the same side get identical commands.
//
//  ODOMETRY:
//  ─────────────────────────────────────────
//  v_left  = mean(ω_FL, ω_RL) × wheel_radius
//  v_right = mean(ω_FR, ω_RR) × wheel_radius
//  v_robot = (v_left + v_right) / 2
//  ω_robot = (v_right - v_left) / effective_track
//  x, y, θ integrated with Euler method each control cycle.
// ════════════════════════════════════════════════════════════════════

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace mercury_drive_controller
{

// ── Thin handle wrapping a loaned interface pointer ──────────────────
struct WheelHandle
{
  hardware_interface::LoanedStateInterface*   velocity_state = nullptr;
  hardware_interface::LoanedCommandInterface* velocity_cmd = nullptr;
};

struct SuspensionHandle
{
  hardware_interface::LoanedStateInterface* position_state = nullptr;
};

// ── Controller class ─────────────────────────────────────────────────
class MercuryDriveController : public controller_interface::ControllerInterface
{
public:
  MercuryDriveController();

  // ── ros2_control mandatory overrides ──────────────────────────────
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & state) override;

private:
  // ── Internal helpers ───────────────────────────────────────────────
  bool reset();
  void halt();
  void integrate_odometry(double v_left, double v_right, double dt);
  void publish_odometry_and_tf(const rclcpp::Time & time);

  bool register_wheel_handles();
  bool register_suspension_handles();

  // ── Robot-specific joint names (must match URDF) ───────────────────
  // These are the only two places in the whole codebase that hard-code
  // joint names — change them here if the URDF names ever change.
  static constexpr std::array<const char *, 2> LEFT_WHEEL_JOINTS  {
    "front_left_wheel_joint", "rear_left_wheel_joint"
  };
  static constexpr std::array<const char *, 2> RIGHT_WHEEL_JOINTS {
    "front_right_wheel_joint", "rear_right_wheel_joint"
  };
  static constexpr std::array<const char *, 2> SUSPENSION_JOINTS  {
    "left_suspension_joint", "right_suspension_joint"
  };

  // ── Parameters (declared in on_init, read in on_configure) ─────────
  double p_wheel_radius_   {0.155};   // m  — from wheel.dae mesh analysis
  double p_wheel_track_    {0.664};   // m  — 2 × (0.266 + 0.066) pivot+offset
  double p_cmd_timeout_    {0.5};     // s  — stop if no cmd_vel for this long
  std::string p_base_frame_{"base_link"};
  std::string p_odom_frame_{"odom"};
  bool        p_pub_tf_    {true};

  // ── Handles to hardware interfaces (filled on_activate) ────────────
  std::array<WheelHandle,      2> left_wheels_;
  std::array<WheelHandle,      2> right_wheels_;
  std::array<SuspensionHandle, 2> suspension_;

  // ── Command subscription (non-RT thread writes, RT thread reads) ───
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::TwistStamped>     cmd_box_;
  geometry_msgs::msg::TwistStamped last_cmd_;   // last accepted command
  bool subscriber_active_{false};

  // ── Odometry state ──────────────────────────────────────────────────
  double odom_x_      {0.0};
  double odom_y_      {0.0};
  double odom_theta_  {0.0};
  bool   odom_inited_ {false};

  // ── Realtime-safe publishers ────────────────────────────────────────
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<
    nav_msgs::msg::Odometry>>  rt_odom_pub_;
  nav_msgs::msg::Odometry odom_msg_;

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<
    tf2_msgs::msg::TFMessage>> rt_tf_pub_;
  tf2_msgs::msg::TFMessage tf_msg_;
};

}  // namespace mercury_drive_controller
