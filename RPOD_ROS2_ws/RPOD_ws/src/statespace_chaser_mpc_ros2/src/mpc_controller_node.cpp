#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

extern "C" {
#include "statespace_chaser_mpc_ros2/statespace_chaser_test0x2810x29.h"
}

using namespace std::chrono_literals;

namespace state_idx {
constexpr int X = 0;
constexpr int Y = 1;
constexpr int YAW = 2;
}

class MpcControllerNode : public rclcpp::Node
{
public:
  MpcControllerNode() : Node("statespace_chaser_mpc_controller")
  {
    statespace_chaser_test0x2810x29_initialize();

    this->declare_parameter<double>("approach_hold_radius", rtP_ApproachHoldRadius);
    this->declare_parameter<double>("approach_final_radius", rtP_ApproachFinalRadius);
    this->declare_parameter<double>(
      "approach_ramp_start_time", rtP_ApproachRampStartTime);
    this->declare_parameter<double>(
      "approach_ramp_end_time", rtP_ApproachRampEndTime);
    this->declare_parameter<double>("dock_center_x", rtP_DockCenterX);
    this->declare_parameter<double>("dock_center_y", rtP_DockCenterY);
    this->declare_parameter<double>("dock_arm_offset", rtP_DockArmOffset);

    this->declare_parameter<bool>("invert_cv_relative_pose", true);

    this->declare_parameter<double>("motor_max_angular_velocity", 3.665);
    this->declare_parameter<double>("motor_min_angular_velocity", -3.665);

    const auto cv_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                          .reliable()
                          .transient_local();

    cv_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "cv/relative_pose",
      cv_qos,
      std::bind(
        &MpcControllerNode::cvPoseCallback,
        this,
        std::placeholders::_1));

    // Receives true when the arm limit switch confirms physical contact.
    arm_contact_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
      "arm/contact",
      10,
      std::bind(
        &MpcControllerNode::armContactCallback,
        this,
        std::placeholders::_1));

    motor_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/wheel_angular_velocities",
      10);

    const auto step_period =
      std::chrono::duration<double>(rtM->Timing.stepSize0);

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(step_period),
      std::bind(&MpcControllerNode::stepController, this));

    RCLCPP_INFO(
      get_logger(),
      "Tumbling MPC started at %.1f Hz. Motors will stop only when "
      "/arm/contact becomes true.",
      1.0 / rtM->Timing.stepSize0);
  }

private:
  static void extractXYYaw(
    const geometry_msgs::msg::PoseStamped & msg,
    double & x,
    double & y,
    double & yaw)
  {
    tf2::Quaternion q(
      msg.pose.orientation.x,
      msg.pose.orientation.y,
      msg.pose.orientation.z,
      msg.pose.orientation.w);

    double roll;
    double pitch;

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    x = msg.pose.position.x;
    y = msg.pose.position.y;
  }

  void cvPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double x;
    double y;
    double yaw;

    extractXYYaw(*msg, x, y, yaw);

    std::lock_guard<std::mutex> lock(pose_mutex_);

    cv_x_ = x;
    cv_y_ = y;
    cv_yaw_ = yaw;
    have_cv_pose_ = true;
  }

  void armContactCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !docking_complete_) {
      docking_complete_ = true;

      RCLCPP_WARN(
        get_logger(),
        "ARM LIMIT SWITCH CONTACT RECEIVED. "
        "Stopping MPC and publishing zero motor commands.");

      publishZeroMotorCommand();

      if (timer_) {
        timer_->cancel();
      }
    }
  }

  static void invertRelativePose(
    double x,
    double y,
    double yaw,
    double & x_inv,
    double & y_inv,
    double & yaw_inv)
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    x_inv = -(x * c + y * s);
    y_inv = -(-x * s + y * c);
    yaw_inv = -yaw;
  }

  void publishZeroMotorCommand()
  {
    std_msgs::msg::Float64MultiArray zero_out;
    zero_out.data.assign(4, 0.0);

    motor_pub_->publish(zero_out);
  }

  void stepController()
  {
    // Once the limit switch confirms contact, this node never sends
    // another MPC motor command.
    if (docking_complete_) {
      publishZeroMotorCommand();
      return;
    }

    double x;
    double y;
    double yaw;

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);

      if (!have_cv_pose_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "No CV relative pose received yet -- holding controller.");

        return;
      }

      x = cv_x_;
      y = cv_y_;
      yaw = cv_yaw_;
    }

    const bool invert =
      this->get_parameter("invert_cv_relative_pose").as_bool();

    double state_x = x;
    double state_y = y;
    double state_yaw = yaw;

    if (invert) {
      invertRelativePose(
        x,
        y,
        yaw,
        state_x,
        state_y,
        state_yaw);
    }

    rtX.Integrator_CSTATE[state_idx::X] = state_x;
    rtX.Integrator_CSTATE[state_idx::Y] = state_y;
    rtX.Integrator_CSTATE[state_idx::YAW] = state_yaw;

    // Raw target yaw from CV is used as the current tumble angle.
    rtP_TargetTumbleAngle = yaw;

    rtP_ApproachHoldRadius =
      this->get_parameter("approach_hold_radius").as_double();

    rtP_ApproachFinalRadius =
      this->get_parameter("approach_final_radius").as_double();

    rtP_ApproachRampStartTime =
      this->get_parameter("approach_ramp_start_time").as_double();

    rtP_ApproachRampEndTime =
      this->get_parameter("approach_ramp_end_time").as_double();

    rtP_DockCenterX =
      this->get_parameter("dock_center_x").as_double();

    rtP_DockCenterY =
      this->get_parameter("dock_center_y").as_double();

    rtP_DockArmOffset =
      this->get_parameter("dock_arm_offset").as_double();

    statespace_chaser_test0x2810x29_step();

    if (rtmGetErrorStatus(rtM) != nullptr) {
      RCLCPP_ERROR(
        get_logger(),
        "Controller error: %s",
        rtmGetErrorStatus(rtM));

      publishZeroMotorCommand();
      return;
    }

    std_msgs::msg::Float64MultiArray out;
    out.data.resize(4);

    const double v_min =
      this->get_parameter("motor_min_angular_velocity").as_double();

    const double v_max =
      this->get_parameter("motor_max_angular_velocity").as_double();

    bool saturated = false;
    bool invalid = false;

    for (int i = 0; i < 4; ++i) {
      double v = rtDW.u[i];

      if (!std::isfinite(v)) {
        invalid = true;
        v = 0.0;
      } else if (v > v_max) {
        saturated = true;
        v = v_max;
      } else if (v < v_min) {
        saturated = true;
        v = v_min;
      }

      out.data[i] = v;
    }

    if (invalid) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid motor command from MPC. Replaced invalid values with zero.");
    }

    if (saturated) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Motor command saturated to [%.2f, %.2f] rad/s.",
        v_min,
        v_max);
    }

    motor_pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cv_pose_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_contact_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex pose_mutex_;

  bool have_cv_pose_ = false;

  double cv_x_ = 0.0;
  double cv_y_ = 0.0;
  double cv_yaw_ = 0.0;

  // Becomes true only after arm/contact publishes true.
  bool docking_complete_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MpcControllerNode>());

  rclcpp::shutdown();

  return 0;
}
