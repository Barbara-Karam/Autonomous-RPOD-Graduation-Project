
#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/float64_multi_array.hpp" 

using namespace std::chrono_literals;

class MpcSupervisorNode : public rclcpp::Node
{
public:
MpcSupervisorNode() : Node("mpc_supervisor_node")
{
declare_parameter<std::string>("active_mpc", "static");
declare_parameter<int>("switch_zero_cycles", 5);

static_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
  "/mpc/static/wheel_angular_velocities",
  10,
  std::bind(&MpcSupervisorNode::staticCallback, this, std::placeholders::_1));

tumbling_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
  "/mpc/tumbling/wheel_angular_velocities",
  10,
  std::bind(&MpcSupervisorNode::tumblingCallback, this, std::placeholders::_1));

wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
  "/wheel_angular_velocities",
  10);

active_mode_ = get_parameter("active_mpc").as_string();

publish_timer_ = create_wall_timer(
  100ms,
  std::bind(&MpcSupervisorNode::publishSelectedCommand, this));

parameter_callback_handle_ = add_on_set_parameters_callback(
  std::bind(
    &MpcSupervisorNode::parameterCallback,
    this,
    std::placeholders::_1));

RCLCPP_INFO(
  get_logger(),
  "MPC Supervisor started. Active MPC: %s",
  active_mode_.c_str());

}

private:
void staticCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
if (msg->data.size() != 4) {
RCLCPP_WARN(get_logger(), "Static MPC command must contain 4 values.");
return;
}

std::lock_guard<std::mutex> lock(command_mutex_);

for (int i = 0; i < 4; ++i) {
  static_command_[i] = msg->data[i];
}

have_static_command_ = true;

}

void tumblingCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
if (msg->data.size() != 4) {
RCLCPP_WARN(get_logger(), "Tumbling MPC command must contain 4 values.");
return;
}

std::lock_guard<std::mutex> lock(command_mutex_);

for (int i = 0; i < 4; ++i) {
  tumbling_command_[i] = msg->data[i];
}

have_tumbling_command_ = true;

}

rcl_interfaces::msg::SetParametersResult parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
rcl_interfaces::msg::SetParametersResult result;
result.successful = true;

for (const auto & parameter : parameters) {
  if (parameter.get_name() != "active_mpc") {
    continue;
  }

  const std::string requested_mode = parameter.as_string();

  if (requested_mode != "static" && requested_mode != "tumbling") {
    result.successful = false;
    result.reason = "active_mpc must be either 'static' or 'tumbling'.";
    return result;
  }

  if (requested_mode != active_mode_) {
    active_mode_ = requested_mode;

    // Publish zero commands before allowing commands from the new MPC.
    zero_cycles_remaining_ =
      get_parameter("switch_zero_cycles").as_int();

    RCLCPP_WARN(
      get_logger(),
      "Switching active MPC to '%s'. Publishing zero commands for %d cycles.",
      active_mode_.c_str(),
      zero_cycles_remaining_);
  }
}

return result;

}

void publishZeroCommand()
{
std_msgs::msg::Float64MultiArray zero_msg;
zero_msg.data.assign(4, 0.0);
wheel_pub_->publish(zero_msg);
}

void publishSelectedCommand()
{
if (zero_cycles_remaining_ > 0) {
publishZeroCommand();
--zero_cycles_remaining_;
return;
}

std_msgs::msg::Float64MultiArray output;
output.data.resize(4);

std::lock_guard<std::mutex> lock(command_mutex_);

if (active_mode_ == "static") {
  if (!have_static_command_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for commands from static MPC. Publishing zeros.");
    publishZeroCommand();
    return;
  }

  for (int i = 0; i < 4; ++i) {
    output.data[i] = static_command_[i];
  }
} else {
  if (!have_tumbling_command_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for commands from tumbling MPC. Publishing zeros.");
    publishZeroCommand();
    return;
  }

  for (int i = 0; i < 4; ++i) {
    output.data[i] = tumbling_command_[i];
  }
}

wheel_pub_->publish(output);

}

rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr static_sub_;
rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tumbling_sub_;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
rclcpp::TimerBase::SharedPtr publish_timer_;

OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

std::mutex command_mutex_;

std::array<double, 4> static_command_{0.0, 0.0, 0.0, 0.0};
std::array<double, 4> tumbling_command_{0.0, 0.0, 0.0, 0.0};

bool have_static_command_ = false;
bool have_tumbling_command_ = false;

std::string active_mode_ = "static";
int zero_cycles_remaining_ = 0;
};

int main(int argc, char ** argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<MpcSupervisorNode>());
rclcpp::shutdown();
return 0;
}
