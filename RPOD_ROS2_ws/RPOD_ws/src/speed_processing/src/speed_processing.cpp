#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <array>
#include <cmath>

using std::placeholders::_1;

//================ Parameters ================//
constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI); // Angular velocity (rad/s) -> RPM
//============================================//

class SpeedProcessingNode : public rclcpp::Node
{
public:
    SpeedProcessingNode() : Node("speed_processing_node")
    {
        // Desired Angular Velocity Subscriber (rad/s)
        desired_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/wheel_angular_velocities", 10,
            std::bind(&SpeedProcessingNode::desiredCallback, this, _1));

        // UART Node Publisher (converted RPM values)
        uart_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/adjusted_rpms", 10);

        RCLCPP_INFO(this->get_logger(), "Speed Processing Node Started");
    }

private:
    // Subscriber
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr desired_sub_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr uart_pub_;

    //------------------------------------------------//
    void desiredCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "Angular velocity array must contain 4 values.");
            return;
        }

        std_msgs::msg::Float64MultiArray rpm_msg;
        rpm_msg.data.resize(4);

        // Convert angular velocities (rad/s) to RPM
        for (int i = 0; i < 4; i++) {
            rpm_msg.data[i] = msg->data[i] * RAD_S_TO_RPM;
        }

        // Publish converted RPM values to UART node
        uart_pub_->publish(rpm_msg);
    }
};  // End of SpeedProcessingNode class
//------------------------------------------------//

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeedProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}