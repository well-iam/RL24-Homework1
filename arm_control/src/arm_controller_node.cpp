#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class ArmControllerNode : public rclcpp::Node
{
public:
    ArmControllerNode() : Node("arm_controller_node")
    {
        // Subscriber to joint_states topic
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ArmControllerNode::jointStateCallback, this, std::placeholders::_1));

        // Publisher to position_controller/command topic
        position_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&ArmControllerNode::timer_callback, this));
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Print current joint positions
        RCLCPP_INFO(this->get_logger(), "Current Joint Positions:");
        for (size_t i = 0; i < msg->position.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Joint %zu: %f", i, msg->position[i]);
        }
    }

    void timer_callback()
    {
      // Publish a command to the joints
      std_msgs::msg::Float64MultiArray command_msg;
      command_msg.data = {1.0, 0.5, 0.0, 0.0}; // Desired positions for the joints
      position_command_publisher_->publish(command_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmControllerNode>());
    rclcpp::shutdown();
    return 0;
}
