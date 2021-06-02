#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/joy_stick_enabled.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Intermediate : public rclcpp::Node
{
  public:
    Intermediate()
    : Node("intermediate_node")
    {
      gear_publisher_ = create_publisher<std_msgs::msg::UInt8>("/joystick/gear_cmd", 20);
      data_publisher_ = create_publisher<std_msgs::msg::Float32>("/joystick/accelerator_cmd", 20);
      steer_publisher_ = create_publisher<std_msgs::msg::Float32>("/joystick/steering_cmd", 20);
      brake_publisher_ = create_publisher<std_msgs::msg::Float32>("/joystick/brake_cmd", 20);
      gear_subscription_ = create_subscription<autoware_auto_msgs::msg::VehicleStateCommand>(
      "vehicle/state_command", 10, std::bind(&Intermediate::gear_callback, this, _1));
      data_subscription_ = create_subscription<autoware_auto_msgs::msg::RawControlCommand>(
      "joystick/raw_command", 10, std::bind(&Intermediate::data_callback, this, _1));
    }
// cm
  private:

    void gear_callback(const autoware_auto_msgs::msg::VehicleStateCommand::SharedPtr msg) const
    {
      auto variable = std_msgs::msg::UInt8();
      variable.data = msg->gear;
      gear_publisher_->publish(variable);
      // std::cout<<msg->gear<<std::endl;
      // Write the message that was received on the console window
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void data_callback(const autoware_auto_msgs::msg::RawControlCommand::SharedPtr msg) const
    {
      auto variable = std_msgs::msg::Float32();
      auto brake_variable = std_msgs::msg::Float32();
      auto need = std_msgs::msg::Float32();
      // decltype(variable.data) msg->throttle;
      variable.data = float(msg->throttle);
      need.data = float((msg->front_steer));
      brake_variable.data = float(msg->brake);
      data_publisher_->publish(variable);
      steer_publisher_->publish(need);
      brake_publisher_->publish(brake_variable);
      // std::cout<<msg->brake<<std::endl;
      // Write the message that was received on the console window
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    // Declare the subscription attribute
    rclcpp::Subscription<autoware_auto_msgs::msg::VehicleStateCommand>::SharedPtr gear_subscription_;
    rclcpp::Subscription<autoware_auto_msgs::msg::RawControlCommand>::SharedPtr data_subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gear_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr data_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steer_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Intermediate>());
  rclcpp::shutdown();
  return 0;
}
