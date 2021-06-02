#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("heart_exec_node_TX")
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
      i = 0;
      timer_ = this->create_wall_timer(50ms, std::bind(&MinimalPublisher::publish_message, this));
    }

  private:
    void publish_message()
    {
      auto message = std_msgs::msg::Int32();
      if (i<=9)
      {
      message.data = i;
      //RCLCPP_INFO(this->get_logger(), "Sending heartbeat counter: '%f'", message->data);
      std::cout<<"Sending heartbeat counter:  "<< message.data<<"\n";
      publisher_->publish(message);

      i += 1;
      }
      else
      {
        i = 0;
      }

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    int i;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
