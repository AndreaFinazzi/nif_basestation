#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
using namespace std::chrono_literals;
using deep_orange_msgs::msg::RcToCt;
using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
  public:
    SubscriberNode()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&SubscriberNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
