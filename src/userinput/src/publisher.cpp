#include <memory>
#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("publisher")
    {
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalSubscriber::timer_callback, this));
      
      //########################################################
      //For RC flag input
      rc_subscriber = this->create_subscription<deep_orange_msgs::msg::RcToCt>(
      "rc_flag", 10, std::bind(&MinimalSubscriber::rc_callback, this, _1));
      rc_publisher_ = this->create_publisher<deep_orange_msgs::msg::RcToCt>("raptor_dbw_interface/rc_to_ct", 10);
    }

  private:
    unsigned int rcflag = 1;

    std::array<bool, 16> black = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    std::array<bool, 16> checkered = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    std::array<bool, 16> purple = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

    void timer_callback()
    {
      auto RCInfo = deep_orange_msgs::msg::RcToCt();
      RCInfo.track_cond = rcflag;
      RCInfo.black = black;
      RCInfo.checkered = checkered;
      RCInfo.purple = purple;
      rc_publisher_->publish(RCInfo);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<deep_orange_msgs::msg::RcToCt>::SharedPtr rc_publisher_;

    //For RC flag
    void rc_callback(const deep_orange_msgs::msg::RcToCt::SharedPtr msg) 
    {
      RCLCPP_INFO(this->get_logger(), "Track Condition: '%u'", msg->track_cond);
      rcflag = msg->track_cond;
      black  = msg->black;
      checkered = msg->checkered;
      purple = msg->purple;
    }
    rclcpp::Subscription<deep_orange_msgs::msg::RcToCt>::SharedPtr rc_subscriber;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}