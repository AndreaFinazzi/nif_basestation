#include <memory>
#include <deep_orange_msgs/msg/base_to_car_summary.hpp>
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
      1s, std::bind(&MinimalSubscriber::timer_callback, this));

      //For RC flag input
      rc_subscriber = this->create_subscription<deep_orange_msgs::msg::BaseToCarSummary>(
      "rc_flag", 10, std::bind(&MinimalSubscriber::rc_callback, this, _1));
      rc_publisher_ = this->create_publisher<deep_orange_msgs::msg::BaseToCarSummary>("rc_to_ct/flag_summary", rclcpp::ServicesQoS());
      old_rc_publisher_ = this->create_publisher<deep_orange_msgs::msg::RcToCt>("raptor_dbw_interface/rc_to_ct", rclcpp::ServicesQoS());

    }

  private:
    unsigned short int hb = 0;
    unsigned short int rcflag = 1;
    unsigned short int vehflag = 0;

    std::array<bool, 16> old_flag_true = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true};
    std::array<bool, 16> old_flag_false = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};

    std::array<bool, 16> black = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    std::array<bool, 16> checkered = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    std::array<bool, 16> purple = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

    void timer_callback()
    {
      auto flag_summary = deep_orange_msgs::msg::BaseToCarSummary();
      flag_summary.stamp = this->now();
      flag_summary.track_flag = rcflag;
      flag_summary.veh_flag = vehflag;
      flag_summary.base_to_car_heartbeat = hb++;
      rc_publisher_->publish(flag_summary);

      auto RCInfo = deep_orange_msgs::msg::RcToCt();
      RCInfo.track_cond = rcflag;
      RCInfo.black = black;
      RCInfo.checkered = checkered;
      RCInfo.purple = purple;
      old_rc_publisher_->publish(RCInfo);

      if (hb > 7) hb = 0;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<deep_orange_msgs::msg::BaseToCarSummary>::SharedPtr rc_publisher_;
    rclcpp::Publisher<deep_orange_msgs::msg::RcToCt>::SharedPtr old_rc_publisher_;

    //For RC flag
    void rc_callback(const deep_orange_msgs::msg::BaseToCarSummary::SharedPtr msg) 
    {
      rcflag = msg->track_flag;
      vehflag = msg->veh_flag;

      switch (vehflag) 
      {
        case 1:
          black = old_flag_false;
          checkered = old_flag_true;
          purple = old_flag_false;
          break;

        case 2:
          black = old_flag_true;
          checkered = old_flag_false;
          purple = old_flag_false;
          break;

        case 8:
          black = old_flag_false;
          checkered = old_flag_false;
          purple = old_flag_true;
          break;

        default:
          black = old_flag_false;
          checkered = old_flag_false;
          purple = old_flag_false;
      }

      RCLCPP_INFO(this->get_logger(), "Track Condition: '%u' \t Vehicle Flag: '%u'", rcflag, vehflag);
    }

    rclcpp::Subscription<deep_orange_msgs::msg::BaseToCarSummary>::SharedPtr rc_subscriber;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}