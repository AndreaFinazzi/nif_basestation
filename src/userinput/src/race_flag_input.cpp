#include "deep_orange_msgs/msg/base_to_car_summary.hpp"
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using namespace std::chrono_literals;
using deep_orange_msgs::msg::BaseToCarSummary;
class RCFlagInput : public rclcpp::Node
{
  public:
    RCFlagInput()
    : Node("rcflag_publisher")
    {
      publisher_ = this->create_publisher<deep_orange_msgs::msg::BaseToCarSummary>("/rc_flag", 10);
      timer_ = this->create_wall_timer(
      1s, std::bind(&RCFlagInput::timer_callback, this));
    }

  private:

    void timer_callback()
    {
      unsigned int rcflag;
      // bool confirm;
      unsigned int vehcond;
      auto flag_summary = deep_orange_msgs::msg::BaseToCarSummary();
      std::cout << "Enter RC flag input [0-4] [0 - null, 1 - Red, 2 - Orange, 3 - Yellow, 4 - Green]:";
      std::cin >> rcflag;
      std::string confirmation_race_flag = "oogabooga";
      while (!(confirmation_race_flag.compare("Y")==0 || confirmation_race_flag.compare("n")==0))
      {
        if ((!rclcpp::ok()) || confirmation_race_flag.compare("")==0)
        {
          return;
        }
        std::cout << "You entered race flag " << rcflag << ". Are you sure this is what you want?[Y/n]";//<<std::endl;
        std::cin >> confirmation_race_flag;
      }
      if (confirmation_race_flag.compare("n")==0)
      {
        std::cout<<"Roger that, not setting race flag"<<std::endl;
        return;
      }
      
      
      if( rcflag > 4 || rcflag < 0 ) {
        throw std::domain_error{"Invalid Race flag"};
      }
      flag_summary.track_flag = rcflag;

      std::cout << "Enter vehicle flag: [0 - null, 1 - checkered, 2 - black, 8 - purple ]";
      std::cin >> vehcond;
      std::string confirmation_vehicle_flag = "oogabooga";
      while (!(confirmation_vehicle_flag.compare("Y")==0 || confirmation_vehicle_flag.compare("n")==0))
      {
        if (!rclcpp::ok() || confirmation_vehicle_flag.compare("")==0)
        {
          return;
        }
        std::cout << "You entered vehicle flag " << vehcond << ". Are you sure this is what you want?[Y/n]";//<<std::endl;
        std::cin >> confirmation_vehicle_flag;
      }
      if (confirmation_vehicle_flag.compare("n")==0)
      {
        std::cout<<"Roger that, not setting vehicle flag"<<std::endl;
        return;
      }
      if(vehcond == 1){
        flag_summary.veh_flag = 1;
      }
      else if(vehcond == 2){
        flag_summary.veh_flag = 2;
      }
      else if(vehcond == 8){
        flag_summary.veh_flag = 8;
      }
      else{
        flag_summary.veh_flag = 0;
      }
    
      RCLCPP_INFO(this->get_logger(), "Publishing race flag: '%u', vehicle flag: '%u'", flag_summary.track_flag, vehcond);
      publisher_->publish(flag_summary);
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<deep_orange_msgs::msg::BaseToCarSummary>::SharedPtr publisher_;
    // size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RCFlagInput>());
  rclcpp::shutdown();
  return 0;
}