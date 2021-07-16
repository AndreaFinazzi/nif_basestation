#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using namespace std::chrono_literals;
using deep_orange_msgs::msg::RcToCt;
class RCFlagInput : public rclcpp::Node
{
  public:
    RCFlagInput()
    : Node("rcflag_publisher")
    {
      publisher_ = this->create_publisher<deep_orange_msgs::msg::RcToCt>("/raptor_dbw_interface/rc_to_ct", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&RCFlagInput::timer_callback, this));
    }

  private:

    void timer_callback()
    {
      unsigned int rcflag;
      // bool confirm;
      unsigned int vehcond;
      auto RaceControlFlag = deep_orange_msgs::msg::RcToCt();
      std::cout << "Enter RC flag input [1-4] [1 - Red, 2 - Orange, 3 - Yellow, 4 - Green]:";
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
      
      
      if(rcflag>4 || rcflag< 1){
        throw std::domain_error{"Invalid Race flag"};
      }
      RaceControlFlag.track_cond = rcflag;

      std::cout << "Enter vehicle flag: [0 - null, 1 - checkered, 2 - black, 3 - purple ]";
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
        RaceControlFlag.black = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
        RaceControlFlag.checkered = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true};
        RaceControlFlag.purple = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
      }
      else if(vehcond == 2){
        RaceControlFlag.black = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true};
        RaceControlFlag.checkered = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
        RaceControlFlag.purple = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
      }
      else if(vehcond == 3){
        RaceControlFlag.black = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
        RaceControlFlag.checkered = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
        RaceControlFlag.purple = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true};
      }
      else{
         RaceControlFlag.black = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
         RaceControlFlag.checkered = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
         RaceControlFlag.purple = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
      }
    
      RCLCPP_INFO(this->get_logger(), "Publishing race flag: '%u', vehicle flag: '%u'", RaceControlFlag.track_cond, vehcond);
      publisher_->publish(RaceControlFlag);
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<deep_orange_msgs::msg::RcToCt>::SharedPtr publisher_;
    // size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RCFlagInput>());
  rclcpp::shutdown();
  return 0;
}