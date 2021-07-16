// Copyright 2020-2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#ifndef JOYSTICK_VEHICLE_INTERFACE_NODES__JOYSTICK_VEHICLE_INTERFACE_NODE_HPP_
#define JOYSTICK_VEHICLE_INTERFACE_NODES__JOYSTICK_VEHICLE_INTERFACE_NODE_HPP_

#include <joystick_vehicle_interface/joystick_vehicle_interface.hpp>
#include <joystick_vehicle_interface_nodes/visibility_control.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "raptor_dbw_msgs/msg/gear_report.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

using joystick_vehicle_interface::Axes;
using joystick_vehicle_interface::Buttons;
using joystick_vehicle_interface::AxisMap;
using joystick_vehicle_interface::AxisScaleMap;
using joystick_vehicle_interface::ButtonMap;

using namespace std::chrono_literals;

namespace joystick_vehicle_interface_nodes
{

/// A node which translates sensor_msgs/msg/Joy messages into messages compatible with the vehicle
/// interface. All participants use SensorDataQoS
class JOYSTICK_VEHICLE_INTERFACE_NODES_PUBLIC JoystickVehicleInterfaceNode : public ::rclcpp::Node
{
public:
  /// ROS 2 parameter constructor
  explicit JoystickVehicleInterfaceNode(const rclcpp::NodeOptions & node_options);

private:
  // parser of joystick commands
  std::unique_ptr<joystick_vehicle_interface::JoystickVehicleInterface> m_core;
  /// Callback for joystick subscription: compute control and state command and publish
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub;
  JOYSTICK_VEHICLE_INTERFACE_NODES_LOCAL void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);

  // basic functionality
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_emergency_stop;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_heartbeat;
  unsigned int cnt = 0; 

  // vehicle control
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_gear_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_accelerator_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_steering_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_brake_pub;

  // vehicle feedback 
  rclcpp::Subscription<raptor_dbw_msgs::msg::GearReport>::SharedPtr m_gear_sub;
  void on_gear_rcv(const raptor_dbw_msgs::msg::GearReport::SharedPtr msg);

  // variables for shifting state machine
  bool try_shifting; 
  int shifting_counter; 
  int current_gear;
  int desired_gear;
  rclcpp::TimerBase::SharedPtr shift_sequence_timer;
  void shift_sequence_update();

};  // class JoystickVehicleInterfaceNode
}  // namespace joystick_vehicle_interface_nodes

#endif  // JOYSTICK_VEHICLE_INTERFACE_NODES__JOYSTICK_VEHICLE_INTERFACE_NODE_HPP_
