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
#include <joystick_vehicle_interface_nodes/joystick_vehicle_interface_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdio.h>
#include<iostream>
#include <memory>
#include <string>
#include <type_traits>

bool flag_allowed = false;

namespace joystick_vehicle_interface_nodes
{

JoystickVehicleInterfaceNode::JoystickVehicleInterfaceNode(
  const rclcpp::NodeOptions & node_options)
: Node{"joystick_vehicle_interface_nodes", node_options}
{
  // maps
  const auto check_set = [this](auto & map, auto key, const std::string & param_name) {
      const auto param = declare_parameter(param_name);
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
        using MapT = std::remove_reference_t<decltype(map)>;
        using ValT = typename MapT::mapped_type;
        const auto val_raw =
          param.get<std::conditional_t<std::is_floating_point<ValT>::value, double, int64_t>>();
        map[key] = static_cast<ValT>(val_raw);
      }
    };
  // axis map
  AxisMap axis_map{};
  check_set(axis_map, Axes::THROTTLE, "axes.throttle");
  check_set(axis_map, Axes::BRAKE, "axes.brake");
  check_set(axis_map, Axes::FRONT_STEER, "axes.front_steer");
  check_set(axis_map, Axes::REAR_STEER, "axes.rear_steer");
  check_set(axis_map, Axes::CURVATURE, "axes.curvature");
  check_set(axis_map, Axes::ACCELERATION, "axes.acceleration");
  // axis scale map
  AxisScaleMap axis_scale_map{};
  check_set(axis_scale_map, Axes::THROTTLE, "axis_scale.throttle");
  check_set(axis_scale_map, Axes::BRAKE, "axis_scale.brake");
  check_set(axis_scale_map, Axes::FRONT_STEER, "axis_scale.front_steer");
  check_set(axis_scale_map, Axes::REAR_STEER, "axis_scale.rear_steer");
  check_set(axis_scale_map, Axes::CURVATURE, "axis_scale.curvature");
  check_set(axis_scale_map, Axes::ACCELERATION, "axis_scale.acceleration");
  // axis offset map
  AxisScaleMap axis_offset_map{};
  check_set(axis_offset_map, Axes::THROTTLE, "axis_offset.throttle");
  check_set(axis_offset_map, Axes::BRAKE, "axis_offset.brake");
  check_set(axis_offset_map, Axes::FRONT_STEER, "axis_offset.front_steer");
  check_set(axis_offset_map, Axes::REAR_STEER, "axis_offset.rear_steer");
  check_set(axis_offset_map, Axes::CURVATURE, "axis_offset.curvature");
  check_set(axis_offset_map, Axes::ACCELERATION, "axis_offset.acceleration");
  // button map
  ButtonMap button_map{};
  check_set(button_map, Buttons::AUTONOMOUS_TOGGLE, "buttons.autonomous");
  check_set(button_map, Buttons::HEADLIGHTS_TOGGLE, "buttons.headlights");
  check_set(button_map, Buttons::WIPER_TOGGLE, "buttons.wiper");
  check_set(button_map, Buttons::GEAR_DRIVE, "buttons.gear_drive");
  check_set(button_map, Buttons::GEAR_REVERSE, "buttons.gear_reverse");
  check_set(button_map, Buttons::GEAR_PARK, "buttons.gear_park");
  check_set(button_map, Buttons::GEAR_NEUTRAL, "buttons.gear_neutral");
  check_set(button_map, Buttons::GEAR_LOW, "buttons.gear_low");
  check_set(button_map, Buttons::BLINKER_LEFT, "buttons.blinker_left");
  check_set(button_map, Buttons::BLINKER_RIGHT, "buttons.blinker_right");
  check_set(button_map, Buttons::BLINKER_HAZARD, "buttons.blinker_hazard");
  check_set(button_map, Buttons::VELOCITY_UP, "buttons.velocity_up");
  check_set(button_map, Buttons::VELOCITY_DOWN, "buttons.velocity_down");
  check_set(button_map, Buttons::RECORDREPLAY_START_RECORD, "buttons.recordreplay_start_record");
  check_set(button_map, Buttons::RECORDREPLAY_START_REPLAY, "buttons.recordreplay_start_replay");
  check_set(button_map, Buttons::RECORDREPLAY_STOP, "buttons.recordreplay_stop");

  // basic functionality (trigger emergency stop and send heartbeat)
  m_emergency_stop = create_publisher<std_msgs::msg::UInt8>("/emergency_stop", 1);
  m_heartbeat = create_publisher<std_msgs::msg::UInt8>("/counter", 1);

  // output vehicle commands
  m_gear_pub = create_publisher<std_msgs::msg::UInt8>("/gear_cmd", 1);
  m_accelerator_pub = create_publisher<std_msgs::msg::Float32>("/accelerator_cmd", 1);
  m_steering_pub = create_publisher<std_msgs::msg::Float32>("/steering_cmd", 1);
  m_brake_pub = create_publisher<std_msgs::msg::Float32>("/brake_cmd", 1);
  m_joy_enable_pub = create_publisher<std_msgs::msg::UInt8>("vehicle/joy_control_enable", 1);

  // Listen to joystick commands
  m_joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS{},
    std::bind(&JoystickVehicleInterfaceNode::on_joy, this, std::placeholders::_1));

  // gear subscription
  m_gear_sub = create_subscription<deep_orange_msgs::msg::PtReport>(
    "raptor_dbw_interface/pt_report", 1, 
    std::bind(&JoystickVehicleInterfaceNode::on_gear_rcv, this, std::placeholders::_1));

  // Maps
  m_core = std::make_unique<joystick_vehicle_interface::JoystickVehicleInterface>(
    axis_map,
    axis_scale_map,
    axis_offset_map,
    button_map);

  // initialize shifting state machine with no-shift state
  try_shifting = false; 
  shifting_counter = 0; 
  // -1 indicates that they haven't been initialized
  current_gear = -1;
  desired_gear = -1;
  // make shifting time a parameter 
  this->declare_parameter<int>("shift_time_ms", 1000);
  // initialize timer 
  shift_sequence_timer = this->create_wall_timer(
      20ms, std::bind(&JoystickVehicleInterfaceNode::shift_sequence_update, this));
}


////////////////////////////////////////////////////////////////////////////////
void JoystickVehicleInterfaceNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // do a purple flag shutdown if both levers are pressed
  const auto send3 = msg->buttons[4];
  const auto send4 = msg->buttons[5];
  if (send3 == 1 && send4 == 1)
  {
    auto msg = std_msgs::msg::UInt8();
    msg.data = 1;
    m_emergency_stop->publish(msg);
  }
  // enable vehicle control 
  const auto send7 = msg->axes[7];
  if(send7 == 1)
  {
    msg_enable.data = 1;
  }
  else if(send7 == -1)
  {
    msg_enable.data = 0;
  }
  m_joy_enable_pub->publish(msg_enable);

  // update gear shifting and output
  m_core->update_state_command(*msg);
  // only listen to gear shift commands while we are not already trying to shift
  if(!try_shifting)
  {
    // check if upshift was requested from the joystick and we are not already in sixth gear
    if(m_core->get_shift_up() && desired_gear < 6)
    {
      RCLCPP_INFO(this->get_logger(), "Shift up");
      desired_gear++;
      // initialize shifting sequence
      try_shifting = true; 
    }
    // check if downshift was requested and we are not already in first gear
    else if(m_core->get_shift_down() && desired_gear > 1)
    {
        RCLCPP_INFO(this->get_logger(), "Shift up");
        desired_gear--;
        // initialize shifting sequence
        try_shifting = true; 
    }
  }
  auto msg_gear = std_msgs::msg::UInt8(); 
  msg_gear.data = desired_gear;
  // only publish after it has been initialized
  if(desired_gear >= 0)
  {
    m_gear_pub->publish(msg_gear);
  }
  
  // output steering, throttle and brakes
  auto msg_throttle = std_msgs::msg::Float32();
  double data = 0; 
  m_core->axis_value_throttle(*msg, Axes::THROTTLE, data);
  msg_throttle.data = data;
  m_accelerator_pub->publish(msg_throttle);
  auto msg_brake = std_msgs::msg::Float32();
  m_core->axis_value(*msg, Axes::BRAKE, data);
  msg_brake.data = data;
  m_brake_pub->publish(msg_brake);
  auto msg_steering = std_msgs::msg::Float32();
  m_core->axis_value_steer(*msg, Axes::FRONT_STEER, data);
  msg_steering.data = data;
  m_steering_pub->publish(msg_steering);

  // publish heartbeat
  auto msg_heartbeat = std_msgs::msg::UInt8();
  msg_heartbeat.data = cnt;
  m_heartbeat->publish(msg_heartbeat);
}

void JoystickVehicleInterfaceNode::shift_sequence_update()
{
  // only update shift sequence if we are shifting
  if(try_shifting)
  {
    // increase shift sequence counter to wait a prescribed number of cycles
    shifting_counter++;
    // update shift time
    this->get_parameter("shift_time_ms", shift_time_ms);
    // if shifting sequence should be finished check desired and current gear
    // cycle time of this is 20ms
    if(20*shifting_counter >= shift_time_ms)
    {
      // stop shifting sequence
      try_shifting = false; 
      shifting_counter = 0; 
      // if the shift was not succesfull put desired gear back to current gear
      if(current_gear != desired_gear)
      {
        desired_gear = current_gear;
        RCLCPP_INFO(this->get_logger(), "Gear shift not succesfull. Reset desired gear to current gear.");
      }
    }
  }
}

void JoystickVehicleInterfaceNode::on_gear_rcv(const deep_orange_msgs::msg::PtReport::SharedPtr msg)
{
  current_gear = msg->current_gear;
  // if gear was not initialized also initialize desired gear with current gear
  if(desired_gear == -1)
  {
    desired_gear = current_gear;
  }
}

}  // namespace joystick_vehicle_interface_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_vehicle_interface_nodes::JoystickVehicleInterfaceNode)
