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
#include <common/types.hpp>
#include <joystick_vehicle_interface_nodes/joystick_vehicle_interface_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdio.h>
#include<iostream>
#include <memory>
#include <string>
#include <type_traits>

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

bool flag_allowed = false;

namespace joystick_vehicle_interface_nodes
{

JoystickVehicleInterfaceNode::JoystickVehicleInterfaceNode(
  const rclcpp::NodeOptions & node_options)
: Node{"joystick_vehicle_interface_nodes", node_options}
{
  // topics
  const auto control_command =
    declare_parameter("control_command").get<std::string>();

  const auto control_command_high =
    declare_parameter("control_command_high").get<std::string>();
  const bool recordreplay_command_enabled =
    declare_parameter("recordreplay_command_enabled").get<bool8_t>();

  // maps
  const auto check_set = [this](auto & map, auto key, const std::string & param_name) {
      const auto param = declare_parameter(param_name);
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
        using MapT = std::remove_reference_t<decltype(map)>;
        using ValT = typename MapT::mapped_type;
        const auto val_raw =
          param.get<std::conditional_t<std::is_floating_point<ValT>::value, float64_t, int64_t>>();
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

  // Control commands
  m_enable_joy_pub = create_publisher<Int8need>(
      "emergency_stop",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  m_enable_joy_control_pub = create_publisher<Int8need>(
      "joy_control_enable",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  m_shutdown_pub = create_publisher<std_msgs::msg::Int8>(
      "joy_shutdown",
      rclcpp::QoS{10U}.reliable().durability_volatile());    
  if (control_command == "high_level") {
    m_cmd_pub = create_publisher<HighLevelControl>(
      "high_level_command",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  } else if (control_command == "raw") {
    m_cmd_pub = create_publisher<RawControl>(
      "raw_command",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  } else if (control_command == "basic") {
    m_cmd_pub = create_publisher<BasicControl>(
      "basic_command",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  } else {
    throw std::domain_error
          {"JoystickVehicleInterfaceNode does not support " + control_command +
            "command control mode"};
  }
  if (control_command_high == "high_level") {
    m_cmd_pub_high = create_publisher<HighLevelControl>(
      "high_level_command",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  } else if (control_command_high == "raw") {
    m_cmd_pub_high = create_publisher<RawControl>(
      "raw_command",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  } else if (control_command_high == "basic") {
    m_cmd_pub_high = create_publisher<BasicControl>(
      "basic_command",
      rclcpp::QoS{10U}.reliable().durability_volatile());
  } else {
    throw std::domain_error
          {"JoystickVehicleInterfaceNode does not support " + control_command_high +
            "command control mode"};
  }

  // State commands
  m_state_cmd_pub =
    create_publisher<autoware_auto_msgs::msg::VehicleStateCommand>(
    "state_command",
    rclcpp::QoS{10U}.reliable().durability_volatile());

  // Recordreplay command
  if (recordreplay_command_enabled) {
    m_recordreplay_cmd_pub = create_publisher<std_msgs::msg::UInt8>("recordreplay_cmd", 10);
  }

  // Joystick
  m_joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS{},
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) {on_joy(msg);});
  // Maps
  m_core = std::make_unique<joystick_vehicle_interface::JoystickVehicleInterface>(
    axis_map,
    axis_scale_map,
    axis_offset_map,
    button_map);
}


////////////////////////////////////////////////////////////////////////////////
void JoystickVehicleInterfaceNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // uint32_t emer_flag = 0;
  const auto send = msg->buttons[7];
  const auto send1 = msg->buttons[6];
  // int checker = 0;
  if (send == 1 && send1 == 1)
  {
    // variable.stamp = msg->header.stamp;
    variable.data = 0;
    flag_allowed = false;
    // std::cout<<"1 oduthu"<<std::endl;
  }
  // std::cout<<checker<<std::endl;
  m_enable_joy_pub->publish(variable);
  const auto send3 = msg->buttons[4];
  const auto send4 = msg->buttons[5];
  if (send3 == 1 && send4 == 1)
  {
    variable.data = 1;
    flag_allowed = true;
    // std::cout<<"3 oduthu"<<std::endl;
    m_enable_joy_pub->publish(variable);
  }
  
  const auto shutdown = msg->axes[6];
  variable_shutdown.data = 0;  //default
  if (shutdown == 1 )
  {
    variable_shutdown.data = -1;  //left press goes to ct_4
  }
  else if (shutdown == -1)
  {
    variable_shutdown.data = 1;  //right press goes to emergency shutdown
  }
  m_shutdown_pub->publish(variable_shutdown);
  
  const auto enable = msg->axes[7];
  if (enable == 1 )
  {
    variable_joy_control.data = 1;
  }
  else if (enable == -1)
  {
    variable_joy_control.data = 0;
  }
  m_enable_joy_control_pub->publish(variable_joy_control);
  // State command: modify state first
  if (m_core->update_state_command(*msg)) {
    auto state_command = m_core->get_state_command();
    m_state_cmd_pub->publish(state_command);
  }
  // Command publish
  const auto compute_publish_command = [this, &msg](auto && pub) -> void {
      using MessageT =
        typename std::decay_t<decltype(pub)>::element_type::MessageUniquePtr::element_type;
      const auto cmd = m_core->compute_command<MessageT>(*msg);
      pub->publish(cmd);
    };
  mpark::visit(compute_publish_command, m_cmd_pub);

  // Command publish
  const auto compute_publish_command_high = [this, &msg](auto && pub) -> void {
      using MessageT =
        typename std::decay_t<decltype(pub)>::element_type::MessageUniquePtr::element_type;
      const auto cmd = m_core->compute_command<MessageT>(*msg);
      pub->publish(cmd);
    };
  mpark::visit(compute_publish_command_high, m_cmd_pub_high);  

  auto recordreplay_command = m_core->get_recordreplay_command();
  if (m_recordreplay_cmd_pub != nullptr && recordreplay_command.data > 0u) {
    m_recordreplay_cmd_pub->publish(recordreplay_command);
    m_core->reset_recordplay();
  }
}

}  // namespace joystick_vehicle_interface_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(joystick_vehicle_interface_nodes::JoystickVehicleInterfaceNode)
