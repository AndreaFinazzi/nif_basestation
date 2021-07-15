// Copyright 2020 the Autoware Foundation
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
#include "joystick_vehicle_interface/joystick_vehicle_interface.hpp"
#include<iostream>

bool gear_allowed = false;

namespace joystick_vehicle_interface
{
JoystickVehicleInterface::JoystickVehicleInterface(
  const AxisMap & axis_map,
  const AxisScaleMap & axis_scale_map,
  const AxisScaleMap & axis_offset_map,
  const ButtonMap & button_map)
{
  m_axis_map = axis_map;
  m_axis_scale_map = axis_scale_map;
  m_axis_offset_map = axis_offset_map;
  m_button_map = button_map;
}

void JoystickVehicleInterface::axis_value(
  const sensor_msgs::msg::Joy & msg,
  Axes axis, double & value) const
{
  const auto axis_it = m_axis_map.find(axis);
  if (m_axis_map.end() == axis_it) {
    return;
  }
  const auto axis_idx = axis_it->second;
  if (axis_idx >= msg.axes.size()) {
    return;
  }
  const auto scale_it = m_axis_scale_map.find(axis);
  const auto scale = m_axis_scale_map.end() == scale_it ? DEFAULT_SCALE : scale_it->second;
  const auto val_raw = msg.axes[axis_idx] * scale;
  const auto offset_it = m_axis_offset_map.find(axis);
  const auto offset = m_axis_offset_map.end() == offset_it ? DEFAULT_OFFSET : offset_it->second;
  // do some magic based on empiric findings
  value = (val_raw + offset)*27579;
}

void JoystickVehicleInterface::axis_value_throttle(
  const sensor_msgs::msg::Joy & msg,
  Axes axis, double & value) const
{
  const auto axis_it = m_axis_map.find(axis);
  if (m_axis_map.end() == axis_it) {
    return;
  }
  const auto axis_idx = axis_it->second;
  if (axis_idx >= msg.axes.size()) {
    return;
  }
  const auto scale_it = m_axis_scale_map.find(axis);
  const auto scale = m_axis_scale_map.end() == scale_it ? DEFAULT_SCALE : scale_it->second;
  // do some magic based on empiric findings
  const auto altered_controller_output = (msg.axes[axis_idx] - 0.9)*(1/1.9); 
  const auto val_raw = altered_controller_output * scale; 
  const auto offset_it = m_axis_offset_map.find(axis);
  const auto offset = m_axis_offset_map.end() == offset_it ? DEFAULT_OFFSET : offset_it->second;
  value = val_raw + offset;
}

void JoystickVehicleInterface::axis_value_steer(
  const sensor_msgs::msg::Joy & msg,
  Axes axis, double & value) const
{
  const auto axis_it = m_axis_map.find(axis);
  if (m_axis_map.end() == axis_it) {
    return;
  }
  const auto axis_idx = axis_it->second;
  if (axis_idx >= msg.axes.size()) {
    return;
  }
  const auto scale_it = m_axis_scale_map.find(axis);
  const auto scale = m_axis_scale_map.end() == scale_it ? DEFAULT_SCALE : scale_it->second;
  const auto val_raw = - msg.axes[axis_idx] * scale;
  const auto offset_it = m_axis_offset_map.find(axis);
  const auto offset = m_axis_offset_map.end() == offset_it ? DEFAULT_OFFSET : offset_it->second;
  // do some magic based on empiric findings
  value = (-((val_raw + offset) * 220)/100)/(9); 
}

bool JoystickVehicleInterface::update_state_command(const sensor_msgs::msg::Joy & msg)
{
  auto ret = false;
  m_state_command = decltype(m_state_command) {};
  m_state_command.stamp = msg.header.stamp;
  
  for (const auto & button_idx : m_button_map) {
    const auto idx = button_idx.second;
    // Check if button is in range and active
    if (idx < msg.buttons.size()) {
      if (1 == msg.buttons[idx]) {
        ret = handle_active_button(button_idx.first) || ret;
      }
    }
  }
  return ret;
}

bool JoystickVehicleInterface::handle_active_button(Buttons button)
{
  auto ret = true;
  // int flag = 0;
  using VSC = decltype(m_state_command);
  switch (button) {
    case Buttons::GEAR_REVERSE: 
      if (gear_allowed == true)
      {
        m_shift_down = true; 
        gear_allowed = false; 
      }
      break;
    case Buttons::GEAR_DRIVE:
      if (gear_allowed == true)
      {
        m_shift_up = true; 
        gear_allowed = false; 
      }
      break;
    case Buttons::GEAR_PARK:
      gear_allowed = true;
      break;
    case Buttons::GEAR_NEUTRAL:
      gear_allowed = true;
      break;
    default:
      throw std::logic_error{"Impossible button was pressed"};
  }
  return ret;
}

const VehicleStateCommand & JoystickVehicleInterface::get_state_command()
{
  return m_state_command;
}

bool JoystickVehicleInterface::get_shift_down()
{
  return m_shift_down;
}

bool JoystickVehicleInterface::get_shift_up()
{
  return m_shift_up;
}

}  // namespace joystick_vehicle_interface
