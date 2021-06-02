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

template<typename T>
void JoystickVehicleInterface::axis_value(
  const sensor_msgs::msg::Joy & msg,
  Axes axis, T & value) const
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
  using ValT = std::decay_t<decltype(value)>;
  // value = static_cast<ValT>(val_raw + offset)*17236; ////// BRAKE PRESSURE VALUE IN PASCAL DIVIDE PASCAL BY 100 -EG 250 PSI - 1723600/100 = 17236
  value = static_cast<ValT>(val_raw + offset)*27579;
  // change brake
  // std::cout<< offset <<"\t" << msg.axes[axis_idx] << "\t" << val_raw <<"\t" << value <<"\t" << scale<< "\t" << axis_idx << "\t" <<std::endl;
}

template<typename T>
void JoystickVehicleInterface::axis_value_throttle(
  const sensor_msgs::msg::Joy & msg,
  Axes axis, T & value) const
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
  using ValT = std::decay_t<decltype(value)>;
  value = static_cast<ValT>(val_raw + offset);

  uint32_t checker = 0;

  if (value >=0 && value <=20) value = value; // change app
  else value = checker;

  // std::cout<< offset <<"\t" << msg.axes[axis_idx] << "\t" << val_raw <<"\t" << value <<"\t" << scale<< "\t" << axis_idx << "\t" <<std::endl;
}

template<typename T>
void JoystickVehicleInterface::axis_value_steer(
  const sensor_msgs::msg::Joy & msg,
  Axes axis, T & value) const
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
  using ValT = std::decay_t<decltype(value)>;
  value = static_cast<ValT>(-((val_raw + offset) * 220)/100)/(9); /// REMOVE NEGATIVE SIGN FOR REVERSE SIGN CONVENTION EG FROM 24 (LEFT) TO -24 (LEFT) AND -24 (RIGHT) TO 24 (RIGHT)
  // value = static_cast<ValT>(-((val_raw + offset) * 220)/100)*3.14/(9.0*180.0); /// REMOVE NEGATIVE SIGN FOR REVERSE SIGN CONVENTION EG FROM 24 (LEFT) TO -24 (LEFT) AND -24 (RIGHT) TO 24 (RIGHT)
  // std::cout<< offset <<"\t" << msg.axes[axis_idx] << "\t" << val_raw <<"\t" << value <<"\t" << scale<< "\t" << axis_idx << "\t" <<std::endl;
}

template<>
JoystickVehicleInterface::HighLevelControl
JoystickVehicleInterface::compute_command(const sensor_msgs::msg::Joy & msg)
{
  HighLevelControl ret{};
  {
    ret.stamp = msg.header.stamp;
    ret.velocity_mps = m_velocity;
    axis_value(msg, Axes::CURVATURE, ret.curvature);
    // axis_value(msg, Axes::BRAKE, ret.brake);
    // axis_value_throttle(msg, Axes::THROTTLE, ret.throttle);
    // axis_value_steer(msg, Axes::FRONT_STEER, ret.front_steer);
    // axis_value(msg, Axes::REAR_STEER, ret.rear_steer);
  }
  return ret;
}

template<>
JoystickVehicleInterface::RawControl
JoystickVehicleInterface::compute_command(const sensor_msgs::msg::Joy & msg)
{
  RawControl ret{};
  ret.stamp = msg.header.stamp;
  // std::cout<<msg.axes[2]<<std::endl;
  axis_value(msg, Axes::BRAKE, ret.brake);
  axis_value_throttle(msg, Axes::THROTTLE, ret.throttle);
  axis_value_steer(msg, Axes::FRONT_STEER, ret.front_steer);
  // axis_value(msg, Axes::REAR_STEER, ret.rear_steer);
  // std::cout<< "Raw control: " << ret.brake << std::endl;
  return ret;
}

template<>
JoystickVehicleInterface::BasicControl
JoystickVehicleInterface::compute_command(const sensor_msgs::msg::Joy & msg)
{
  BasicControl ret{};
  {
    ret.stamp = msg.header.stamp;
    axis_value(msg, Axes::ACCELERATION, ret.long_accel_mps2);
    axis_value(msg, Axes::FRONT_STEER, ret.front_wheel_angle_rad);
    axis_value(msg, Axes::REAR_STEER, ret.rear_wheel_angle_rad);
  }
  return ret;
}

bool JoystickVehicleInterface::update_state_command(const sensor_msgs::msg::Joy & msg)
{
  auto ret = false;
  // std::cout<<unsigned(gear_val)<<std::endl;
  m_state_command = decltype(m_state_command) {};
  m_state_command.stamp = msg.header.stamp;
  // int gear_val = 0;
  
  for (const auto & button_idx : m_button_map) {
    const auto idx = button_idx.second;
    // Check if button is in range and active
    if (idx < msg.buttons.size()) {
      if (1 == msg.buttons[idx]) {
        ret = handle_active_button(button_idx.first) || ret;
      }
    }
  }
  if (ret) {
    m_state_command.hand_brake = m_hand_brake_on;
    m_state_command.horn = m_horn_on;
  }
  return ret;
}

bool JoystickVehicleInterface::handle_active_button(Buttons button)
{
  auto ret = true;
  // int flag = 0;
  using VSC = decltype(m_state_command);
  switch (button) {
    case Buttons::VELOCITY_UP:  ///< For high level control
      ret = false;
      m_velocity += VELOCITY_INCREMENT;
      break;
    case Buttons::VELOCITY_DOWN:   ///< For high level control
      ret = false;
      m_velocity -= VELOCITY_INCREMENT;
      break;
    case Buttons::AUTONOMOUS_TOGGLE:
      m_state_command.mode = m_autonomous ? VSC::MODE_MANUAL : VSC::MODE_AUTONOMOUS;
      m_autonomous = !m_autonomous;
      break;
    case Buttons::HEADLIGHTS_TOGGLE:
      m_state_command.headlight = m_headlights_on ? VSC::HEADLIGHT_OFF : VSC::HEADLIGHT_ON;
      m_headlights_on = !m_headlights_on;
      break;
    case Buttons::WIPER_TOGGLE:
      m_state_command.wiper = m_wipers_on ? VSC::WIPER_OFF : VSC::WIPER_LOW;
      m_wipers_on = !m_wipers_on;
      break;
    case Buttons::HAND_BRAKE_TOGGLE:
      m_hand_brake_on = !m_hand_brake_on;
      break;
    case Buttons::HORN_TOGGLE:
      m_horn_on = !m_horn_on;
      break;

    // case Buttons::GEAR_DRIVE:
    //   if (m_state_command.gear>0 && m_state_command.gear < 6) m_state_command.gear = gear_val++;
    //   if (m_state_command.gear >=6 && m_state_command.gear < 255) break;
    //   break;
    // case Buttons::GEAR_REVERSE:
    //   // m_state_command.gear = VSC::GEAR_REVERSE;
    //   if(m_state_command.gear>0 && m_state_command.gear <6) m_state_command.gear = gear_val--;
    //   else if (m_state_command.gear >=6 && m_state_command.gear < 255) break;
    //   break;
    case Buttons::GEAR_REVERSE: 
      // std::cout<<"A pressed, gear_allowed "<<gear_allowed<<std::endl;
      if (gear_allowed == true)
      {
        gear_val = uint8_t (gear_val + increment);
        gear_allowed = false;
        if (gear_val > uint8_t(6)) gear_val = uint8_t(6);
      }
      m_state_command.gear = gear_val;
      // std::cout<<"m state gear "<<unsigned(m_state_command.gear)<<" gear val "<<unsigned(gear_val)<<std::endl;
      break;
    case Buttons::GEAR_DRIVE:
      if (gear_allowed == true)
      {
        if (gear_val > uint8_t(0))
        {
          gear_val = uint8_t (gear_val - decrement);
          gear_allowed = false;
        }
      }
      m_state_command.gear = gear_val;
      break;
    case Buttons::GEAR_PARK:
      gear_allowed = true;
      m_state_command.gear = gear_val;
      // std::cout<<"B pressed, gear_allowed "<<gear_allowed<<std::endl;
      //m_state_command.gear = VSC::GEAR_PARK;
      break;
    case Buttons::GEAR_NEUTRAL:
      // std::cout<<"neutral pressed, gear_allowed "<<std::endl;
      gear_allowed = true;
      m_state_command.gear = gear_val;
      break;
    case Buttons::GEAR_LOW:
      m_state_command.gear = VSC::GEAR_LOW;
      break;
    case Buttons::BLINKER_LEFT:
      m_state_command.blinker = VSC::BLINKER_LEFT;
      break;
    case Buttons::BLINKER_RIGHT:
      m_state_command.blinker = VSC::BLINKER_RIGHT;
      break;
    case Buttons::BLINKER_HAZARD:
      m_state_command.blinker = VSC::BLINKER_HAZARD;
      break;
    case Buttons::RECORDREPLAY_START_RECORD:
      m_recordreplay_command.data =
        static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::START_RECORD);
      break;
    case Buttons::RECORDREPLAY_START_REPLAY:
      m_recordreplay_command.data =
        static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::START_REPLAY);
      break;
    case Buttons::RECORDREPLAY_STOP:
      m_recordreplay_command.data =
        static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::STOP);
      break;
    default:
      throw std::logic_error{"Impossible button was pressed"};
  }
  return ret;
}

void JoystickVehicleInterface::reset_recordplay()
{
  m_recordreplay_command.data =
    static_cast<decltype(std_msgs::msg::UInt8::data)>(Recordreplay::NOOP);
}

const VehicleStateCommand & JoystickVehicleInterface::get_state_command()
{
  return m_state_command;
}

const std_msgs::msg::UInt8 & JoystickVehicleInterface::get_recordreplay_command()
{
  return m_recordreplay_command;
}

}  // namespace joystick_vehicle_interface
