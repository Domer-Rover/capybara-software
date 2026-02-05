// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "roboclaw_hardware_interface/roboclaw_unit.hpp"

#include <iostream>
#include <map>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <roboclaw_serial/command.hpp>
#include <roboclaw_serial/interface.hpp>

#include "roboclaw_hardware_interface/motor_joint.hpp"

namespace roboclaw_hardware_interface
{

RoboClawUnit::RoboClawUnit(
  roboclaw_serial::Interface::SharedPtr interface, uint8_t address, MotorJoint::SharedPtr m1,
  MotorJoint::SharedPtr m2)
: address_(address)
{
  // Copy the pointer to the roboclaw interface
  interface_ = interface;

  // Set motor joint configurations
  joints[0] = m1;
  joints[1] = m2;
}

// Read the encoder counts from the roboclaw and update position state
bool RoboClawUnit::read()
{
  try {
    // Read and update position (CRITICAL - must succeed)
    interface_->read(encoder_state_, address_);

    // Get constant references to fields in the encoder counts message
    const auto & [m1_ticks, m2_ticks] = encoder_state_.fields;

    // Convert tick counts to position states for each field if the corresponding joint exists
    if (joints[0]) {
      joints[0]->setPositionState(m1_ticks);
    }
    if (joints[1]) {
      joints[1]->setPositionState(m2_ticks);
    }

    // Try to read velocity - if this fails, we still have position (non-critical)
    try {
      // Read M1 speed
      if (joints[0]) {
        interface_->read(encoder_speed_m1_, address_);
        const auto & [m1_speed, m1_status] = encoder_speed_m1_.fields;
        joints[0]->setVelocityState(m1_speed);
      }
      
      // Read M2 speed
      if (joints[1]) {
        interface_->read(encoder_speed_m2_, address_);
        const auto & [m2_speed, m2_status] = encoder_speed_m2_.fields;
        joints[1]->setVelocityState(m2_speed);
      }
    } catch (const std::exception & e) {
      // Velocity read failed - not critical, position is enough
      // Only log occasionally to avoid spam
      static int error_count = 0;
      if (++error_count % 100 == 0) {
        std::cerr << "[RoboClawUnit] Velocity read errors on address 0x" 
                  << std::hex << static_cast<int>(address_) << std::dec 
                  << " (count: " << error_count << ")" << std::endl;
      }
    }
    
    return true;
  } catch (const std::exception & e) {
    // Position read failed - this is critical!
    std::cerr << "[RoboClawUnit] Critical position read error on address 0x" 
              << std::hex << static_cast<int>(address_) << std::dec 
              << ": " << e.what() << std::endl;
    return false;
  }
}

// Write the tick rate request to the roboclaw and update
bool RoboClawUnit::write()
{
  try {
    // Get references to fields in the tick rate command message
    auto & [m1_speed, m2_speed] = tick_rate_command_.fields;

    // Set values to each field if the corresponding joint exists
    if (joints[0]) {
      m1_speed = joints[0]->getTickRateCommand();
    }
    if (joints[1]) {
      m2_speed = joints[1]->getTickRateCommand();
    }

    // Debug output (remove after testing)
    static int count = 0;
    if (++count % 20 == 0) {  // Print every 20 cycles (every 1 second at 20Hz)
      RCLCPP_INFO(rclcpp::get_logger("RoboClawUnit"), 
                  "Writing to address 0x%02X M1: %d M2: %d", 
                  address_, m1_speed, m2_speed);
    }

    // Write the rate request to the roboclaw driver
    interface_->write(tick_rate_command_, address_);
    return true;
  } catch (const std::exception & e) {
    std::cerr << "[RoboClawUnit] Write error on address 0x" 
              << std::hex << static_cast<int>(address_) << std::dec 
              << ": " << e.what() << std::endl;
    return false;
  }
}
}  // namespace roboclaw_hardware_interface
