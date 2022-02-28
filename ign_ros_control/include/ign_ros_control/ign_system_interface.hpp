// Copyright 2022 The ros_control team.
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

#ifndef IGN_ROS_CONTROL__IGN_SYSTEM_INTERFACE_HPP_
#define IGN_ROS_CONTROL__IGN_SYSTEM_INTERFACE_HPP_

#include <ignition/gazebo/System.hh>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>

// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>

namespace ign_ros_control
{

// SystemInterface provides API-level access to read and command joint properties.
class IgnitionSystemInterface
    : public hardware_interface::RobotHW
{
public:

  /// \brief Initialize the system interface
  /// param[in] model_nh Pointer to the ros node
  /// param[in] joints Map with the name of the joint as the key and the value is
  /// related with the entity in Gazebo
  /// param[in] _ecm Entity-component manager.
  /// param[in] transmissions structure to handle joint with transmissions
  /// param[in] update_rate controller update rate
  virtual bool initSim(
      ros::NodeHandle model_nh,
      std::map<std::string, ignition::gazebo::Entity> & joints,
      ignition::gazebo::EntityComponentManager & _ecm,
      std::vector<transmission_interface::TransmissionInfo> transmissions,
      int & update_rate) = 0;

  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

  //// Documentation Inherited
  virtual void read() = 0;
  //
  //// Documentation Inherited
  virtual void write() = 0;

protected:

  ros::NodeHandle nh_;
};

}  // namespace ign_ros_control

#endif  // IGN_ROS_CONTROL__IGN_SYSTEM_INTERFACE_HPP_
