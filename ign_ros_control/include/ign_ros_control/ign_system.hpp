// Copyright 2022 The ROS-Control team.
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

#ifndef IGN_ROS_CONTROL__IGN_SYSTEM_HPP_
#define IGN_ROS_CONTROL__IGN_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ign_ros_control/ign_system_interface.hpp"

namespace ign_ros_control
{
// Forward declaration
class IgnitionSystemPrivate;

// These class must inherit `ign_ros_control::IgnitionSystemInterface` which implements a
// simulated `ros_control` `hardware_interface::SystemInterface`.

class IgnitionSystem : public IgnitionSystemInterface
{
public:

  IgnitionSystem();

  virtual ~IgnitionSystem() override;

  //// Documentation Inherited
  virtual void read() override;
  //
  //// Documentation Inherited
  virtual void write() override;

  // Documentation Inherited
  bool initSim(
      ros::NodeHandle model_nh,
      std::map<std::string, ignition::gazebo::Entity> & joints,
      ignition::gazebo::EntityComponentManager & ecm,
      std::vector<transmission_interface::TransmissionInfo> transmissions,
      int & update_rate) override;

private:

  /// \brief Private data class
  std::unique_ptr<IgnitionSystemPrivate> dataPtr;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
};

}  // namespace ign_ros_control

#endif  // IGN_ROS_CONTROL__IGN_SYSTEM_HPP_
