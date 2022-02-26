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

#include "ign_ros_control/ign_system.hpp"

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>

#include <ignition/msgs/imu.pb.h>

#include <ignition/transport/Node.hh>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/hardware_interface.h>

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief handles to the joints from within Gazebo
  ignition::gazebo::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  ign_ros_control::IgnitionSystemInterface::ControlMethod joint_control_method;
};

class ign_ros_control::IgnitionSystemPrivate
{
public:
  IgnitionSystemPrivate() = default;

  ~IgnitionSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  ros::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  ignition::gazebo::EntityComponentManager* ecm_;

  /// \brief controller update rate
  int update_rate_;

  /// \brief Ignition communication node.
  ignition::transport::Node node;
};

namespace ign_ros_control
{

IgnitionSystem::IgnitionSystem()
  : dataPtr(nullptr)
{

}

IgnitionSystem::~IgnitionSystem() = default;

bool IgnitionSystem::initSim(ros::NodeHandle model_nh,
                             std::map<std::string, ignition::gazebo::Entity> & enableJoints,
                             ignition::gazebo::EntityComponentManager & ecm,
                             std::vector<transmission_interface::TransmissionInfo> transmissions,
                             int & update_rate)
{
  this->dataPtr = std::make_unique<IgnitionSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = ros::Time::now();

  this->nh_ = model_nh;
  this->dataPtr->ecm_ = &ecm;
  this->dataPtr->n_dof_ = transmissions.size();
  this->dataPtr->update_rate_ = update_rate;

  ROS_DEBUG("[Ignition ROS Control] n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

  if (this->dataPtr->n_dof_ == 0) {
    ROS_WARN_STREAM("[Ignition ROS Control] There is not joint available");
    return false;
  }

  std::string joint_name;
  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    joint_name = this->dataPtr->joints_[j].name = transmissions[j].joints_[0].name_;

    ignition::gazebo::Entity simjoint = enableJoints[joint_name];
    this->dataPtr->joints_[j].sim_joint = simjoint;

    // Create joint position component if one doesn't exist
    if (!ecm.EntityHasComponentType(
          simjoint,
          ignition::gazebo::components::JointPosition().TypeId()))
    {
      ecm.CreateComponent(simjoint, ignition::gazebo::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!ecm.EntityHasComponentType(
          simjoint,
          ignition::gazebo::components::JointVelocity().TypeId()))
    {
      ecm.CreateComponent(simjoint, ignition::gazebo::components::JointVelocity());
    }

    // Create joint force component if one doesn't exist
    if (!ecm.EntityHasComponentType(
          simjoint,
          ignition::gazebo::components::JointForce().TypeId()))
    {
      ecm.CreateComponent(simjoint, ignition::gazebo::components::JointForce());
    }

    // Accept this joint and continue configuration
    ROS_INFO_STREAM("[Ignition ROS Control] Loading joint: " << joint_name);

    // Add data from transmission
    this->dataPtr->joints_[j].joint_position     = 0.0;
    this->dataPtr->joints_[j].joint_velocity     = 0.0;
    this->dataPtr->joints_[j].joint_effort       = 0.0;  // N/m for continuous joints
    this->dataPtr->joints_[j].joint_effort_cmd   = 0.0;
    this->dataPtr->joints_[j].joint_position_cmd = 0.0;
    this->dataPtr->joints_[j].joint_velocity_cmd = 0.0;

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
    {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM("[Ignition ROS Control] The <hardware_interface> element of tranmission " <<
                      transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
                      "The transmission will be properly loaded, but please update " <<
                      "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty())
    {
      ROS_WARN_STREAM("[Ignition ROS Control] Joint " << transmissions[j].joints_[0].name_ <<
                                                                                              " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
                                                                                              "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      ROS_WARN_STREAM("[Ignition ROS Control] Joint " << transmissions[j].joints_[0].name_ <<
                                                                                              " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
                                                                                              "Currently the default robot hardware simulation interface only supports one. Using the first entry");
      //continue;
    }

    const std::string& hardware_interface = joint_interfaces.front();

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
                                   joint_name, &this->dataPtr->joints_[j].joint_position, &this->dataPtr->joints_[j].joint_velocity, &this->dataPtr->joints_[j].joint_effort));

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;
    if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface")
    {
      // Create effort joint interface
      this->dataPtr->joints_[j].joint_control_method = EFFORT;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_name),
                                                     &this->dataPtr->joints_[j].joint_effort_cmd);
      ej_interface_.registerHandle(joint_handle);
    }
    else if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")
    {
      // Create position joint interface
      this->dataPtr->joints_[j].joint_control_method = POSITION;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_name),
                                                     &this->dataPtr->joints_[j].joint_position_cmd);
      pj_interface_.registerHandle(joint_handle);
    }
    else if(hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface")
    {
      // Create velocity joint interface
      this->dataPtr->joints_[j].joint_control_method = VELOCITY;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_name),
                                                     &this->dataPtr->joints_[j].joint_velocity_cmd);
      vj_interface_.registerHandle(joint_handle);
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
                             << hardware_interface << "' while loading interfaces for " << joint_name );
      return false;
    }

    if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface") {
      ROS_WARN_STREAM("[Ignition ROS Control] Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface << "' within the <hardwareInterface> tag in joint '" << joint_name << "'.");
    }
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  return true;
}

void IgnitionSystem::read()
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    // Get the joint velocity
    const auto * jointVelocity =
        this->dataPtr->ecm_->Component<ignition::gazebo::components::JointVelocity>(
          this->dataPtr->joints_[i].sim_joint);

    // TODO(ahcorde): Revisit this part ignitionrobotics/ign-physics#124
    // Get the joint force
    //const auto * jointForce =
    //  this->dataPtr->ecm_->Component<ignition::gazebo::components::JointForce>(
    //  this->dataPtr->joints_[i].sim_joint);

    // Get the joint position
    const auto * jointPositions =
        this->dataPtr->ecm_->Component<ignition::gazebo::components::JointPosition>(
          this->dataPtr->joints_[i].sim_joint);

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
    //this->dataPtr->joints_[i].joint_effort   = jointForce->Data()[0];
  }
}

void IgnitionSystem::write()
{

  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].joint_control_method == VELOCITY) {
      if (!this->dataPtr->ecm_->Component<ignition::gazebo::components::JointVelocityCmd>(
            this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm_->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              ignition::gazebo::components::JointVelocityCmd({0}));
      } else {

        double targetVel = this->dataPtr->joints_[i].joint_velocity_cmd;
        auto vel =
            this->dataPtr->ecm_->Component<ignition::gazebo::components::JointVelocityCmd>(
              this->dataPtr->joints_[i].sim_joint);

        if (vel == nullptr) {
          this->dataPtr->ecm_->CreateComponent(
                this->dataPtr->joints_[i].sim_joint,
                ignition::gazebo::components::JointVelocityCmd({targetVel}));
        } else if (!vel->Data().empty()) {
          vel->Data()[0] = targetVel;
        }
      }
    }

    if (this->dataPtr->joints_[i].joint_control_method == POSITION) {
      // Get error in position
      double error;
      error = (this->dataPtr->joints_[i].joint_position -
               this->dataPtr->joints_[i].joint_position_cmd) * this->dataPtr->update_rate_;
      // Calculate target velocity
      double targetVel = -error;

      auto vel =
          this->dataPtr->ecm_->Component<ignition::gazebo::components::JointVelocityCmd>(
            this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr) {
        this->dataPtr->ecm_->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              ignition::gazebo::components::JointVelocityCmd({targetVel}));
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = targetVel;
      }
    }

    if (this->dataPtr->joints_[i].joint_control_method == EFFORT) {
      if (!this->dataPtr->ecm_->Component<ignition::gazebo::components::JointForceCmd>(
            this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm_->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              ignition::gazebo::components::JointForceCmd({0}));
      } else {
        const auto jointEffortCmd =
            this->dataPtr->ecm_->Component<ignition::gazebo::components::JointForceCmd>(
              this->dataPtr->joints_[i].sim_joint);
        *jointEffortCmd = ignition::gazebo::components::JointForceCmd(
        {this->dataPtr->joints_[i].joint_effort_cmd});
      }
    }
  }
}

}  // namespace ign_ros_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
    ign_ros_control::IgnitionSystem, ign_ros_control::IgnitionSystemInterface)
