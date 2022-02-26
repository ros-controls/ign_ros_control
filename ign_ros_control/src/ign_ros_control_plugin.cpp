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

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Model.hh>

#include <ignition/plugin/Register.hh>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <controller_manager/controller_manager.h>

#include <hardware_interface/interface_resources.h>

#include <transmission_interface/transmission_parser.h>

#include <pluginlib/class_loader.hpp>

#include <urdf/model.h>

#include "ign_ros_control/ign_ros_control_plugin.hpp"
#include "ign_ros_control/ign_system.hpp"

namespace ign_ros_control
{
//////////////////////////////////////////////////
class IgnitionROSControlPluginPrivate
{
public:

  IgnitionROSControlPluginPrivate(const std::string& robot_namespace,
                                  const std::string& robot_description
                                  )
    : model_nh_(robot_namespace)
    , robot_description_(robot_description)
  {
    async_ros_spin_.reset(new ros::AsyncSpinner(0)); // will use a thread for each CPU core
    async_ros_spin_->start();
  }

  ~IgnitionROSControlPluginPrivate()
  {
    async_ros_spin_->stop();
  }

  /// \brief Get the URDF XML from the parameter server
  std::string getURDF(std::string param_name) const;

  bool parseTransmissionsFromURDF(const std::string &urdf_string);

  /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
  /// joint names are specified in the plugin configuration, all valid 1-axis
  /// joints are returned
  /// \param[in] _entity Entity of the model that the plugin is being
  /// configured for
  /// \param[in] _ecm Ignition Entity Component Manager
  /// \return List of entities containing all enabled joints
  std::map<std::string, ignition::gazebo::Entity> GetEnabledJoints(
      const ignition::gazebo::Entity & _entity,
      ignition::gazebo::EntityComponentManager & _ecm) const;

  // Transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  /// \brief Entity ID for sensor within Gazebo.
  ignition::gazebo::Entity entity_;

  /// \brief Timing
  ros::Duration control_period_ = ros::Duration(1, 0);

  /// \brief Interface loader
  std::shared_ptr<pluginlib::ClassLoader<
  ign_ros_control::IgnitionSystemInterface>>
  robot_hw_sim_loader_{nullptr};

  /// \brief Controller manager
  std::shared_ptr<controller_manager::ControllerManager>
  controller_manager_{nullptr};

  /// \brief Robot hardware
  boost::shared_ptr<ign_ros_control::IgnitionSystemInterface>
  robot_hw_{nullptr};

  /// \brief Last time the update method was called
  ros::Time last_update_sim_time_ros_;

  /// \brief ECM pointer
  ignition::gazebo::EntityComponentManager * ecm_{nullptr};

  // Node Handles
  ros::NodeHandle model_nh_; // namespaces to robot name

  /// \brief Robot description
  std::string robot_description_ = "robot_description";

  /// \brief ROS comm
  std::shared_ptr<ros::AsyncSpinner> async_ros_spin_;
};

//////////////////////////////////////////////////
std::map<std::string, ignition::gazebo::Entity>
IgnitionROSControlPluginPrivate::GetEnabledJoints(
    const ignition::gazebo::Entity & _entity,
    ignition::gazebo::EntityComponentManager & _ecm) const
{
  std::map<std::string, ignition::gazebo::Entity> output;

  std::vector<std::string> enabledJoints;

  // Get all available joints
  auto jointEntities = _ecm.ChildrenByComponents(_entity, ignition::gazebo::components::Joint());

  // Iterate over all joints and verify whether they can be enabled or not
  for (const auto & jointEntity : jointEntities) {
    const auto jointName = _ecm.Component<ignition::gazebo::components::Name>(
          jointEntity)->Data();

    // Make sure the joint type is supported, i.e. it has exactly one
    // actuated axis
    const auto * jointType = _ecm.Component<ignition::gazebo::components::JointType>(jointEntity);
    switch (jointType->Data()) {
    case sdf::JointType::PRISMATIC:
    case sdf::JointType::REVOLUTE:
    case sdf::JointType::CONTINUOUS:
    case sdf::JointType::GEARBOX:
    {
      // Supported joint type
      break;
    }
    case sdf::JointType::FIXED:
    {
      ROS_INFO(
            "[Ignition ROS Control] Fixed joint [%s] (Entity=%lu)] is skipped",
            jointName.c_str(), jointEntity);
      continue;
    }
    case sdf::JointType::REVOLUTE2:
    case sdf::JointType::SCREW:
    case sdf::JointType::BALL:
    case sdf::JointType::UNIVERSAL:
    {
      ROS_WARN(
            "[Ignition ROS Control] Joint [%s] (Entity=%lu)] is of unsupported type."
            " Only joints with a single axis are supported.",
            jointName.c_str(), jointEntity);
      continue;
    }
    default:
    {
      ROS_WARN(
            "[Ignition ROS Control] Joint [%s] (Entity=%lu)] is of unknown type",
            jointName.c_str(), jointEntity);
      continue;
    }
    }
    output[jointName] = jointEntity;
  }

  return output;
}

//////////////////////////////////////////////////
std::string IgnitionROSControlPluginPrivate::getURDF(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE("[Ignition ROS Control] ign_ros_control_plugin is waiting for model"
                    " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE("[Ignition ROS Control] ign_ros_control_plugin is waiting for model"
                    " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  ROS_INFO_STREAM("[Ignition ROS Control] Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Get Transmissions from the URDF
bool IgnitionROSControlPluginPrivate::parseTransmissionsFromURDF(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

//////////////////////////////////////////////////
IgnitionROSControlPlugin::IgnitionROSControlPlugin()
  : dataPtr(nullptr)
{
}

//////////////////////////////////////////////////
IgnitionROSControlPlugin::~IgnitionROSControlPlugin()
{
}

//////////////////////////////////////////////////
void IgnitionROSControlPlugin::Configure(
    const ignition::gazebo::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager &)
{
  // Get namespace for nodehandle
  std::string robot_namespace, robot_description, robot_hw_sim_type_str;
  int update_rate;
  if(_sdf->HasElement("robotNamespace"))
  {
    robot_namespace = _sdf->Get<std::string>("robotNamespace");
  }
  else
  {
    robot_namespace = ""; // default
  }
  // Get robot_description ROS param name
  if (_sdf->HasElement("robotParam"))
  {
    robot_description = _sdf->Get<std::string>("robotParam");
  }
  else
  {
    robot_description = "robot_description";
  }
  // Get the robot simulation interface type
  if(_sdf->HasElement("robotSimType"))
  {
    robot_hw_sim_type_str = _sdf->Get<std::string>("robotSimType");
  }
  else
  {
    robot_hw_sim_type_str = "ign_ros_control/IgnitionSystem";
    ROS_DEBUG_STREAM("[Ignition ROS Control] Using default plugin for RobotHWSim (none specified in URDF/SDF)\""<<robot_hw_sim_type_str<<"\"");
  }
  if(_sdf->HasElement("updateRate"))
  {
    update_rate = _sdf->Get<int>("updateRate");
  }
  else
  {
    ROS_ERROR("[Ignition ROS Control] No updateRate defined in the sdf file.");
    return;
  }

  // setup ros related
  std::string ros_node_name = "ign_ros_control_plugin";
  int argc = 1;
  char* arg0 = strdup(ros_node_name.c_str());
  char* argv[] = {arg0, 0};
  if (!ros::isInitialized())
    ros::init(argc,argv,ros_node_name,ros::init_options::NoSigintHandler);
  else
    ROS_ERROR("[Ignition ROS Control] Something other than this ign_ros_control_plugin started ros::init(...).");

  dataPtr = std::make_unique<IgnitionROSControlPluginPrivate>(robot_namespace,robot_description);

  // Make sure the controller is attached to a valid model
  const auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm)) {
    ROS_ERROR(
          "[Ignition ROS Control] Failed to initialize because [%s] (Entity=%lu)] is not a model."
          "Please make sure that Ignition ROS Control is attached to a valid model.",
          model.Name(_ecm).c_str(), _entity);
    return;
  }

  ROS_DEBUG_STREAM("[Ignition ROS Control] Setting up controller for [" <<
                   model.Name(_ecm) << "] (Entity=" << _entity << ")].");

  // Get list of enabled joints
  auto enabledJoints = this->dataPtr->GetEnabledJoints(
        _entity,
        _ecm);

  if (enabledJoints.size() == 0) {
    ROS_DEBUG_STREAM("[Ignition ROS Control] There are no available Joints.");
    return;
  }

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  const std::string urdf_string = this->dataPtr->getURDF(robot_description);
  if (!this->dataPtr->parseTransmissionsFromURDF(urdf_string))
  {
    ROS_ERROR("[Ignition ROS Control] Error parsing URDF in gazebo_ros_control plugin, plugin not active.\n");
    return;
  }

  // Load the RobotHWSim abstraction to interface the controllers with the gazebo model
  try
  {
    this->dataPtr->robot_hw_sim_loader_.reset
        (new pluginlib::ClassLoader<ign_ros_control::IgnitionSystemInterface>
         ("ign_ros_control",
          "ign_ros_control::IgnitionSystemInterface"));

    this->dataPtr->robot_hw_ = this->dataPtr->robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str);
    //urdf::Model urdf_model;
    //const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : nullptr;

    if(!this->dataPtr->robot_hw_->initSim(this->dataPtr->model_nh_, enabledJoints, _ecm, this->dataPtr->transmissions_, update_rate))
    {
      ROS_FATAL("[Ignition ROS Control] Could not initialize robot simulation interface.");
      return;
    }

    // Create the controller manager
    ROS_DEBUG_STREAM("[Ignition ROS Control] Loading controller_manager.");
    this->dataPtr->controller_manager_.reset
        (new controller_manager::ControllerManager(this->dataPtr->robot_hw_.get(), this->dataPtr->model_nh_));


  }
  catch(pluginlib::LibraryLoadException &ex)
  {
    ROS_FATAL_STREAM("[Ignition ROS Control] Failed to create robot simulation interface loader: "<<ex.what());
  }

  this->dataPtr->control_period_ = ros::Duration(ros::Rate(update_rate));

  this->dataPtr->entity_ = _entity;
}

//////////////////////////////////////////////////
void IgnitionROSControlPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    ignition::gazebo::EntityComponentManager & /*_ecm*/)
{
  static bool warned{false};
  if (!warned) {
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>( _info.dt ).count(); // Secs
    ros::Duration gazebo_period(dt);

    // Check the period against the simulation period
    if (this->dataPtr->control_period_.toSec() < dt) {
      ROS_ERROR_STREAM(
            "[Ignition ROS Control] Desired controller update period (" << this->dataPtr->control_period_.toSec() <<
            " s) is faster than the gazebo simulation period (" <<
            gazebo_period.toSec() << " s).");
    } else if (this->dataPtr->control_period_.toSec() > gazebo_period.toSec()) {
      ROS_WARN_STREAM(
            "[Ignition ROS Control] Desired controller update period (" << this->dataPtr->control_period_.toSec() <<
            " s) is slower than the gazebo simulation period (" <<
            gazebo_period.toSec() << " s).");
    }
    warned = true;
  }

  // Always set commands on joints, otherwise at low control frequencies the joints tremble
  // as they are updated at a fraction of gazebo sim time
  this->dataPtr->robot_hw_->write();
}

//////////////////////////////////////////////////
void IgnitionROSControlPlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    const ignition::gazebo::EntityComponentManager & /*_ecm*/)
{

  // Get the simulation time and period
  double sim_time = std::chrono::duration_cast<std::chrono::duration<double>>( _info.simTime ).count(); // Secs

  ros::Time sim_time_ros(sim_time);
  ros::Duration sim_period(sim_time_ros.toSec() - this->dataPtr->last_update_sim_time_ros_.toSec());

  if (sim_period.toSec() >= this->dataPtr->control_period_.toSec()) {
    this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
    this->dataPtr->robot_hw_->read();
    this->dataPtr->controller_manager_->update(sim_time_ros, sim_period);
  }

}
}  // namespace ign_ros_control

IGNITION_ADD_PLUGIN(
    ign_ros_control::IgnitionROSControlPlugin,
    ignition::gazebo::System,
    ign_ros_control::IgnitionROSControlPlugin::ISystemConfigure,
    ign_ros_control::IgnitionROSControlPlugin::ISystemPreUpdate,
    ign_ros_control::IgnitionROSControlPlugin::ISystemPostUpdate)
