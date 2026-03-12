#ifndef MOONS_HARDWARE_INTERFACE_H
#define MOONS_HARDWARE_INTERFACE_H

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ros_control
#include <controller_manager/controller_manager.h>

#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/robot_transmissions.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

// moons
#include <moons_control/ecat_controller.hpp>
#include <moons_control/servo_client.hpp>

namespace moons_control
{
namespace hw_interface
{

using namespace hardware_interface;
using namespace joint_limits_interface;

using moons_control::ethercat_controller::Controller;
using moons_control::servo::ServoClient;
using moons_control::servo::ServoInput;
using moons_control::servo::ServoOutput;

struct ActuatorData
{
  std::string joint_name_;
  std::string hardware_id_;
  std::string actuator_name_;
  double pos_cmd_;
  double vel_cmd_;
  double pos_;
  double vel_;
  double eff_;

  JointLimits joint_limits;
};

typedef std::vector<ActuatorData> ActuatorDataContainer;

class JointControlInterface
{
public:
  JointControlInterface(int slave_no, ActuatorDataContainer &actuators_data);
  ~JointControlInterface(){};

  std::string getJointName();
  std::string getActuatorName();

  virtual void read() = 0;
  virtual void write() = 0;
  virtual void shutdown() = 0;
  virtual bool home_servo() = 0;
  virtual void set_mode(int controller_mode) = 0;

protected:
  ActuatorData &actuator;
  double prev_vel_cmd;
  double prev_pos_cmd;
  int mode = 0x08;
};

class EtherCATJointControlInterface : public JointControlInterface
{
public:
  EtherCATJointControlInterface(Controller *manager, int slave_no, ActuatorDataContainer &actuators_data);
  ~EtherCATJointControlInterface();
  void read();
  void write();
  void shutdown();
  bool home_servo();
  void set_mode(int controller_mode);

protected:
  uint32 initial_pos;
  ServoClient *client;
  ServoInput input;
  ServoOutput output;
};

class MimicEtherCATJointControlInterface : public EtherCATJointControlInterface
{
public:
  MimicEtherCATJointControlInterface(Controller *manager, int slave_no, ActuatorDataContainer &actuators_data, int offset_slave_no);
  ~MimicEtherCATJointControlInterface();
  void read();
  void write();
  using EtherCATJointControlInterface::home_servo;
  using EtherCATJointControlInterface::set_mode;
  using EtherCATJointControlInterface::shutdown;

private:
  double &offset;
  double prev_offset;
};

class DummyJointControlInterface : public JointControlInterface
{
public:
  DummyJointControlInterface(int slave_no, ActuatorDataContainer &actuators_data);
  ~DummyJointControlInterface();
  void read();
  void write();
  void shutdown(){};
  bool home_servo();
  void set_mode(int controller_mode);
};

class MoonsHardwareInterface : public RobotHW
{
private:
  //Node Handles
  ros::NodeHandle nh_; // no namespace
  ros::NodeHandle robot_hw_nh_;

  // Configuration
  std::vector<std::string> joint_names_;
  std::size_t num_joints_;
  urdf::Model *urdf_model_;
  std::string urdf_string_;
  bool sim_;
  std::string ifname_;

  // Hardware interfaces
  JointStateInterface* joint_state_interface;
  VelocityJointInterface* joint_velocity_interface;
  PositionJointInterface* joint_position_interface;
  ActuatorStateInterface actuator_state_inteface;
  PositionActuatorInterface actuator_position_interface;
  VelocityActuatorInterface actuator_velocity_interface;

  // Joint limits interfaces - Saturation
  PositionJointSaturationInterface pos_joint_limits_interface;
  VelocityJointSaturationInterface vel_joint_limits_interface;

  //Transmission
  transmission_interface::RobotTransmissions robot_transmissions_;
  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_;
  transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state;
  transmission_interface::JointToActuatorPositionInterface* jnt_to_act_pos;
  transmission_interface::JointToActuatorVelocityInterface* jnt_to_act_vel;

  bool hasA2JStateInterface=false;
  bool hasJ2AVelInterface=false;
  bool hasJ2APosInterface=false;

  // Kinematic properties
  unsigned int n_dof_;

  typedef std::vector<JointControlInterface *> JointControlContainer;
  JointControlContainer controls;

  // Actuator space states, commands
  ActuatorDataContainer actuators_data;

  // Ethercat Manager
  Controller *manager;

public:
  /**
   * \brief Constructor/Descructor
   */
  MoonsHardwareInterface(ros::NodeHandle &nh, ros::NodeHandle &robot_hw_nh, urdf::Model *urdf_model, std::string ifname, bool in_simulation);
  ~MoonsHardwareInterface();
  void init();
  void registerControl(JointControlInterface *);
  void read(const ros::Time time, const ros::Duration period);
  void write(const ros::Time time, const ros::Duration period);
  void shutdown();
  void loadURDF(ros::NodeHandle &nh, std::string param_name);
  bool checkForConflict(const std::list<ControllerInfo> &info) const;
  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
  ros::Time getTime();
  ros::Duration getPeriod();

  bool quit = false;
};

} // namespace hw_interface
} // namespace moons_control

#endif