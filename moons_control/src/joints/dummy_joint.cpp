#include <moons_control/moons_hardware_interface.hpp>

using namespace moons_control::hw_interface;

DummyJointControlInterface::DummyJointControlInterface(int slave_no, ActuatorDataContainer &actuators_data) : JointControlInterface(slave_no, actuators_data)
{
  actuator.pos_cmd_ = actuator.pos_ = actuator.vel_ = actuator.eff_ = 0;
  actuator.pos_ = 0;
  prev_vel_cmd =  prev_pos_cmd = 0;
}

void DummyJointControlInterface::read()
{

}

void DummyJointControlInterface::write()
{
  if(mode == 0x08)
  {
    if(!isnan(actuator.pos_cmd_) && actuator.pos_cmd_ != prev_pos_cmd)
    {
      actuator.pos_ = (actuator.pos_cmd_);
    }
    prev_pos_cmd = actuator.pos_cmd_;
  }
  else if(mode == 0x09 && actuator.joint_name_.compare("joint7") != 0)
  {
    ROS_ERROR_ONCE("Dummy joints do not support velocity mode simulation");
    ros::shutdown();
  }
}

bool DummyJointControlInterface::home_servo()
{
  return true; //Dummy Interfaces are always homed
}

void DummyJointControlInterface::set_mode(int controller_mode)
{
  mode = controller_mode;
}