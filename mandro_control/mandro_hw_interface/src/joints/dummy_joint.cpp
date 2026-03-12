#include <mandro_hw_interface/mandro_hardware_interface.hpp>

DummyJointControlInterface::DummyJointControlInterface(ros::NodeHandle &robot_hw_nh, int slave_no, ActuatorDataContainer &actuators_data) : JointControlInterface(slave_no, actuators_data)
{
  actuator.pos_cmd_ = actuator.pos_ = actuator.vel_ = actuator.eff_ = 0;
  actuator.pos_ = 0;
  prev_vel_cmd =  prev_pos_cmd = 0;
}

void DummyJointControlInterface::read()
{

}

void DummyJointControlInterface::write(ros::Duration period)
{
  if(mode == 0x08)
  {
    if(!isnan(actuator.pos_cmd_) && actuator.pos_cmd_ != prev_pos_cmd)
    {
      actuator.pos_ = actuator.pos_cmd_;
    }
    prev_pos_cmd = actuator.pos_cmd_;
  }
  else if(mode == 0x09 && actuator.joint_name_.compare("joint7") != 0)
  {
    actuator.vel_ = actuator.vel_cmd_;
    actuator.pos_ += actuator.vel_cmd_ * period.toSec();
  }
}

void DummyJointControlInterface::set_mode(int controller_mode)
{
  mode = static_cast<MoonsServo::OperationMode>(controller_mode);
  ROS_INFO("[dummy_servo]Sucesfully changed mode to %d", controller_mode);
}

int DummyJointControlInterface::estop()
{
  ROS_INFO("[dummy_servo] estop active");
  return true;
}