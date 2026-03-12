#include <mandro_hw_interface/mandro_hardware_interface.hpp>

MimicEtherCATJointControlInterface::MimicEtherCATJointControlInterface(ros::NodeHandle &robot_hw_nh, Controller *manager, int slave_no, ActuatorDataContainer &actuators_data, int offset_slave_no)
: EtherCATJointControlInterface(robot_hw_nh, manager, slave_no, actuators_data),
offset(actuators_data[offset_slave_no].pos_)
{ 
  client->ReadInputs();
  actuator.pos_ = client->GetCurrentPos() - offset;
  client->SetPosCommand(client->GetCurrentPos());
  client->WriteOutputs();
  client->ServoOn();
  prev_offset = prev_pos_cmd = 0;
  ROS_INFO("Initialize MimicEtherCATJoint .. done");
}

void MimicEtherCATJointControlInterface::read()
{
  client->ReadInputs();
  actuator.pos_ = client->GetCurrentPos() - offset;
  actuator.vel_ = client->GetCurrentVel();
  //TODO: effort reading, actuator.eff_ = static_cast<int32_t>(input.torque_actual_value);
}

void MimicEtherCATJointControlInterface::write(ros::Duration period)
{
  if (mode == 0x08)
  {
    if (!isnan(actuator.pos_cmd_) && (prev_pos_cmd != actuator.pos_cmd_ or prev_offset != offset))
    {
      client->SetPosCommand(actuator.pos_cmd_ + offset);
      
    }
    client->WriteOutputs();
    prev_pos_cmd = actuator.pos_cmd_;
    prev_offset = offset;
  }
  else if (mode == 0x09)
  {
    ROS_ERROR_ONCE("Mimic joints do not support velocity mode");
    ros::shutdown();
  }
}