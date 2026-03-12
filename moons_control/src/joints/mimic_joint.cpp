#include <moons_control/moons_hardware_interface.hpp>

using namespace moons_control::hw_interface;

MimicEtherCATJointControlInterface::MimicEtherCATJointControlInterface(Controller *manager, int slave_no, ActuatorDataContainer &actuators_data, int offset_slave_no)
: EtherCATJointControlInterface(manager, slave_no, actuators_data),
offset(actuators_data[offset_slave_no].pos_)
{ 
  actuator.pos_ = static_cast<int32_t>(input.position_actual_value - static_cast<int32_t>(offset));
  output.target_position = static_cast<int32_t>(input.position_actual_value);
  client->writeOutputs();
  client->servoOn();
  prev_offset = prev_pos_cmd = 0;
  ROS_INFO("Initialize MimicEtherCATJoint .. done");
}

void MimicEtherCATJointControlInterface::read()
{
  input = client->readInputs();

  if (input.error_code == 65329 or input.error_code == 65330)
  {
    ros::shutdown(); //Shutdown hw_interface after reaching any of the hardware limit sensors
  }

  actuator.pos_ = static_cast<int32_t>(input.position_actual_value - static_cast<int32_t>(offset));
  actuator.vel_ = static_cast<int32_t>(input.velocity_actual_value);
  actuator.eff_ = static_cast<int32_t>(input.torque_actual_value);
}

void MimicEtherCATJointControlInterface::write()
{
  if (mode == 0x08)
  {
    if (!isnan(actuator.pos_cmd_) && (prev_pos_cmd != actuator.pos_cmd_ or prev_offset != offset))
    {
      output.target_position = static_cast<int32_t>(actuator.pos_cmd_  + static_cast<int32_t>(offset));
      client->writeOutputs();
    }
    prev_pos_cmd = actuator.pos_cmd_;
    prev_offset = offset;
  }
  else if (mode == 0x09)
  {
    ROS_ERROR_ONCE("Mimic joints do not support velocity mode");
    ros::shutdown();
  }
  client->writeOutputs();
}