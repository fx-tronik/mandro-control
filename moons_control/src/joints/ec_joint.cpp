#include <moons_control/moons_hardware_interface.hpp>

using namespace moons_control::hw_interface;

EtherCATJointControlInterface::EtherCATJointControlInterface(Controller *manager, int slave_no, ActuatorDataContainer &actuators_data) : JointControlInterface(slave_no, actuators_data)
{
  std::string name;
  int eep_man, eep_id, eep_rev;
  int obits, ibits, state, pdelay, hasdc;
  int activeports, configadr;
  manager->getStatus(slave_no, name, eep_man, eep_id, eep_rev, obits, ibits, state, pdelay, hasdc, activeports, configadr);
  std::stringstream ss;
  ss << name << "(" << configadr << ")";
  actuator.hardware_id_ = ss.str();

  ROS_INFO("Initialize EtherCATJoint (%d) %s(man:%x, id:%x, rev:%x, port:%x, addr:%x)", slave_no, name.c_str(), eep_man, eep_id, eep_rev, activeports, configadr);

  ROS_INFO("eep_id = %d, name = %s", eep_id, actuator.joint_name_.c_str());

  memset(&output, 0x00, sizeof(ServoOutput));
  // EtherCAT
  client = new ServoClient(*manager, slave_no, output);

  ROS_INFO("Initialize EtherCATJoint (reset)");
  client->reset();

  assert(client->checkHome() && "drive was not homed before launching ros");

  // get current positoin
  ROS_INFO("Initialize EtherCATJoint (readInputs)");
  // read inputs
  input = client->readInputs();
  // set current position as offset for further calculations
  initial_pos = client->readInputs().position_actual_value;
  // set desired mode for joint

  mode = client->change_mode(mode);

  //input.position_actual_value;
  ROS_INFO("                         (PositionActualValue %d)", input.position_actual_value);

  ROS_INFO("Initialize EtherCATJoint (set target position)");
  // set target position
  output.target_position = initial_pos;
  output.target_velocity = 0;
  ROS_WARN("target position = %08x, %d", output.target_position, output.target_position);

  actuator.pos_ = static_cast<int32>(input.position_actual_value); // set current encoder reading as initial position of the joint
  actuator.vel_cmd_ = actuator.pos_cmd_ = prev_vel_cmd = prev_pos_cmd = 0;
  actuator.vel_ = actuator.eff_ = 0;

  // servo on
  ROS_INFO("Initialize EtherCATJoint (servoOn)");
  if(actuator.joint_name_.compare("joint6")!=0)
  {
    client->servoOn();
    ROS_INFO("Initialize EtherCATJoint .. done");
  }
}

EtherCATJointControlInterface::~EtherCATJointControlInterface()
{
  ROS_INFO_STREAM_NAMED("moons", "~EtherCATJointControlInterface()");
  shutdown();
  delete (client);
}

void EtherCATJointControlInterface::shutdown()
{
  ROS_INFO_STREAM_NAMED("moons", actuator.joint_name_ + " shutdown()");
  client->printPDSStatus(input);
  client->printPDSOperation(input);
  client->reset();
  client->servoOff();
}

void EtherCATJointControlInterface::read()
{
  input = client->readInputs();

  if (input.error_code == 65329 or input.error_code == 65330)
  {
    ros::shutdown(); //Shutdown hw_interface after reaching any of the hardware limit sensors
  }

  actuator.pos_ = static_cast<int32_t>(input.position_actual_value);
  actuator.vel_ = static_cast<int32_t>(input.velocity_actual_value);
  actuator.eff_ = static_cast<int32_t>(input.torque_actual_value);
}

void EtherCATJointControlInterface::write() // FILL IN YOUR READ COMMAND TO ETHERCAT
{
  if (mode == 0x08)
  {
    // make sure that there was change in command before sending it to hardware
    // this fixes problems with sending empty target positions to drives before controller initializes
    if (!isnan(actuator.pos_cmd_) && prev_pos_cmd != actuator.pos_cmd_) 
    {
      output.target_position = static_cast<int32_t>(actuator.pos_cmd_);
      client->writeOutputs();
    }
    prev_pos_cmd = actuator.pos_cmd_;
  }
  else if (mode == 0x09)
  {
    if (!isnan(actuator.vel_cmd_) && prev_vel_cmd != actuator.vel_cmd_)
    {
      output.target_velocity = static_cast<int32_t>(actuator.vel_cmd_) ;
      client->writeOutputs();
    }
    prev_vel_cmd = actuator.vel_cmd_;
  }
}

bool EtherCATJointControlInterface::home_servo()
{
  return client->home();
}

void EtherCATJointControlInterface::set_mode(int controller_mode)
{
  mode = client->change_mode(controller_mode);
}
