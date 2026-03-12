#include <mandro_hw_interface/mandro_hardware_interface.hpp>

EtherCATJointControlInterface::EtherCATJointControlInterface(ros::NodeHandle &robot_hw_nh, Controller *manager, int slave_no, ActuatorDataContainer &actuators_data) : JointControlInterface(slave_no, actuators_data)
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

  // EtherCAT
  client = new MoonsServo(robot_hw_nh, *manager, slave_no);

  ROS_INFO("Initialize EtherCATJoint (reset)");
  client->Reset();

  assert(client->CheckHome() && "drive was not homed before launching ros");

  // get current positoin
  ROS_INFO("Initialize EtherCATJoint (readInputs)");
  // read inputs
  client->ReadInputs();
  // set current position as offset for further calculations
  initial_pos = client->GetCurrentPos();
  // set desired mode for joint

  mode = client->ChangeMode(mode);

  //input.position_actual_value;
  ROS_INFO("                         (PositionActualValue %f)", client->GetCurrentPos());

  ROS_INFO("Initialize EtherCATJoint (set target position)");
  // set target position
  client->SetPosCommand(initial_pos);
  client->SetVelCommand(0);
  client->WriteOutputs();
  ROS_WARN("target position = %f", initial_pos);

  actuator.pos_ = initial_pos; // set current encoder reading as initial position of the joint
  actuator.vel_cmd_ = actuator.pos_cmd_ = prev_vel_cmd = prev_pos_cmd = 0;
  actuator.vel_ = actuator.eff_ = 0;

  // servo on
  ROS_INFO("Initialize EtherCATJoint (servoOn)");
  if(actuator.joint_name_.compare("joint6")!=0)
  {
    client->ServoOn();
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
  client->Reset();
  client->ServoOff();
}

void EtherCATJointControlInterface::read()
{
  client->ReadInputs();

  actuator.pos_ = client->GetCurrentPos(); 
  actuator.vel_ = client->GetCurrentVel();

  //TODO: effort reading, actuator.eff_ = static_cast<int32_t>(input.torque_actual_value);
}

void EtherCATJointControlInterface::write(ros::Duration period) // FILL IN YOUR READ COMMAND TO ETHERCAT
{
  if (mode == 0x08)
  {
    // make sure that there was change in command before sending it to hardware
    // this fixes problems with sending empty target positions to drives before controller initializes
    if (!isnan(actuator.pos_cmd_) && prev_pos_cmd != actuator.pos_cmd_) 
    {
      client->SetPosCommand(actuator.pos_cmd_);
      
    }
    client->WriteOutputs();
    prev_pos_cmd = actuator.pos_cmd_;
  }
  else if (mode == 0x09)
  {
    if (!isnan(actuator.vel_cmd_) && prev_vel_cmd != actuator.vel_cmd_)
    {
      client->SetVelCommand(actuator.vel_cmd_);
      
    }
    client->WriteOutputs();
    prev_vel_cmd = actuator.vel_cmd_;
  }
}

void EtherCATJointControlInterface::set_mode(int controller_mode)
{
  mode = client->ChangeMode(static_cast<MoonsServo::OperationMode>(controller_mode));
}

int EtherCATJointControlInterface::estop()
{
  return client->EStop();
}
