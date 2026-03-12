#include <mandro_hw_interface/mandro_hardware_interface.hpp>

MandroHardwareInterface::MandroHardwareInterface(ros::NodeHandle &nh, ros::NodeHandle &robot_hw_nh, urdf::Model *urdf_model, std::string ifname, bool in_simulation)
    : manager(NULL), nh_(nh), robot_hw_nh_(robot_hw_nh), ifname_(ifname), sim_(in_simulation)
{

  ROS_FATAL_STREAM("Testujemy!");

  if (urdf_model == NULL)
    loadURDF(nh, "robot_description");
  else
    urdf_model_ = urdf_model;

  // Load rosparams
  std::size_t error = 0;
  error += !rosparam_shortcuts::get("hardware_interface", nh, "/hardware_interface/joints", joint_names_);
  rosparam_shortcuts::shutdownIfError("hardware_interface", error);

  try
  {
    transmission_loader_.reset(new transmission_interface::TransmissionInterfaceLoader(this, &robot_transmissions_));
  }
  catch (const std::invalid_argument &ex)
  {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    quit = true;
  }
  catch (const pluginlib::LibraryLoadException &ex)
  {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    quit = true;
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    quit = true;
  }
}

void MandroHardwareInterface::init()
{

   s_estop = robot_hw_nh_.advertiseService("hw_estop", &MandroHardwareInterface::SEstop, this);

  num_joints_ = joint_names_.size();

  actuators_data.resize(num_joints_ + 1);

  // Hardware Interface in simulation mode, create dummy joints
  if (sim_)
  {
    ROS_INFO_STREAM_NAMED("moons", "Moons Hardware Interface in simulation mode");
    for (int i = 1; i <= 7; i++)
    {
      registerControl(new DummyJointControlInterface(robot_hw_nh_, i, actuators_data));
    }
  }

  // We are working with actual hardware
  else
  {

    try
    {
      manager = new Controller(nh_, ifname_); // Create EtherCat Manager
    }
    catch (std::exception)
    {
      quit = true;
      return;
    }

    n_dof_ = manager->getNumClients();
    if (n_dof_ != 6)
    {
      ROS_ERROR_STREAM_NAMED("moons", "Moons Hardware Interface expecting 6 clients");
    }

    int i;
    for (i = 1; i <= n_dof_; i++)
    {
      // Create JointControlInterfaces for each drive
      if (i == 6) // Joint 6 is mimic
      {
        registerControl(new MimicEtherCATJointControlInterface(robot_hw_nh_, manager, i, actuators_data, 5));
      }
      else
      {
        registerControl(new EtherCATJointControlInterface(robot_hw_nh_, manager, i, actuators_data));
      }
    }
    // Dummy JointControlInterface - ROS doesnt work well without 6 degrees of freedom
    for (; i <= 7; i++)
    {
      registerControl(new DummyJointControlInterface(robot_hw_nh_, i, actuators_data));
    }
  }

  // Create state, position command and velocity command handles for hardware_interface
  for (int i = 1; i <= 7; i++)
  {
    ActuatorStateHandle act_state_handle(actuators_data[i].actuator_name_, &actuators_data[i].pos_, &actuators_data[i].vel_, &actuators_data[i].eff_);
    actuator_state_inteface.registerHandle(act_state_handle);

    ActuatorHandle a_cmd_handle(act_state_handle, &actuators_data[i].pos_cmd_);
    actuator_position_interface.registerHandle(a_cmd_handle);

    ActuatorHandle a_vel_handle(act_state_handle, &actuators_data[i].vel_cmd_);
    actuator_velocity_interface.registerHandle(a_vel_handle);
  }

  // Register actuator interfaces in hw
  registerInterface(&actuator_state_inteface);
  registerInterface(&actuator_position_interface);
  registerInterface(&actuator_velocity_interface);

  // Load transmission from urdf
  transmission_loader_->load(urdf_string_);

  // Check interfaces loaded from URDF
  for (auto var : robot_transmissions_.getNames())
  {
    if (var == "transmission_interface::JointToActuatorPositionInterface")
      hasJ2APosInterface = true;
    else if (var == "transmission_interface::JointToActuatorVelocityInterface")
      hasJ2AVelInterface = true;
    else if (var == "transmission_interface::ActuatorToJointStateInterface")
      hasA2JStateInterface = true;
  };

  // Fetch implicitly created state interfaces
  if (hasA2JStateInterface)
  {
    act_to_jnt_state = robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>();
    joint_state_interface = this->get<JointStateInterface>();
    act_to_jnt_state->propagate(); //propagate states once, we need joint position to check working space boundaries
  }

  // Fetch implicitly created position interfaces
  std::vector<std::string> jp;
  if (hasJ2APosInterface)
  {
    jnt_to_act_pos = robot_transmissions_.get<transmission_interface::JointToActuatorPositionInterface>();
    joint_position_interface = this->get<PositionJointInterface>();
    jp = joint_position_interface->getNames();
  }
  
  // Fetch implicitly created velocity interfaces
  std::vector<std::string> jv;
  if (hasJ2AVelInterface)
  {
    jnt_to_act_vel = robot_transmissions_.get<transmission_interface::JointToActuatorVelocityInterface>();
    joint_velocity_interface = this->get<VelocityJointInterface>();
    jv = joint_velocity_interface->getNames();
  }

  // Joint limits
  for (int i = 1; i <= 7; i++)
  {
    // Read limits from URDF model
    urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(actuators_data[i].joint_name_);
    if (getJointLimits(urdf_joint, actuators_data[i].joint_limits))
    {
      ROS_INFO_STREAM_NAMED("moons", "Joint " << actuators_data[i].joint_name_ << " has URDF position limits ["
                                              << actuators_data[i].joint_limits.min_position << ", "
                                              << actuators_data[i].joint_limits.max_position << "]");
      if (actuators_data[i].joint_limits.has_velocity_limits)
      {
        ROS_INFO_STREAM_NAMED("moons", "Joint " << actuators_data[i].joint_name_ << " has URDF velocity limit ["
                                                << actuators_data[i].joint_limits.max_velocity << "]");
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("moons", "Joint " << actuators_data[i].joint_name_ << " doesnt have velocity limit");
      }
    }
    // Check whether all joints are in working space
    double current_pos = joint_state_interface->getHandle(actuators_data[i].joint_name_).getPosition();
    if (actuators_data[i].joint_limits.min_position <= current_pos && actuators_data[i].joint_limits.max_position < current_pos)
    {
      ROS_ERROR_STREAM_ONCE_NAMED("moons", "Joint " << actuators_data[i].joint_name_ << " is outside of its position limits. Current pos " << current_pos);
      outside_bounds = true;
    };

    // Disable acceleration limits in hardware - it messes with max velocity calculation
    actuators_data[i].joint_limits.has_acceleration_limits = false;

    // Position Limit Interface
    if (hasJ2APosInterface)
    {
      // Create and register limit handles for joints in pos interface
      if (std::find(jp.begin(), jp.end(), actuators_data[i].joint_name_) != jp.end())
      {
        PositionJointSaturationHandle pos_limit_handle(joint_position_interface->getHandle(actuators_data[i].joint_name_), actuators_data[i].joint_limits);
        pos_joint_limits_interface.registerHandle(pos_limit_handle);
      }
    }

    // Velocity Limit Interface
    if (hasJ2AVelInterface)
    {
      // Create and register limit handles for joints in vel interface
      if (std::find(jv.begin(), jv.end(), actuators_data[i].joint_name_) != jv.end())
      {
        VelocityJointSaturationHandle vel_limit_handle(joint_velocity_interface->getHandle(actuators_data[i].joint_name_), actuators_data[i].joint_limits);
        vel_joint_limits_interface.registerHandle(vel_limit_handle);
      }
    }
  }

  // Register limit interfaces in hw
  if(hasJ2APosInterface)
    registerInterface(&pos_joint_limits_interface);
  if(hasJ2AVelInterface)
    registerInterface(&vel_joint_limits_interface);
};

MandroHardwareInterface::~MandroHardwareInterface()
{
  shutdown();
}

void MandroHardwareInterface::shutdown()
{
  for (JointControlInterface *control : controls)
  {
    control->shutdown();
  }

  controls.clear();
  if (manager != NULL)
  {
    ROS_INFO_STREAM_NAMED("moons", "Delete manager");
    delete (manager);
  }
  manager = NULL;
}

void MandroHardwareInterface::registerControl(JointControlInterface *control)
{
  controls.push_back(control);
}

void MandroHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
  for (JointControlInterface *control : controls)
  {
    control->read();
  }

  act_to_jnt_state->propagate();
}

void MandroHardwareInterface::write(const ros::Time time, const ros::Duration period)
{

  if (hasJ2APosInterface)
  {
    pos_joint_limits_interface.enforceLimits(period);
    jnt_to_act_pos->propagate();
  }

  if (hasJ2AVelInterface)
  {
    vel_joint_limits_interface.enforceLimits(period);
    jnt_to_act_vel->propagate();
  }

  for (JointControlInterface *control : controls)
  {
    control->write(period);
  }
}

bool MandroHardwareInterface::checkForConflict(const std::list<ControllerInfo> &info) const
{
  // TODO:checking for conflicts
  return false;
}

void MandroHardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
{
  // TODO:this only works when a single controller is being loaded at the time
  for (auto controller : start_list)
  {
    for (auto resource : controller.claimed_resources.front().resources)
    {
      for (auto control : controls)
      {
        if (control->getJointName() == resource)
        {
          if (controller.type == "position_controllers/JointTrajectoryController")
          {
            ROS_INFO("Setting JointControlInterface for %s in cyclic synchronous position mode (0x08)", control->getJointName().data());
            control->set_mode(0x08);
          }
          else if (controller.type == "velocity_controllers/JointTrajectoryController")
          {
            ROS_INFO("Setting JointControlInterface for %s in cyclic synchronous velocity mode (0x09)", control->getJointName().data());
            control->set_mode(0x09);
          }
          else if (controller.type == "position_controllers/GripperActionController")
          {
            ROS_INFO("Setting JointControlInterface for %s in cyclic position velocity mode (0x08)", control->getJointName().data());
            control->set_mode(0x08);
          }
        }
      }
    }
  }
}

inline ros::Time MandroHardwareInterface::getTime()
{
  return ros::Time::now();
}

inline ros::Duration MandroHardwareInterface::getPeriod()
{
  return ros::Duration(0.001);
}

void MandroHardwareInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
{
  std::string urdf_string;
  urdf_model_ = new urdf::Model();

  // search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok())
  {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name))
    {
      ROS_INFO_STREAM_NAMED("moons", "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace() << search_param_name);
      nh.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_STREAM_NAMED("moons", "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace() << param_name);
      nh.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }

  urdf_string_ = urdf_string;

  if (!urdf_model_->initString(urdf_string))
    ROS_ERROR_STREAM_NAMED("moons", "Unable to load URDF model");
  else
    ROS_DEBUG_STREAM_NAMED("moons", "Received URDF from param server");
}

int MandroHardwareInterface::Estop()
{

  bool result = false;
  for (JointControlInterface *control : controls)
  {
    estops.push_back(std::async(&JointControlInterface::estop, control));
  }

  for (std::future<int> &future : estops)
  {
    result += future.get();
  }

  if(result)
  {
    estops.clear();
    return result;
  }
  else
  {
    throw;
  }

}

bool MandroHardwareInterface::SEstop(mandro_hw_interface::estop::Request &req, mandro_hw_interface::estop::Response &res)
{
  switch (this->Estop())
  {
  case 1:
    res.message = "Sucessfully stopped drive";
    res.success = true;
    return true;
  case 2:
    res.message = "Servo is not enabled, quick stop is not required";
    res.success = false;
    return true;
  case 0:
    return true;
  }

  return false;
}