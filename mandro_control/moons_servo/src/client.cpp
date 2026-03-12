#include <moons_servo/client.hpp>

using boost::accumulators::accumulator_set;
using boost::accumulators::extract_result;
using boost::accumulators::stats;
using boost::accumulators::tag::max;
using boost::accumulators::tag::mean;
using boost::accumulators::tag::count;

static constexpr unsigned SLEEP_TIME_MS = 1; // 1 ms
static constexpr int SEC_2_NSEC = 1e+9;
static constexpr int SEC_2_USEC = 1e+6;

static struct
{
  accumulator_set<double, stats<max, mean, count>> current_acc;
  int error_code;
} g_stats;

MoonsServo::MoonsServo(ros::NodeHandle &nh, Controller &controller, int slave_no)
    : nh_(nh), controller_(controller), slave_no_(slave_no)
{

  // create joint name
  std::stringstream ss;
  ss << "servo_" << slave_no;
  servo_name = ss.str();

  //preallocate memory
  memset(&input_, 0x00, sizeof(ServoInput));
  memset(&output_, 0x00, sizeof(ServoOutput));
  input_map_.fill(0x00);
  read_output_map_.fill(0x00);
  write_output_map_.fill(0x00);
  diagnostic_thread = std::thread(&MoonsServo::DiagnosticLoop, this);
  in_rt_loop = false;

  int eep_man;
  int eep_id;
  int eep_rev;

  controller.getMan(slave_no_, eep_man, eep_id, eep_rev);

  if (eep_man == 0x168 && eep_id == 3 && eep_rev == 1) // TODO: Checking servo type to determine pulses per revolution. This should happen in controller
  {
    pulse_per_rev_ = 10000;
    ROS_INFO("[%s] pulse_per_rev set to %f", servo_name.data(), pulse_per_rev_);
  }
  else if (eep_man == 0x168 && eep_id == 1 && eep_rev == 1)
  {
    pulse_per_rev_ = 20000;
    ROS_INFO("[%s] pulse_per_rev set to %f", servo_name.data(), pulse_per_rev_);
  }
  else
  {
    ROS_INFO("man %d, id %d, rev %d", eep_man, eep_id, eep_rev);
    throw;
  }

  ROS_INFO_STREAM("Registering " << servo_name << " services");
  s_home = nh.advertiseService(servo_name + "_home", &MoonsServo::SHome, this);
  s_reset = nh.advertiseService(servo_name + "_reset", &MoonsServo::SReset, this);
  s_go2pos = nh.advertiseService(servo_name + "_go2pos", &MoonsServo::SGo2Pos, this);
  s_estop = nh.advertiseService(servo_name + "_estop", &MoonsServo::SEstop, this);
}

MoonsServo::~MoonsServo()
{
  ServoOff();
  diagnostic_thread.join();
}

void MoonsServo::DiagnosticLoop()
{

  publisher = new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray>(nh_, "/diagnostics", 2);
  int i=0;

  while (ros::ok())
  {
    if (input_.error_code >> 8 == 0xff)
    {
      ecode = (input_.error_code) & 0x00ff;
      //ecode = 54;
      it = mapping.find(ecode);
      if (it != mapping.end())
      {
        if (ecode < 0x9f)
          ROS_WARN_THROTTLE_NAMED(5, "servodrive", "[%s]ALARM : %d %s", servo_name.data(), ecode, it->second.data());
        else
          ROS_ERROR_THROTTLE_NAMED(5, "serodrive", "[%s]ERROR : %d %s", servo_name.data(), ecode, it->second.data());
      }
      else if (it == mapping.end())
      {
        ROS_FATAL_THROTTLE_NAMED(5, "servodrive", "[%s]FATAL: %d UNKNOWN ERROR", servo_name.data(), ecode);
      }
    }
    else
    {
      ecode = 0;
    }
    
    publishDiagnostics(*publisher);


    usleep(1000 * SLEEP_TIME_MS * 1000);
  }
  publisher->stop();
  delete publisher;
}

const ServoOutput &MoonsServo::WriteOutputsToBuffer()
{

  write_output_map_[0] = (output_.controlword) & 0x00ff;
  write_output_map_[1] = (output_.controlword >> 8) & 0x00ff;
  write_output_map_[2] = output_.operation_mode;
  write_output_map_[3] = (output_.target_position) & 0x00ff;
  write_output_map_[4] = (output_.target_position >> 8) & 0x00ff;
  write_output_map_[5] = (output_.target_position >> 16) & 0x00ff;
  write_output_map_[6] = (output_.target_position >> 24) & 0x00ff;
  write_output_map_[7] = (output_.target_velocity) & 0x00ff;
  write_output_map_[8] = (output_.target_velocity >> 8) & 0x00ff;
  write_output_map_[9] = (output_.target_velocity >> 16) & 0x00ff;
  write_output_map_[10] = (output_.target_velocity >> 24) & 0x00ff;

  for (auto i = 0; i <= write_output_map_.max_size(); ++i)
  {
    controller_.write(slave_no_, i, write_output_map_[i]);
  } 
  
  return output_;
}

const ServoInput &MoonsServo::ReadInputsFromBuffer()
{
  
  for (int i = 0; i <= input_map_.max_size(); ++i)
  {
    input_map_[i] = controller_.readInput(slave_no_, i);
  } 
  
  input_.error_code = *reinterpret_cast<uint16_t *>(input_map_.data() + 0);
  input_.statusword = *reinterpret_cast<uint16_t *>(input_map_.data() + 2);
  input_.operation_mode = *reinterpret_cast<uint8_t *>(input_map_.data() + 4);
  input_.position_actual_value = *reinterpret_cast<uint32_t *>(input_map_.data() + 5);
  input_.follow_error_value = *reinterpret_cast<uint32_t *>(input_map_.data() + 9);
  input_.velocity_actual_value = *reinterpret_cast<uint32_t *>(input_map_.data() + 13);
  input_.torque_actual_value = *reinterpret_cast<uint16_t *>(input_map_.data() + 17);
  input_.alarm_code = *reinterpret_cast<uint32_t *>(input_map_.data() + 19);
  input_.current_actual_value = *reinterpret_cast<uint16_t *>(input_map_.data() + 23);
 
  return input_;
}

bool MoonsServo::SReset(moons_servo::reset::Request &req, moons_servo::reset::Response &res)
{

  if (in_rt_loop)
  {
    res.message = "Async services are not available while in RT loop";
    res.success = false;
    return true;
  }

  switch (Reset())
  {
  case 0:
    res.success = false;
    res.message = "Couldnt reset fault in the drive";
    return true;
  case 1: 
    res.success = true;
    res.message = "No error in servodrive, skipping fault reset";
    return true;
  case 2:
    res.success = true;
    res.message = "Fault cleared";
    return true;
  }

  return false;
}

bool MoonsServo::SHome(moons_servo::home::Request &req, moons_servo::home::Response &res)
{
  if (in_rt_loop)
  {
    res.message = "Async services are not available while in RT loop";
    res.success = false;
    return true;
  }

  switch(Home())
  {
  case 0:
    res.success = false;
    res.message = "Couldnt home drives";
    return true;
  case 1:
    res.success = true;
    res.message = "Drive Homed";
    return true;
  }

  return false;
}

bool MoonsServo::SGo2Pos(moons_servo::go2pos::Request &req, moons_servo::go2pos::Response &res)
{
  if (in_rt_loop)
  {
    res.message = "Async services are not available while in RT loop";
    res.success = false;
    return true;
  }

  return false;
}

bool MoonsServo::SEstop(moons_servo::estop::Request &req, moons_servo::estop::Response &res)
{
  switch (EStop())
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

int MoonsServo::Reset()
{
  in_rt_loop = false;
  uint16_t cw_tmp = output_.controlword; //create copy of outputs before overwriting

  if (ReadInputsFromBuffer().error_code == 0)
  {
    ROS_INFO_NAMED("servodrive", "[%s]No error in servodrive, skipping fault reset", servo_name.data());
    return 1;
  }

  ROS_INFO_NAMED("servodrive", "[%s]Writing fault reset command",servo_name.data());
  output_.controlword = 0x0080; // fault reset
  WriteOutputsToBuffer();

  int chk = 2000;
  while (ReadInputsFromBuffer().error_code != 0 && chk-- && ros::ok())
  {
    ROS_WARN_THROTTLE_NAMED(4, "servodrive", "[%s]error_code = %04x, status_word %04x, operation_mode = %2d, position = %08x", servo_name.data(), input_.error_code, input_.statusword, input_.operation_mode, input_.position_actual_value);
    ROS_WARN_THROTTLE_NAMED(2, "servodrive", "[%s]Waiting for Fault Reset...", servo_name.data());
    output_.controlword = 0x0080; // fault reset
    WriteOutputsToBuffer();
    usleep(SLEEP_TIME_MS * 5 * 1000);
  }

  output_.controlword = cw_tmp; //restore servo outputs

  if (ReadInputsFromBuffer().error_code != 0)
  {
    ROS_ERROR_NAMED("servodrive", "[%s]Couldnt reset fault in the drive", servo_name.data());
    return 0;
  }
  else
  {
    ROS_INFO_NAMED("servodrive", "[%s]Fault was cleared", servo_name.data());
    return 2;
  }
}

int MoonsServo::ServoOn()
{
  int loop = 0;
  int chk = 500;

  while (getState(ReadInputsFromBuffer()) != InternalState::Operation_Enable && ros::ok() && chk--)
  {
    switch (getState(input_))
    {
    case InternalState::Switch_On_Disabled:
      output_.controlword = 0x0006; // move to ready to switch on
      break;
    case InternalState::Ready_To_Switch_On:
      output_.controlword = 0x0007; // move to switched on
      break;
    case InternalState::Switched_On:
      output_.controlword = 0x000f; // move to operation enabled
      break;
    case InternalState::Operation_Enable:
      break;
    default:
      ROS_ERROR_NAMED("servodrive", "[%s]Did not manage to enable servodrive (uknown status)", servo_name.data());
      return 0;
    }
    WriteOutputsToBuffer();
    usleep(SLEEP_TIME_MS * 1000);
  }

  if (getState(ReadInputsFromBuffer()) != InternalState::Operation_Enable)
  {
    ROS_ERROR_NAMED("servodrive", "[%s]Did not manage to enable servodrive (too many attempts)", servo_name.data());
    return 0;
  }
  else
  {
    ROS_INFO_NAMED("servodrive", "[%s]Servodrive enabled", servo_name.data());
    return 1;
  }
}

int MoonsServo::ServoOff()
{
  int chk = 50;
  while (not(getState(ReadInputsFromBuffer()) == InternalState::Ready_To_Switch_On || getState(input_) == InternalState::Switch_On_Disabled) && chk--)
  {
    switch (getState(input_))
    {
    case InternalState::Ready_To_Switch_On:
      //output.controlword = 0x0000; // disable voltage
      //not implemented
      break;
    case InternalState::Switched_On:
      output_.controlword = 0x0006; // shutdown
      break;
    case InternalState::Operation_Enable:
      output_.controlword = 0x0007; // disable operation
      break;
    case InternalState::Switch_On_Disabled:
      //output.controlword = 0x0000; // disable voltage
      //not implemented
      break;
    default:
      ROS_ERROR_NAMED("servodrive", "[%s]Did not manage to disable servodrive (uknown status)", servo_name.data());
      output_.controlword = 0x0006; // shutdown
      return false;
    }
    WriteOutputsToBuffer();
    usleep(SLEEP_TIME_MS * 1000);
  }
  if (not(getState(ReadInputsFromBuffer()) == InternalState::Ready_To_Switch_On || getState(input_) == InternalState::Switch_On_Disabled))
  {
    ROS_ERROR_NAMED("servodrive", "[%s]Did not manage to disable servodrive (too many attempts)", servo_name.data());
    return false;
  }
  else
  {
    ROS_INFO_NAMED("servodrive", "[%s]Servo disabled", servo_name.data());
    return true;
  }
}

int MoonsServo::EStop()
{
  in_rt_loop = false;
  int chk = 500;

  if(!(getState(ReadInputsFromBuffer()) == InternalState::Operation_Enable))
  {
    ROS_INFO_NAMED("servodrive", "[%s]Servo is not enabled, quick stop is not required", servo_name.data());
    return 2;
  }

  while (not(getState(ReadInputsFromBuffer()) == InternalState::Quick_Stop_Active) && chk--)
  {
    output_.controlword = 0x0002; // quickstop
    WriteOutputsToBuffer();
    usleep(SLEEP_TIME_MS * 1000);
    ROS_INFO_THROTTLE(1, "current state: %d", static_cast<uint8_t>(getState(ReadInputsFromBuffer())));
  }

  if (not(getState(ReadInputsFromBuffer()) == InternalState::Quick_Stop_Active))
  {
    ROS_FATAL_NAMED("servodrive", "[%s]Did not manage to quick stop servodrive (too many attempts)", servo_name.data()); // Should this ever happen?
    ROS_FATAL_STREAM_NAMED("servodrive", "Killing hw_interface");
    throw; // Just throw at this point
  }
  else
  {
    ROS_INFO_NAMED("servodrive", "[%s]Quick stop active", servo_name.data());
    return 1;
  }
}

MoonsServo::OperationMode MoonsServo::ChangeMode(MoonsServo::OperationMode mode)
{

  output_.operation_mode = mode;
  WriteOutputsToBuffer();

  int chk = 500;
  while (ReadInputsFromBuffer().operation_mode != mode && chk-- && ros::ok())
  {
    ROS_INFO_THROTTLE_NAMED(1, "servodrive", "[%s]Changing servo mode to %d", servo_name.data(), mode);
    output_.operation_mode = mode;
    WriteOutputsToBuffer();
    usleep(SLEEP_TIME_MS * 1000);
  }

  if (ReadInputsFromBuffer().operation_mode == mode)
  {
    ROS_INFO_NAMED("servodrive", "[%s]Sucesfully changed mode to %d", servo_name.data(), mode);
    return mode;
  }
  else
  {
    ROS_ERROR_NAMED("servodrive", "[%s]Could not change servo mode", servo_name.data());
    return OperationMode::No_Mode;
  }
}

int MoonsServo::Home()
{
  //FIXME:Placeholder function from old moons_control
  if(!ChangeMode(OperationMode::Homing) == OperationMode::Homing)
  {
    return 0;
  }

  if (!ServoOn())
  {
    return 0;
  }

  output_.controlword = 0x1F;
  WriteOutputsToBuffer();

  while (!((ReadInputsFromBuffer().statusword & 0x1000) and (ReadInputsFromBuffer().statusword & 0x0400)))
  {}

  output_.controlword = 0x0F;
  WriteOutputsToBuffer();

  return 1;
}

int MoonsServo::CheckHome()
{
  return true;
}

int MoonsServo::Go2Pos(uint32_t position)
{
  //FIXME:Placeholder function from old moons_control
  if(!ChangeMode(OperationMode::Profiled_Position) == OperationMode::Profiled_Position)
  {
    return 0;
  }

  if (!ServoOn())
  {
    return 0;
  }

  output_.target_position = position; // set set-point to 0
  output_.controlword = 0x01F; // halt drive
  WriteOutputsToBuffer();

  ROS_INFO_STREAM_NAMED("servodrive", "Setting new setpoint: " << static_cast<int32>(position));

  while(!(ReadInputsFromBuffer().statusword & 0x1000)) // wait for drive to accept new setpoint
  {}

  output_.controlword = 0x0F; // execute set-point
  WriteOutputsToBuffer();

  while(!(ReadInputsFromBuffer().statusword & 0x0400)) // wait untill set-point is reached
  {
    ROS_INFO_THROTTLE_NAMED(0.5, "servodrive", "[%s]Target position: %d | current position: %d ", servo_name.data(), static_cast<int32>(position), static_cast<int32>(input_.position_actual_value));
    usleep(SLEEP_TIME_MS * 1000);
  }

  return true;
}

MoonsServo::InternalState MoonsServo::getState(const ServoInput input)
{

  static constexpr uint16_t r = (1 << StatusWord::SW_Ready_To_Switch_On);
  static constexpr uint16_t s = (1 << StatusWord::SW_Switched_On);
  static constexpr uint16_t o = (1 << StatusWord::SW_Operation_enabled);
  static constexpr uint16_t f = (1 << StatusWord::SW_Fault);
  static constexpr uint16_t q = (1 << StatusWord::SW_Quick_stop);
  static constexpr uint16_t d = (1 << StatusWord::SW_Switch_on_disabled);

  InternalState new_state = InternalState::Unknown;

  uint16_t state = input.statusword & (d | q | f | o | s | r);
  switch (state)
  {
  //   ( d | q | f | o | s | r ):
  case (0 | 0 | 0 | 0 | 0 | 0):
  case (0 | q | 0 | 0 | 0 | 0):
    new_state = InternalState::Not_Ready_To_Switch_On;
    break;

  case (d | 0 | 0 | 0 | 0 | 0):
  case (d | q | 0 | 0 | 0 | 0):
    new_state = InternalState::Switch_On_Disabled;
    break;

  case (0 | q | 0 | 0 | 0 | r):
    new_state = InternalState::Ready_To_Switch_On;
    break;

  case (0 | q | 0 | 0 | s | r):
    new_state = InternalState::Switched_On;
    break;

  case (0 | q | 0 | o | s | r):
    new_state = InternalState::Operation_Enable;
    break;

  case (0 | 0 | 0 | o | s | r):
    new_state = InternalState::Quick_Stop_Active;
    break;

  case (0 | 0 | f | o | s | r):
  case (0 | q | f | o | s | r):
    new_state = InternalState::Fault_Reaction_Active;
    break;

  case (0 | 0 | f | 0 | 0 | 0):
  case (0 | q | f | 0 | 0 | 0):
    new_state = InternalState::Fault;
    break;

  default:
    ROS_ERROR_STREAM_NAMED("servodrive", "Servo in an unknown state");
  }

  return new_state;
}

void MoonsServo::publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher)
{
  if (publisher.trylock())
  {
    accumulator_set<double, stats<max, mean, count>> zero;
    std::vector<diagnostic_msgs::DiagnosticStatus> statuses;

    diagnostic_updater::DiagnosticStatusWrapper status;

    static double max_current = 0;
    double avg_current;

    avg_current = extract_result<mean>(g_stats.current_acc);

    max_current = std::max(max_current, extract_result<max>(g_stats.current_acc));
    //g_stats.current_acc = zero;

    status.addf("Max ServoDrive current (A)", "%.2f", max_current);
    status.addf("Avg ServoDrive current (A)", "%.2f", avg_current);
    status.addf("Current error code", "%d", it != mapping.end() ? it->first : 0);
    status.addf("Error message", "%s", it != mapping.end() ? it->second.data() : " ");

    status.name = servo_name + " diagnostics";

    if (ecode < 0x9f && ecode !=0)
    {
      status.level = 1;
      status.message = "Alarms in servodrive";
    }
    else if (ecode != 0)
    {
      status.level = 2;
      status.message = "Errors in servodrive";
    }
    else
    {
      status.level = 0;
      status.message = "Servo OK";
    }
    

    statuses.push_back(status);
    publisher.msg_.status = statuses;
    publisher.msg_.header.stamp = ros::Time::now();
    publisher.unlockAndPublish();
  }
}