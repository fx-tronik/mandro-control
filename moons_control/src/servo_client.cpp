
#include <moons_control/servo_client.hpp>

#include <vector>
#include <stdio.h>
#include <sys/mman.h>

using namespace moons_control::servo;

static const unsigned SLEEP_TIME_MS = 1; // 1 ms

ServoClient::ServoClient(Controller &manager, int slave_no, ServoOutput &output)
    : manager_(manager)
    , slave_no_(slave_no)
    , output(output)
{
}

ServoClient::~ServoClient()
{
  //servoOff();
  printf("Servo deleted\n");
}

void ServoClient::writeOutputs()
{
  // PDO Mapping: 88 bit = 11  byte
  uint8_t map[11] = {0}; // array containing all 4 output registers

  map[0] = (output.controlword) & 0x00ff;
  map[1] = (output.controlword >> 8) & 0x00ff;
  map[2] = output.operation_mode;
  map[3] = (output.target_position) & 0x00ff;
  map[4] = (output.target_position >> 8) & 0x00ff;
  map[5] = (output.target_position >> 16) & 0x00ff;
  map[6] = (output.target_position >> 24) & 0x00ff;
  map[7] = (output.target_velocity) & 0x00ff;
  map[8] = (output.target_velocity >> 8) & 0x00ff;
  map[9] = (output.target_velocity >> 16) & 0x00ff;
  map[10] = (output.target_velocity >> 24) & 0x00ff;

  for (unsigned i = 0; i < 11; ++i)
  {
    manager_.write(slave_no_, i, map[i]);
  }
}

ServoInput ServoClient::readInputs() const
{
  ServoInput input;
  uint8_t map[24];
  for (unsigned i = 0; i < 24; ++i)
  {
    map[i] = manager_.readInput(slave_no_, i);
  }

  input.error_code = *(uint16 *)(map + 0);
  input.statusword = *(uint16 *)(map + 2);
  input.operation_mode = *(uint8 *)(map + 4);
  input.position_actual_value = *(uint32 *)(map + 5);
  input.follow_error_value = *(uint32 *)(map + 9);
  input.velocity_actual_value = *(uint32 *)(map + 13);
  input.torque_actual_value = *(uint16 *)(map + 17);
  input.alarm_code = *(uint32 *)(map + 19);
  input.current_actual_value = *(uint16 *)(map + 23);

  if (input.error_code >> 8 == 0xff)
  {
    int ecode = (input.error_code) & 0x00ff;
    printf("%s : %d ", (ecode < 0x9f) ? "ALARM" : "ERROR", ecode);
    for (unsigned i = 0; i < sizeof(error_map) / sizeof(error_map[0]); i++)
    {
      if (error_map[i].code == 99 || error_map[i].code == ecode)
      {
        printf("%s", error_map[i].text);
        break;
      }
    }
    printf("\n");
  }
  return input;
}

void ServoClient::readOutputs() const
{

  uint8_t map[10];
  for (unsigned i = 0; i < 10; ++i)
  {
    map[i] = manager_.readOutput(slave_no_, i);
  }

  output.controlword = *(uint16 *)(map + 0);
  output.operation_mode = *(uint8 *)(map + 2);
  output.target_position = *(uint32 *)(map + 3);
  output.target_velocity = *(uint32 *)(map + 7);
}

void ServoClient::reset()
{
  ServoInput input = readInputs();

  ServoOutput output_temp = output;

  if (input.error_code == 0)
    return;

  output.controlword = 0x0080; // fault reset
  writeOutputs();


  int loop = 0;
  int chk = 10;
  while (input.error_code != 0 && chk--)
  {
    input = readInputs();
    if (loop++ % 100 == 1)
    {
      printf("error_code = %04x, status_word %04x, operation_mode = %2d, position = %08x\n",
             input.error_code, input.statusword, input.operation_mode, input.position_actual_value);
      printf("Waiting for Fault Reset...\n");
    }
    output.controlword = 0x0080; // fault reset
    writeOutputs();
    usleep(SLEEP_TIME_MS * 5 * 1000);
  }
  printf("Fault was cleared\n");
  output = output_temp;
}

void ServoClient::servoOn()
{
  ServoInput input = readInputs();
  printPDSStatus(input);
  int loop = 0;
  int chk = 10;
  while (getPDSStatus(input) != OPERATION_ENABLED && chk--)
  {
    switch (getPDSStatus(input))
    {
    case SWITCH_DISABLED:
      output.controlword = 0x0006; // move to ready to switch on
      break;
    case READY_SWITCH:
      output.controlword = 0x0007; // move to switched on
      break;
    case SWITCHED_ON:
      output.controlword = 0x000f; // move to operation enabled
      break;
    case OPERATION_ENABLED:
      break;
    default:
      std::cout << "Did not manage to enable servodrive (uknown status)" << std::endl;
      return;
    }
    writeOutputs();
    usleep(SLEEP_TIME_MS * 1000);
    input = readInputs();
    if (loop++ % 100 == 1)
      printPDSStatus(input);
  }
  if(getPDSStatus(input) != OPERATION_ENABLED)
    std::cout << "Did not manage to enable servodrive (too many attempts)" << std::endl;
}

int ServoClient::change_mode(int mode)
{
  output.operation_mode = mode;
  while (readInputs().operation_mode != mode)
  {
    output.operation_mode = mode;
    writeOutputs();
    usleep(SLEEP_TIME_MS * 1000);
  }
  if (readInputs().operation_mode == mode)
    return mode;
  else
    return 0;
}

bool ServoClient::home()
{
  output.controlword = 0x06;
  writeOutputs();
  change_mode(0x06);
  printf("\n\n\ncurrent mode of operation = %d\n", readInputs().operation_mode);
  printPDSStatus(readInputs());
  output.controlword = 0x0F;
  writeOutputs();

  while((((readInputs().statusword)&0x006f) != 0x0027))
  {}
  printf("status word = %x\n", readInputs().statusword);
  if (readInputs().statusword & 0x1000)
  {
    printf("homed\n");
  }
  else
  {
    printf("we are not homed?\n");
    printPDSStatus(readInputs());
  }
  
  printf("starting homing\n");
  output.controlword = 0x1F;
  writeOutputs();
  while (!((readInputs().statusword & 0x1000) and (readInputs().statusword & 0x0400)))
  {}
  printPDSStatus(readInputs());
  printf("homing done\n");
  output.controlword = 0x0F;
  writeOutputs();
  return 1;

}

bool ServoClient::checkHome()
{
  //TODO:reliable way of checking if drive was already homed...
  //FIXME: This only checks whether drive was previously enabled! 
  // Starting ROS and bypassing home check will enable drives, starting
  // ROS again even with home check will result in false positive
  return readInputs().statusword != 0x231; 
}

void ServoClient::PPGoTo(uint32 position)
{

  change_mode(0x01); // swap to PP mode

  output.target_position = position; // set set-point to 0
  output.controlword = 0x01F; // execute set-point
  writeOutputs();
  std::cout << "Setting new setpoint: " << static_cast<int32>(position) << std::endl;
  while(!(readInputs().statusword & 0x1000)) // wait for drive to accept new setpoint
  {}

  output.controlword = 0x0F; // execute set-point
  writeOutputs();

  while(!(readInputs().statusword & 0x0400)) // wait untill set-point is reached
  {
    std::cout << "Target position: " << static_cast<int32>(position) << " current position: " << static_cast<int32>(readInputs().position_actual_value) << std::endl;
    sleep(0.5);
  }

}

void ServoClient::servoOff()
{
  ServoInput input = readInputs();
  printPDSStatus(input);
  int loop = 0;
  int chk = 10;
  while (not(getPDSStatus(input) == READY_SWITCH or getPDSStatus(input) == SWITCH_DISABLED) && chk--)
  {
    switch (getPDSStatus(input))
    {
    case READY_SWITCH:
      //output.controlword = 0x0000; // disable voltage
      //not implemented
      break;
    case SWITCHED_ON:
      output.controlword = 0x0006; // shutdown
      break;
    case OPERATION_ENABLED:
      output.controlword = 0x0007; // disable operation
      break;
    case SWITCH_DISABLED:
      //output.controlword = 0x0000; // disable voltage
      //not implemented
      break;
    default:
      std::cout << "Did not manage to disable servodrive (uknown status)" << std::endl;
      output.controlword = 0x0006; // shutdown
      return;
    }
    writeOutputs();
    usleep(SLEEP_TIME_MS * 1000);
    input = readInputs();
    if (loop++ % 100 == 1)
      printPDSStatus(input);
  }
  if (not(getPDSStatus(input) == READY_SWITCH or getPDSStatus(input) == SWITCH_DISABLED))
    std::cout << "Did not manage to disable servodrive (too many attempts)" << std::endl;
}

PDS_STATUS ServoClient::getPDSStatus(const ServoInput input) const
{
  uint16 statusword = input.statusword;
  if (((statusword)&0x004f) == 0x0000)
  { // x0xx 0000
    return NOT_READY;
  }
  else if (((statusword)&0x004f) == 0x0040)
  { // x1xx 0000
    return SWITCH_DISABLED;
  }
  else if (((statusword)&0x006f) == 0x0021)
  { // x01x 0001
    return READY_SWITCH;
  }
  else if (((statusword)&0x006f) == 0x0023)
  { // x01x 0011
    return SWITCHED_ON;
  }
  else if (((statusword)&0x006f) == 0x0027)
  { // x01x 0111
    return OPERATION_ENABLED;
  }
  else if (((statusword)&0x006f) == 0x0007)
  { // x00x 0111
    return QUICK_STOP;
  }
  else if (((statusword)&0x004f) == 0x000f)
  { // x0xx 1111
    return FAULT_REACTION;
  }
  else if (((statusword)&0x004f) == 0x0008)
  { // x0xx 1000
    return FAULT;
  }
  else
  {
    return UNKNOWN;
  }
}

PDS_OPERATION ServoClient::getPDSOperation(const ServoInput input) const
{
  int8 operation_mode = input.operation_mode;
  switch (operation_mode)
  {
  case 0:
    return NO_MODE_CHANGE;
    break;
  case 1:
    return PROFILE_POSITION_MODE;
    break; // pp
  case 2:
    return VELOCITY_MODE;
    break; // vl
  case 3:
    return PROFILE_VELOCITY_MODE;
    break; // pv
  case 4:
    return TORQUE_PROFILE_MODE;
    break; // tq
  case 6:
    return HOMING_MODE;
    break; // hm
  case 7:
    return INTERPOLATED_POSITION_MODE;
    break; // ip
  case 8:
    return CYCLIC_SYNCHRONOUS_POSITION_MODE;
    break; // csp
  case 9:
    return CYCLIC_SYNCHRONOUS_VELOCITY_MODE;
    break; // csv
  case 10:
    return CYCLIC_SYNCHRONOUS_TORQUE_MODE;
    break; // cst
  default:
    return _INCORRECT;
    break;
  }
}

PDS_STATUS ServoClient::getPDSControl(const ServoInput input) const
{
  uint16 statusword = input.statusword;

  return _PLACEHOLDER;
}

void ServoClient::printPDSStatus(const ServoInput input) const
{
  printf("Statusword(6041h): %04x\n ", input.statusword);
  std::cout << std::bitset<16>(input.statusword) << std::endl;
  switch (getPDSStatus(input))
  {
  case NOT_READY:
    printf("Not ready to switch on\n");
    break;
  case SWITCH_DISABLED:
    printf("Switch on disabled\n");
    break;
  case READY_SWITCH:
    printf("Ready to switch on\n");
    break;
  case SWITCHED_ON:
    printf("Switched on\n");
    break;
  case OPERATION_ENABLED:
    printf("Operation enabled\n");
    break;
  case QUICK_STOP:
    printf("Quick stop active\n");
    break;
  case FAULT_REACTION:
    printf("Fault reaction active\n");
    break;
  case FAULT:
    printf("Fault\n");
    break;
  case UNKNOWN:
    printf("Unknown status %04x\n", input.statusword);
    break;
  }
  if (input.statusword & 0x0800)
  {
    printf(" Internal limit active\n");
  }
  switch (getPDSOperation(input))
  {
  case PROFILE_POSITION_MODE:
    if ((input.statusword & 0x2000))
    {
      printf(" Following error\n");
    }
    if ((input.statusword & 0x1000))
    {
      printf(" Set-point acknowledge\n");
    }
    if ((input.statusword & 0x0400))
    {
      printf(" Target reached\n");
    }
    break;
  case VELOCITY_MODE:
    break;
  case PROFILE_VELOCITY_MODE:
    if ((input.statusword & 0x2000))
    {
      printf(" Max slippage error (Not supported)\n");
    }
    if ((input.statusword & 0x1000))
    {
      printf(" Speed\n");
    }
    if ((input.statusword & 0x0400))
    {
      printf(" Target reached\n");
    }
    break;
  case TORQUE_PROFILE_MODE:
    if ((input.statusword & 0x0400))
    {
      printf(" Target reached\n");
    }
    break;
  case HOMING_MODE:
    if ((input.statusword & 0x2000))
    {
      printf(" Homing error\n");
    }
    if ((input.statusword & 0x1000))
    {
      printf(" Homing attained\n");
    }
    if ((input.statusword & 0x0400))
    {
      printf(" Target reached\n");
    }
    break;
  case INTERPOLATED_POSITION_MODE:
    if ((input.statusword & 0x1000))
    {
      printf(" Ip mode active\n");
    }
    if ((input.statusword & 0x0400))
    {
      printf(" Target reached\n");
    }
    break;
  case CYCLIC_SYNCHRONOUS_POSITION_MODE:
    if ((input.statusword & 0x2000))
    {
      printf(" Following error\n");
    }
    if ((input.statusword & 0x1000))
    {
      printf(" Drive follows command value\n");
    }
    break;
  case CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
    if ((input.statusword & 0x1000))
    {
      printf(" Drive follows command value\n");
    }
    break;
  case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
    if ((input.statusword & 0x1000))
    {
      printf(" Drive follows command value\n");
    }
    break;
  case _INCORRECT:
    printf(" Incorrect value in operation mode register\n");
    break;
  }
}

void ServoClient::printPDSOperation(const ServoInput input) const
{
  printf("Mode of operation(6061h): %04x\n ", input.operation_mode);
  switch (getPDSOperation(input))
  {
  case NO_MODE_CHANGE:
    printf("No mode change / no mode assigned\n");
    break;
  case PROFILE_POSITION_MODE:
    printf("Profile position mode\n");
    break;
  case VELOCITY_MODE:
    printf("Velocity mode\n");
    break;
  case PROFILE_VELOCITY_MODE:
    printf("Profile velocity mode\n");
    break;
  case TORQUE_PROFILE_MODE:
    printf("Torque profile mode\n");
    break;
  case HOMING_MODE:
    printf("Homing mode\n");
    break;
  case INTERPOLATED_POSITION_MODE:
    printf("Interpolated position mode\n");
    break;
  case CYCLIC_SYNCHRONOUS_POSITION_MODE:
    printf("Cyclic synchronous position mode\n");
    break;
  case CYCLIC_SYNCHRONOUS_VELOCITY_MODE:
    printf("Cyclic synchronous velocity mode\n");
    break;
  case CYCLIC_SYNCHRONOUS_TORQUE_MODE:
    printf("Cyclic synchronous torque mode\n");
    break;
  default:
    printf("Reserved %04x\n", input.operation_mode);
    break;
  }
}

void ServoClient::printPDSControl(const ServoInput input) const
{
}