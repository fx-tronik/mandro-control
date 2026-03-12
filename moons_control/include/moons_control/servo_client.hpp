#ifndef SERVO_CLIENT_H
#define SERVO_CLIENT_H

#include <soem/osal.h>
#include <moons_control/ecat_controller.hpp>
#include <bitset>

#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

typedef enum
{
    NOT_READY,
    SWITCH_DISABLED,
    READY_SWITCH,
    SWITCHED_ON,
    OPERATION_ENABLED,
    QUICK_STOP,
    FAULT_REACTION,
    FAULT,
    UNKNOWN,
    _PLACEHOLDER
} PDS_STATUS; // Statusword(6041h) SX-DSV02470 p.78

typedef enum
{
    NO_MODE_CHANGE,
    PROFILE_POSITION_MODE,
    VELOCITY_MODE,
    PROFILE_VELOCITY_MODE,
    TORQUE_PROFILE_MODE,
    HOMING_MODE,
    INTERPOLATED_POSITION_MODE,
    CYCLIC_SYNCHRONOUS_POSITION_MODE,
    CYCLIC_SYNCHRONOUS_VELOCITY_MODE,
    CYCLIC_SYNCHRONOUS_TORQUE_MODE,
    _INCORRECT
} PDS_OPERATION; // Mode of operation(6061h) SX-DSV02470 p.83

typedef enum
{
    HALT,
    FAULT_RESET,
    ENABLE_OPERATION,
    QUICK_STOP_C,
    ENABLE_VOLTAGE,
    SWITCH_ON,
} PDS_CONTROL; // Controlworld(6040h) SX-DSV02470 p.76

typedef enum
{
    POS_LIMIT       = 1 <<  0,
    CCW_LIMIT       = 1 <<  1,
    CW_LIMIT        = 1 <<  2,
    OVER_TEMP       = 1 <<  3,
    VOLTAGE_BAD     = 1 <<  4,
    OVER_VOLTAGE    = 1 <<  5,
    UNDER_VOLTAGE   = 1 <<  6,
    OVER_CURRENT    = 1 <<  7,
    OPEN_WINDING    = 1 <<  8,
    BAD_ENCODER     = 1 <<  9,
    COMM_ERROR      = 1 << 10,
    SAVE_FAILED     = 1 << 11,
    NO_MOVE         = 1 << 12,
    CURR_FOLDBACK   = 1 << 13,
    BLANK_Q_SEG     = 1 << 14,
    NV_MEM_ERROR    = 1 << 15,
    EXCESS_REGEN    = 1 << 16,
    RESERVED_17     = 1 << 17,
    STO             = 1 << 18,
    RESERVED_19     = 1 << 19,
    RESERVED_20     = 1 << 20,
} ALARM_CODES; // Alarm codes(200Fh)

namespace moons_control
{
namespace servo
{

using moons_control::ethercat_controller::Controller;

typedef struct
{
    //input
    uint16 error_code;
    uint16 statusword;
    uint8 operation_mode;
    uint32 position_actual_value;
    uint32 follow_error_value;
    uint32 velocity_actual_value;
    uint16 torque_actual_value;
    uint32 alarm_code;
    uint16 current_actual_value;
} ServoInput;

typedef struct
{
    uint16 controlword;
    uint8 operation_mode;
    uint32 target_position;
    uint32 target_velocity;
} ServoOutput;

class ServoClient
{
public:
    ServoClient(Controller &manager, int slave_no, ServoOutput &output);
    ~ServoClient();

    void writeOutputs();

    ServoInput readInputs() const;

    void readOutputs() const;
    void reset();

    void servoOn();

    int change_mode(int mode);

    bool home();

    bool checkHome();

    void PPGoTo(uint32 position);

    void servoOff();

    void printPDSStatus(const ServoInput input) const;

    void printPDSOperation(const ServoInput input) const;

    void printPDSControl(const ServoInput input) const;

private:
    PDS_STATUS getPDSStatus(const ServoInput input) const;

    PDS_OPERATION getPDSOperation(const ServoInput input) const;

    PDS_STATUS getPDSControl(const ServoInput input) const;

    Controller &manager_;
    ServoOutput &output;
    const int slave_no_;
};

const struct
{
    unsigned int code;
    const char *text;
} error_map[] = {
    {  1, "Over Current"},
    {  2, "Over Voltage"},
    {  3, "Over Temperature"},
    {  4, "Open Motor Winding"},
    {  5, "Internal Voltage Bad"},
    {  6, "Position Limit"},
    {  7, "Bad encoder"},
    {  8, "reserved"},
    {  9, "reserved"},
    { 10, "Excess Regen"},
    { 11, "Safe Torque Off"},
    { 49, "CW Limit"},
    { 50, "CCW Limit"},
    { 51, "CW and CCW Limit"},
    { 52, "Current Foldback"},
    { 53, "Move while disabled"},
    { 54, "Under Voltage"},
    { 55, "Blank Q Segment"},
    { 65, "Save Error"},
    { 99, "Uknown error"},
    {255, "Other error"},
};

} // namespace servo
} // namespace moons_control

#endif