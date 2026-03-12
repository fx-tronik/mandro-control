#pragma once

#include <cstdint>
#include <array>
#include <bitset>
#include <thread>
#include <map>
#include <string>
#include <vector>
#include <unistd.h>
#include <math.h>

#include <soem/osal.h>

#include <ros/ros.h>
#include <urdf/model.h>

#include <realtime_tools/realtime_publisher.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <std_msgs/Float64.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include <moons_servo/home.h>
#include <moons_servo/reset.h>
#include <moons_servo/go2pos.h>
#include <moons_servo/estop.h>

#include <ethercat_controller/controller.hpp>

typedef enum
{
    POS_LIMIT = 1 << 0,
    CCW_LIMIT = 1 << 1,
    CW_LIMIT = 1 << 2,
    OVER_TEMP = 1 << 3,
    VOLTAGE_BAD = 1 << 4,
    OVER_VOLTAGE = 1 << 5,
    UNDER_VOLTAGE = 1 << 6,
    OVER_CURRENT = 1 << 7,
    OPEN_WINDING = 1 << 8,
    BAD_ENCODER = 1 << 9,
    COMM_ERROR = 1 << 10,
    SAVE_FAILED = 1 << 11,
    NO_MOVE = 1 << 12,
    CURR_FOLDBACK = 1 << 13,
    BLANK_Q_SEG = 1 << 14,
    NV_MEM_ERROR = 1 << 15,
    EXCESS_REGEN = 1 << 16,
    RESERVED_17 = 1 << 17,
    STO = 1 << 18,
    RESERVED_19 = 1 << 19,
    RESERVED_20 = 1 << 20,
} ALARM_CODES; // Alarm codes(200Fh)

class Controller;

typedef struct
{
    uint16_t error_code;
    uint16_t statusword;
    uint8_t operation_mode;
    uint32_t position_actual_value;
    uint32_t follow_error_value;
    uint32_t velocity_actual_value;
    uint16_t torque_actual_value;
    uint32_t alarm_code;
    uint16_t current_actual_value;
} ServoInput;

typedef struct
{
    uint16_t controlword;
    uint8_t operation_mode;
    uint32_t target_position;
    uint32_t target_velocity;
} ServoOutput;

class MoonsServo
{
public:
    enum OperationMode
    {
        No_Mode = 0,
        Profiled_Position = 1,
        Velocity = 2,
        Profiled_Velocity = 3,
        Profiled_Torque = 4,
        Reserved = 5,
        Homing = 6,
        Interpolated_Position = 7,
        Cyclic_Synchronous_Position = 8,
        Cyclic_Synchronous_Velocity = 9,
        Cyclic_Synchronous_Torque = 10,
    };

    MoonsServo(ros::NodeHandle &nh, Controller &controller, int slave_no);
    ~MoonsServo();

    /** \name Real-Time Safe Functions */

    inline void WriteOutputs()
    {
        WriteOutputsToBuffer();
    }

    inline void ReadInputs()
    {
        ReadInputsFromBuffer();
    }

    inline void SetPosCommand(double pos_cmd) { output_.target_position = RAD2PULSE(pos_cmd); }

    inline double GetCurrentPos() const { return PULSE2RAD(input_.position_actual_value); }

    inline void SetVelCommand(double vel_cmd) { output_.target_velocity = RAD2PULSE(vel_cmd); }

    inline double GetCurrentVel() const { return PULSE2RAD(input_.velocity_actual_value); }

    /** \name Non Real-Time Safe Functions */

    int Reset();

    int ServoOn();

    int ServoOff();

    OperationMode ChangeMode(OperationMode mode);

    int CheckHome();

    int Go2Pos(uint32_t position);

    int Home();

    int EStop();

private:
    enum StatusWord
    {
        SW_Ready_To_Switch_On = 0,
        SW_Switched_On = 1,
        SW_Operation_enabled = 2,
        SW_Fault = 3,
        SW_Voltage_enabled = 4,
        SW_Quick_stop = 5,
        SW_Switch_on_disabled = 6,
        SW_Warning = 7,
        SW_Manufacturer_specific0 = 8,
        SW_Remote = 9,
        SW_Target_reached = 10,
        SW_Internal_limit = 11,
        SW_Operation_mode_specific0 = 12,
        SW_Operation_mode_specific1 = 13,
        SW_Manufacturer_specific1 = 14,
        SW_Manufacturer_specific2 = 15
    };

    enum class InternalState
    {
        Unknown = 0,
        Start = 0,
        Not_Ready_To_Switch_On = 1,
        Switch_On_Disabled = 2,
        Ready_To_Switch_On = 3,
        Switched_On = 4,
        Operation_Enable = 5,
        Quick_Stop_Active = 6,
        Fault_Reaction_Active = 7,
        Fault = 8,
    };

    std::map<int, std::string> mapping =
        {
            {1, "Over Current"},
            {2, "Over Voltage"},
            {3, "Over Temperature"},
            {4, "Open Motor Winding"},
            {5, "Internal Voltage Bad"},
            {6, "Position Limit"},
            {7, "Bad encoder"},
            {8, "reserved"},
            {9, "reserved"},
            {10, "Excess Regen"},
            {11, "Safe Torque Off"},
            {49, "CW Limit"},
            {50, "CCW Limit"},
            {51, "CW and CCW Limit"},
            {52, "Current Foldback"},
            {53, "Move while disabled"},
            {54, "Under Voltage"},
            {55, "Blank Q Segment"},
            {65, "Save Error"},
            {99, "Uknown error"},
            {255, "Other error"},
    };

    std::map<int, std::string>::iterator it = mapping.end();

    /* ROS API */

    //Node Handle
    ros::NodeHandle nh_;

    //ROS Services
    ros::ServiceServer s_reset;
    ros::ServiceServer s_home;
    ros::ServiceServer s_go2pos;
    ros::ServiceServer s_estop;

    bool SHome(moons_servo::home::Request &req, moons_servo::home::Response &res);
    bool SReset(moons_servo::reset::Request &req, moons_servo::reset::Response &res);
    bool SGo2Pos(moons_servo::go2pos::Request &req, moons_servo::go2pos::Response &res);
    bool SEstop(moons_servo::estop::Request &req, moons_servo::estop::Response &res);

    //Input&Output structures
    ServoInput input_;
    ServoOutput output_;

    //ServoDrive variables
    double pulse_per_rev_;
    double Kt_;

    //EtherCAT Controller
    Controller &controller_;
    const int slave_no_;

    //Ethercat input/output maps
    std::array<uint8_t, 24> input_map_ = {0};
    std::array<uint8_t, 11> write_output_map_ = {0};
    std::array<uint8_t, 11> read_output_map_ = {0};

    //Internal
    bool in_rt_loop;
    std::string servo_name;
    std::thread diagnostic_thread;
    int ecode = 0;
    realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> *publisher = NULL;


    void DiagnosticLoop();

    const ServoOutput &WriteOutputsToBuffer();

    const ServoInput &ReadInputsFromBuffer();

    inline uint32_t RAD2PULSE(double rad) const { return static_cast<uint32_t>(rad * (pulse_per_rev_ / (2 * M_PI))); }

    inline double PULSE2RAD(int32 pulse) const { return static_cast<double>(pulse / (pulse_per_rev_ / (2 * M_PI))); }

    InternalState getState(const ServoInput input);

    void publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher);
};