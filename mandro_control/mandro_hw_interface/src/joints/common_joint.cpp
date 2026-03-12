#include <mandro_hw_interface/mandro_hardware_interface.hpp>

JointControlInterface::JointControlInterface(int slave_no, ActuatorDataContainer &actuators_data) : actuator(actuators_data[slave_no])
{
    // create joint name
    std::stringstream ss;
    ss << "joint" << slave_no;
    actuator.joint_name_ = ss.str();
    ss << "_motor";
    actuator.actuator_name_ = ss.str();
}

std::string JointControlInterface::getJointName()
{
    return actuator.joint_name_;
}

std::string JointControlInterface::getActuatorName()
{
    return actuator.actuator_name_;
}