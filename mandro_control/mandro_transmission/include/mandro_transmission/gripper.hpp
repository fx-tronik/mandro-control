#pragma once

#include <cassert>
#include <string>
#include <vector>
#include <math.h>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace transmission_interface
{

//TODO: efficiency
class Gripper : public Transmission
{
public:
  Gripper(const double reduction);

  void actuatorToJointEffort(const ActuatorData &act_data,
                             JointData &jnt_data);

  /**
   * \brief Transform \e velocity variables from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre Actuator and joint velocity vectors must have size 1 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void actuatorToJointVelocity(const ActuatorData &act_data,
                               JointData &jnt_data);

  /**
   * \brief Transform \e position variables from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre Actuator and joint position vectors must have size 1 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void actuatorToJointPosition(const ActuatorData &act_data,
                               JointData &jnt_data);

  /**
   * \brief Transform \e effort variables from joint to actuator space.
   * \param[in]  jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre Actuator and joint effort vectors must have size 1 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void jointToActuatorEffort(const JointData &jnt_data,
                             ActuatorData &act_data);

  /**
   * \brief Transform \e velocity variables from joint to actuator space.
   * \param[in]  jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre Actuator and joint velocity vectors must have size 1 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void jointToActuatorVelocity(const JointData &jnt_data,
                               ActuatorData &act_data);

  /**
   * \brief Transform \e position variables from joint to actuator space.
   * \param[in]  jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre Actuator and joint position vectors must have size 1 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void jointToActuatorPosition(const JointData &jnt_data,
                               ActuatorData &act_data);

  void actuatorToJointAbsolutePosition(const ActuatorData& act_data,
                                             JointData&    jnt_data){;}

  void actuatorToJointTorqueSensor(const ActuatorData& act_data,
                                         JointData&    jnt_data){;}

  bool hasActuatorToJointAbsolutePosition() const {return false;}
  bool hasActuatorToJointTorqueSensor()     const {return false;}

  std::size_t numActuators() const { return 1; }
  std::size_t numJoints() const { return 1; }

  double getActuatorReduction() const { return reduction_; }

private:
  double reduction_;
};

inline Gripper::Gripper(const double reduction)
    : Transmission(),
      reduction_(reduction)
{
  if (0.0 == reduction_)
  {
    throw TransmissionInterfaceException("Transmission reduction ratio cannot be zero.");
  }
}

inline void Gripper::actuatorToJointEffort(const ActuatorData &act_data,
                                             JointData &jnt_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && jnt_data.effort[0]);
  //TODO: Effort calculation
  *jnt_data.effort[0] = *act_data.effort[0];
}

inline void Gripper::actuatorToJointVelocity(const ActuatorData &act_data,
                                               JointData &jnt_data)
{
  assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  assert(act_data.velocity[0] && jnt_data.velocity[0]);
  *jnt_data.velocity[0] = *act_data.velocity[0];
}

inline void Gripper::actuatorToJointPosition(const ActuatorData &act_data,
                                               JointData &jnt_data)
{
  assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  assert(act_data.position[0] && jnt_data.position[0]);
  *jnt_data.position[0] = *act_data.position[0];
}

inline void Gripper::jointToActuatorEffort(const JointData &jnt_data,
                                             ActuatorData &act_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && jnt_data.effort[0]);
  //TODO: Effort calculation
  *act_data.effort[0] = *jnt_data.effort[0];
}

inline void Gripper::jointToActuatorVelocity(const JointData &jnt_data,
                                               ActuatorData &act_data)
{
  assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  assert(act_data.velocity[0] && jnt_data.velocity[0]);
  *act_data.velocity[0] = *jnt_data.velocity[0];
}

inline void Gripper::jointToActuatorPosition(const JointData &jnt_data,
                                               ActuatorData &act_data)
{
  assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  assert(act_data.position[0] && jnt_data.position[0]);
  *act_data.position[0] = *jnt_data.position[0];
}

} // namespace transmission_interface
