// ROS
#include <ros/console.h>

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// ros_control
#include <hardware_interface/internal/demangle_symbol.h>
#include <mandro_transmission/gripper.hpp>
#include <mandro_transmission/gripper_loader.hpp>
namespace transmission_interface
{

TransmissionSharedPtr GripperLoader::load(const TransmissionInfo &transmission_info)
{
  // Transmission should contain only one actuator/joint
  if (!checkActuatorDimension(transmission_info, 1))
  {
    return TransmissionSharedPtr();
  }
  if (!checkJointDimension(transmission_info, 1))
  {
    return TransmissionSharedPtr();
  }

  // Parse actuator and joint xml elements
  TiXmlElement actuator_el = loadXmlElement(transmission_info.actuators_.front().xml_element_);
  TiXmlElement joint_el = loadXmlElement(transmission_info.joints_.front().xml_element_);

  // Parse required mechanical reduction
  double reduction = 0.0;
  const ParseStatus reduction_status = getActuatorReduction(actuator_el,
                                                            transmission_info.actuators_.front().name_,
                                                            transmission_info.name_,
                                                            true, // Required
                                                            reduction);
  if (reduction_status != SUCCESS)
  {
    return TransmissionSharedPtr();
  }

  // Parse optional joint offset. Even though it's optional --and to avoid surprises-- we fail if the element is
  // specified but is of the wrong type
  double joint_offset = 0.0;
  const ParseStatus joint_offset_status = getJointOffset(joint_el,
                                                         transmission_info.joints_.front().name_,
                                                         transmission_info.name_,
                                                         false, // Optional
                                                         joint_offset);
  if (joint_offset_status == BAD_TYPE)
  {
    return TransmissionSharedPtr();
  }

  // Transmission instance
  try
  {
    TransmissionSharedPtr transmission(new Gripper(reduction));
    return transmission;
  }
  catch (const TransmissionInterfaceException &ex)
  {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '" << transmission_info.name_ << "' of type '" << demangledTypeName<Gripper>() << "'. " << ex.what());
    return TransmissionSharedPtr();
  }
}

} // namespace transmission_interface

PLUGINLIB_EXPORT_CLASS(transmission_interface::GripperLoader,
                       transmission_interface::TransmissionLoader)
