// ROS
#include <ros/console.h>

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// ros_control
#include <hardware_interface/internal/demangle_symbol.h>
#include <mandro_transmission/lead_screw.hpp>
#include <mandro_transmission/lead_screw_loader.hpp>
namespace transmission_interface
{

TransmissionLoader::ParseStatus LeadScrewLoader::getActuatorScrewPitch(const TiXmlElement &parent_el,
                                                                       const std::string &actuator_name,
                                                                       const std::string &transmission_name,
                                                                       bool required,
                                                                       double &sp)
{
  // Get XML element
  const TiXmlElement *sp_el = parent_el.FirstChildElement("screwPitch");
  if (!sp_el)
  {
    if (required)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' does not specify the required <screwPitch> element.");
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' does not specify the optional <screwPitch> element.");
    }
    return NO_DATA;
  }

  // Cast to number
  try
  {
    sp = boost::lexical_cast<double>(sp_el->GetText());
  }
  catch (const boost::bad_lexical_cast &)
  {
    ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' specifies the <screwPitch> element, but is not a number.");
    return BAD_TYPE;
  }
  return SUCCESS;
}

TransmissionSharedPtr LeadScrewLoader::load(const TransmissionInfo &transmission_info)
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

  // Parse required pinion radius
  double sp = 0.0;
  const int sp_status = getActuatorScrewPitch(actuator_el,
                                              transmission_info.actuators_.front().name_,
                                              transmission_info.name_,
                                              true, // Required
                                              sp);
  if (sp_status != SUCCESS)
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
    TransmissionSharedPtr transmission(new LeadScrew(sp, reduction));
    return transmission;
  }
  catch (const TransmissionInterfaceException &ex)
  {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '" << transmission_info.name_ << "' of type '" << demangledTypeName<LeadScrew>() << "'. " << ex.what());
    return TransmissionSharedPtr();
  }
}

} // namespace transmission_interface

PLUGINLIB_EXPORT_CLASS(transmission_interface::LeadScrewLoader,
                       transmission_interface::TransmissionLoader)
