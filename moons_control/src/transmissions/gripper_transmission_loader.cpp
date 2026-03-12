// ROS
#include <ros/console.h>

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// ros_control
#include <hardware_interface/internal/demangle_symbol.h>
#include <moons_control/gripper_transmission.hpp>
#include <moons_control/gripper_transmission_loader.hpp>
namespace transmission_interface
{

TransmissionLoader::ParseStatus GripperTransmissionLoader::getR1(const TiXmlElement &parent_el,
                                                                 const std::string &actuator_name,
                                                                 const std::string &transmission_name,
                                                                 bool required,
                                                                 double &r1)
{
    // Get XML element
    const TiXmlElement *sp_el = parent_el.FirstChildElement("R1");
    if (!sp_el)
    {
        if (required)
        {
            ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' does not specify the required <R1> element.");
        }
        else
        {
            ROS_DEBUG_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' does not specify the optional <R2> element.");
        }
        return NO_DATA;
    }

    // Cast to number
    try
    {
        r1 = boost::lexical_cast<double>(sp_el->GetText());
    }
    catch (const boost::bad_lexical_cast &)
    {
        ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' specifies the <R1> element, but is not a number.");
        return BAD_TYPE;
    }
    return SUCCESS;
}

TransmissionLoader::ParseStatus GripperTransmissionLoader::getR2(const TiXmlElement &parent_el,
                                                                 const std::string &actuator_name,
                                                                 const std::string &transmission_name,
                                                                 bool required,
                                                                 double &r2)
{
    // Get XML element
    const TiXmlElement *sp_el = parent_el.FirstChildElement("R2");
    if (!sp_el)
    {
        if (required)
        {
            ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' does not specify the required <R2> element.");
        }
        else
        {
            ROS_DEBUG_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' does not specify the optional <R2> element.");
        }
        return NO_DATA;
    }

    // Cast to number
    try
    {
        r2 = boost::lexical_cast<double>(sp_el->GetText());
    }
    catch (const boost::bad_lexical_cast &)
    {
        ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' specifies the <R2> element, but is not a number.");
        return BAD_TYPE;
    }
    return SUCCESS;
}

TransmissionLoader::ParseStatus GripperTransmissionLoader::getOffset(const TiXmlElement &parent_el,
                                                                 const std::string &actuator_name,
                                                                 const std::string &transmission_name,
                                                                 bool required,
                                                                 double &offset)
{
    // Get XML element
    const TiXmlElement *sp_el = parent_el.FirstChildElement("offset");
    if (!sp_el)
    {
        if (required)
        {
            ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' does not specify the required <offset> element.");
        }
        else
        {
            ROS_DEBUG_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' does not specify the optional <R2> element.");
        }
        return NO_DATA;
    }

    // Cast to number
    try
    {
        offset = boost::lexical_cast<double>(sp_el->GetText());
    }
    catch (const boost::bad_lexical_cast &)
    {
        ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << actuator_name << "' of transmission '" << transmission_name << "' specifies the <R2> element, but is not a number.");
        return BAD_TYPE;
    }
    return SUCCESS;
}

TransmissionSharedPtr GripperTransmissionLoader::load(const TransmissionInfo &transmission_info)
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

    // Parse required mechanical reduction
    double r1 = 0.0;
    const ParseStatus r1_status = getR1(actuator_el,
                                                       transmission_info.actuators_.front().name_,
                                                       transmission_info.name_,
                                                       true, // Required
                                                       r1);
    if (r1_status != SUCCESS)
    {
        return TransmissionSharedPtr();
    }

    double r2 = 0.0;
    const ParseStatus r2_status = getR2(actuator_el,
                                                       transmission_info.actuators_.front().name_,
                                                       transmission_info.name_,
                                                       true, // Required
                                                       r2);
    if (r2_status != SUCCESS)
    {
        return TransmissionSharedPtr();
    }

    double offset = 0.0;
    const ParseStatus offset_status = getOffset(actuator_el,
                                                       transmission_info.actuators_.front().name_,
                                                       transmission_info.name_,
                                                       true, // Required
                                                       offset);
    if (offset_status != SUCCESS)
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
        TransmissionSharedPtr transmission(new GripperTransmission(reduction, r1, r2, offset));
        return transmission;
    }
    catch (const TransmissionInterfaceException &ex)
    {
        using hardware_interface::internal::demangledTypeName;
        ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '" << transmission_info.name_ << "' of type '" << demangledTypeName<GripperTransmission>() << "'. " << ex.what());
        return TransmissionSharedPtr();
    }
}

} // namespace transmission_interface

PLUGINLIB_EXPORT_CLASS(transmission_interface::GripperTransmissionLoader,
                       transmission_interface::TransmissionLoader)
