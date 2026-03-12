#pragma once

// ros_control
#include <transmission_interface/transmission_loader.h>

namespace transmission_interface
{

/**
 * \brief Class for loading a simple transmission instance from configuration data.
 */
class LeadScrewLoader : public TransmissionLoader
{
public:

  TransmissionSharedPtr load(const TransmissionInfo& transmission_info);
  TransmissionLoader::ParseStatus getActuatorScrewPitch(const TiXmlElement& parent_el,
                                         const std::string&  actuator_name,
                                         const std::string&  transmission_name,
                                         bool                required,
                                         double&             sp);

};

} // namespace