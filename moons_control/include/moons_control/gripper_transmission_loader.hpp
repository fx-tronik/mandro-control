#pragma once

// ros_control
#include <transmission_interface/transmission_loader.h>

namespace transmission_interface
{

/**
 * \brief Class for loading a simple transmission instance from configuration data.
 */
class GripperTransmissionLoader : public TransmissionLoader
{
public:
    TransmissionSharedPtr load(const TransmissionInfo &transmission_info);
    TransmissionLoader::ParseStatus getR1(const TiXmlElement &parent_el,
                                          const std::string &actuator_name,
                                          const std::string &transmission_name,
                                          bool required,
                                          double &r1);
    TransmissionLoader::ParseStatus getR2(const TiXmlElement &parent_el,
                                          const std::string &actuator_name,
                                          const std::string &transmission_name,
                                          bool required,
                                          double &r2);
    TransmissionLoader::ParseStatus getOffset(const TiXmlElement &parent_el,
                                          const std::string &actuator_name,
                                          const std::string &transmission_name,
                                          bool required,
                                          double &offset);
};
}; // namespace transmission_interface
