#pragma once

// ros_control
#include <transmission_interface/transmission_loader.h>

namespace transmission_interface
{

/**
 * \brief Class for loading a simple transmission instance from configuration data.
 */
class GripperLoader : public TransmissionLoader
{
public:

  TransmissionSharedPtr load(const TransmissionInfo& transmission_info);
};

} // namespace