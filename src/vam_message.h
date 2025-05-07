// filepath: /home/vboxuser/colcon_ws/src/v2x_stack/include/v2x_stack/vam_message.h
#ifndef V2X_STACK_VAM_MESSAGE_H
#define V2X_STACK_VAM_MESSAGE_H

#include <v2x_stack/extern/vanetza/vanetza/asn1/vam.hpp>
#include <vanetza/asn1/vam.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

namespace v2x_stack_btp
{
boost::shared_ptr<ros_etsi_its_msgs::msg::VAM> convertVam(const vanetza::asn1::r1::Vam& asn1, std::string* error_msg = nullptr);
vanetza::asn1::r1::Vam convertVam(ros_etsi_its_msgs::msg::VAM::ConstSharedPtr ptr);
}

#endif // V2X_STACK_VAM_MESSAGE_H