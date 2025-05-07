#include "vam_message.h"
#include <boost/make_shared.hpp>
#include <ros_etsi_its_msgs/msg/vam.hpp>
#include <vanetza/asn1/support/asn_application.h>
#include <vanetza/asn1/vam.hpp>

namespace v2x_stack_btp
{

boost::shared_ptr<ros_etsi_its_msgs::msg::VAM> convertVam(const vanetza::asn1::r1::Vam& asn1, std::string* error_msg)
{
    auto msg = boost::make_shared<ros_etsi_its_msgs::msg::VAM>();

    // VAM header fields
    msg->its_header.protocol_version = asn1->header.protocolVersion;
    msg->its_header.station_id = asn1->header.stationID;

    // VAM payload
    const auto& vam = asn1->vam;
    msg->generation_delta_time = vam.generationDeltaTime;

    // Basic container
    const auto& basic = vam.vamParameters.basicContainer;
    msg->basic_container.station_type.value = basic.stationType;
    msg->basic_container.reference_position.latitude = basic.referencePosition.latitude;
    msg->basic_container.reference_position.longitude = basic.referencePosition.longitude;
    msg->basic_container.reference_position.altitude.value = basic.referencePosition.altitude.altitudeValue;
    msg->basic_container.reference_position.altitude.confidence = basic.referencePosition.altitude.altitudeConfidence;

    // High-frequency container
    const auto& hfc = vam.vamParameters.vruHighFrequencyContainer;
    msg->vru_high_frequency_container.speed.value = hfc.speed.speedValue;
    msg->vru_high_frequency_container.speed.confidence = hfc.speed.speedConfidence;
    msg->vru_high_frequency_container.heading.value = hfc.heading.headingValue;
    msg->vru_high_frequency_container.heading.confidence = hfc.heading.headingConfidence;

    // Optional containers
    if (vam.vamParameters.vruLowFrequencyContainer)
    {
        const auto& lfc = *vam.vamParameters.vruLowFrequencyContainer;
        msg->vru_low_frequency_container_is_present = true;
        msg->vru_low_frequency_container.vehicle_role.value = lfc.vehicleRole;
    }

    return msg;
}

vanetza::asn1::r1::Vam convertVam(ros_etsi_its_msgs::msg::VAM::ConstSharedPtr ptr)
{
    vanetza::asn1::r1::Vam msg;

    // VAM header
    msg->header.protocolVersion = ptr->its_header.protocol_version;
    msg->header.stationID = ptr->its_header.station_id;

    // VAM payload
    auto& vam = msg->vam;
    vam.generationDeltaTime = ptr->generation_delta_time;

    // Basic container
    auto& basic = vam.vamParameters.basicContainer;
    basic.stationType = ptr->basic_container.station_type.value;
    basic.referencePosition.latitude = ptr->basic_container.reference_position.latitude;
    basic.referencePosition.longitude = ptr->basic_container.reference_position.longitude;
    basic.referencePosition.altitude.altitudeValue = ptr->basic_container.reference_position.altitude.value;
    basic.referencePosition.altitude.altitudeConfidence = ptr->basic_container.reference_position.altitude.confidence;

    // High-frequency container
    auto& hfc = vam.vamParameters.vruHighFrequencyContainer;
    hfc.speed.speedValue = ptr->vru_high_frequency_container.speed.value;
    hfc.speed.speedConfidence = ptr->vru_high_frequency_container.speed.confidence;
    hfc.heading.headingValue = ptr->vru_high_frequency_container.heading.value;
    hfc.heading.headingConfidence = ptr->vru_high_frequency_container.heading.confidence;

    // Optional containers
    if (ptr->vru_low_frequency_container_is_present)
    {
        vam.vamParameters.vruLowFrequencyContainer = vanetza::asn1::allocate<VruLowFrequencyContainer_t>();
        auto& lfc = *vam.vamParameters.vruLowFrequencyContainer;
        lfc.vehicleRole = ptr->vru_low_frequency_container.vehicle_role.value;
    }

    return msg;
}

} // namespace v2x_stack_btp