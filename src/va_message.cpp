#include "va_message.hpp"
#include <boost/make_shared.hpp>
#include <ros_etsi_its_msgs/msg/vam.hpp>
#include <vanetza/asn1/support/asn_application.h>
#include <vanetza/asn1/vam.hpp>

namespace v2x_stack_btp
{

namespace
{

inline uint8_t reverse_byte(uint8_t byte)
{
    byte = (byte & 0xf0) >> 4 | (byte & 0x0f) << 4;
    byte = (byte & 0xcc) >> 2 | (byte & 0x33) << 2;
    byte = (byte & 0xaa) >> 1 | (byte & 0x55) << 1;
    return byte;
}

} // namespace

boost::shared_ptr<ros_etsi_its_msgs::msg::VAM> convertVam(const vanetza::asn1::r1::Vam& asn1, std::string* error_msg)
{
    auto msg = boost::make_shared<ros_etsi_its_msgs::msg::VAM>();
    //ros_etsi_its_msgs::msg::VAM msg;

    // etsi_its_msgs/VAM header fields
    //msg.header.stamp = ros::Time::now();
    msg->its_header.protocol_version = asn1->header.protocolVersion;
    msg->its_header.station_id = asn1->header.stationID;
    msg->generation_delta_time = asn1->vam.generationDeltaTime;

    // basic container
    const auto& params = asn1->vam.vamParameters;
    msg->station_type.value = params.basicContainer.stationType;
    const auto& refpos = params.basicContainer.referencePosition;
    msg->reference_position.altitude.value = refpos.altitude.altitudeValue;
    msg->reference_position.altitude.confidence = refpos.altitude.altitudeConfidence;
    msg->reference_position.latitude = refpos.latitude;
    msg->reference_position.longitude = refpos.longitude;
    msg->reference_position.position_confidence.semi_major_confidence = refpos.positionConfidenceEllipse.semiMajorConfidence;
    msg->reference_position.position_confidence.semi_minor_confidence = refpos.positionConfidenceEllipse.semiMinorConfidence;
    msg->reference_position.position_confidence.semi_major_orientation = refpos.positionConfidenceEllipse.semiMajorOrientation;

    // high frequency container
    // TODO: implement parsing for rsuContainerHighFrequency
    if (params.highFrequencyContainer.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency)
    {
        const auto& hfc = params.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        msg->high_frequency_container.heading.value = hfc.heading.headingValue;
        msg->high_frequency_container.heading.confidence = hfc.heading.headingConfidence;
        msg->high_frequency_container.speed.value = hfc.speed.speedValue;
        msg->high_frequency_container.speed.confidence = hfc.speed.speedConfidence;
        msg->high_frequency_container.drive_direction.value = hfc.driveDirection;
        msg->high_frequency_container.vehicle_length.value = hfc.vehicleLength.vehicleLengthValue;
        msg->high_frequency_container.vehicle_length.confidence_indication = hfc.vehicleLength.vehicleLengthConfidenceIndication;
        msg->high_frequency_container.vehicle_width.value = hfc.vehicleWidth;
        msg->high_frequency_container.longitudinal_acceleration.value = hfc.longitudinalAcceleration.longitudinalAccelerationValue;
        msg->high_frequency_container.longitudinal_acceleration.confidence = hfc.longitudinalAcceleration.longitudinalAccelerationConfidence;
        msg->high_frequency_container.curvature.value = hfc.curvature.curvatureValue;
        msg->high_frequency_container.curvature.confidence = hfc.curvature.curvatureConfidence;
        msg->high_frequency_container.curvature_calculation_mode.value = hfc.curvatureCalculationMode;
        msg->high_frequency_container.yaw_rate.value = hfc.yawRate.yawRateValue;
        msg->high_frequency_container.yaw_rate.confidence = hfc.yawRate.yawRateConfidence;
    }
    //else
    //{
        //if (error_msg) *error_msg = "missing BasicVehicleContainerHighFrequency container";
        // return nullptr;
        // nullpointer on return commented to avoid non-functional node due to unexpected HF container
    //}

    if (params.lowFrequencyContainer && params.lowFrequencyContainer->present == LowFrequencyContainer_PR_basicVehicleContainerLowFrequency)
    {
        const auto& lfc = params.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency;
        msg->has_low_frequency_container = true;

        msg->low_frequency_container.vehicle_role.value = lfc.vehicleRole;
        msg->low_frequency_container.exterior_lights.value = 0;
        if (lfc.exteriorLights.size == 1)
        {
            // need to reverse bits from asn1c bit string to match our bit masks
            msg->low_frequency_container.exterior_lights.value = reverse_byte(lfc.exteriorLights.buf[0]);
        }

        for (int i = 0; i < lfc.pathHistory.list.count; ++i)
        {
            const PathPoint_t* asn1_path_point = lfc.pathHistory.list.array[i];
            ros_etsi_its_msgs::msg::PathPoint path_point;

            path_point.path_position.delta_latitude = asn1_path_point->pathPosition.deltaLatitude;
            path_point.path_position.delta_longitude = asn1_path_point->pathPosition.deltaLongitude;
            path_point.path_position.delta_altitude = asn1_path_point->pathPosition.deltaAltitude;

            path_point.path_delta_time.value = ros_etsi_its_msgs::msg::PathDeltaTime::UNAVAILABLE;
            if (asn1_path_point->pathDeltaTime)
            {
                path_point.path_delta_time.value = *(asn1_path_point->pathDeltaTime);
            }

            msg->low_frequency_container.path_history.points.push_back(path_point);
        }
    }

    return msg;
}

vanetza::asn1::r1::Vam convertVam(ros_etsi_its_msgs::msg::VAM::ConstSharedPtr ptr)
{
    vanetza::asn1::r1::Vam msg;

    ItsPduHeader_t& header = msg->header;
    header.protocolVersion = ptr->its_header.protocol_version;
    header.messageID = ptr->its_header.message_id;
    header.stationID = ptr->its_header.station_id;

    CoopAwareness_t& va = msg->vam;
    va.generationDeltaTime = ptr->generation_delta_time;

    BasicContainer_t& basic = va.vamParameters.basicContainer;
    basic.stationType = ptr->station_type.value;
    basic.referencePosition.altitude.altitudeValue = ptr->reference_position.altitude.value;
    basic.referencePosition.altitude.altitudeConfidence = ptr->reference_position.altitude.confidence;
    basic.referencePosition.latitude = ptr->reference_position.latitude;
    basic.referencePosition.longitude = ptr->reference_position.longitude;
    basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
        ptr->reference_position.position_confidence.semi_major_confidence;
    basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
        ptr->reference_position.position_confidence.semi_minor_confidence;
    basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation =
        ptr->reference_position.position_confidence.semi_major_orientation;

    HighFrequencyContainer_t& hfc = va.vamParameters.highFrequencyContainer;
    hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    BasicVehicleContainerHighFrequency_t& bvc = hfc.choice.basicVehicleContainerHighFrequency;
    bvc.heading.headingValue = ptr->high_frequency_container.heading.value;
    bvc.heading.headingConfidence = ptr->high_frequency_container.heading.confidence;
    bvc.speed.speedValue = ptr->high_frequency_container.speed.value;
    bvc.speed.speedConfidence = ptr->high_frequency_container.speed.confidence;
    bvc.driveDirection = ptr->high_frequency_container.drive_direction.value;
    bvc.longitudinalAcceleration.longitudinalAccelerationValue =
        ptr->high_frequency_container.longitudinal_acceleration.value;
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence =
        ptr->high_frequency_container.longitudinal_acceleration.confidence;
    bvc.curvature.curvatureValue = ptr->high_frequency_container.curvature.value;
    bvc.curvature.curvatureConfidence = ptr->high_frequency_container.curvature.confidence;
    bvc.curvatureCalculationMode = ptr->high_frequency_container.curvature_calculation_mode.value;
    bvc.yawRate.yawRateValue = ptr->high_frequency_container.yaw_rate.value;
    bvc.yawRate.yawRateConfidence = ptr->high_frequency_container.yaw_rate.confidence;
    bvc.vehicleLength.vehicleLengthValue = ptr->high_frequency_container.vehicle_length.value;
    bvc.vehicleLength.vehicleLengthConfidenceIndication =
        ptr->high_frequency_container.vehicle_length.confidence_indication;
    bvc.vehicleWidth = ptr->high_frequency_container.vehicle_width.value;
    if (ptr->high_frequency_container.has_acceleration_control)
    {
        bvc.accelerationControl = vanetza::asn1::allocate<AccelerationControl_t>();
        bvc.accelerationControl->buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
        bvc.accelerationControl->size = 1;
        bvc.accelerationControl->buf[0] = reverse_byte(ptr->high_frequency_container.acceleration_control.value);
    }

    if (ptr->has_low_frequency_container)
    {
        va.vamParameters.lowFrequencyContainer = vanetza::asn1::allocate<LowFrequencyContainer_t>();
        va.vamParameters.lowFrequencyContainer->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;

        BasicVehicleContainerLowFrequency& bvcl = va.vamParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency;
        bvcl.vehicleRole = ptr->low_frequency_container.vehicle_role.value;
        bvcl.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
        bvcl.exteriorLights.size = 1;
        bvcl.exteriorLights.buf[0] = reverse_byte(ptr->low_frequency_container.exterior_lights.value);

        for (const ros_etsi_its_msgs::msg::PathPoint& path_point : ptr->low_frequency_container.path_history.points)
        {
            PathPoint_t* pathPoint = vanetza::asn1::allocate<PathPoint_t>();
            if (path_point.path_delta_time.value != ros_etsi_its_msgs::msg::PathDeltaTime::UNAVAILABLE)
            {
                pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
                *pathPoint->pathDeltaTime = path_point.path_delta_time.value;
            }
            pathPoint->pathPosition.deltaAltitude = path_point.path_position.delta_altitude;
            pathPoint->pathPosition.deltaLatitude = path_point.path_position.delta_latitude;
            pathPoint->pathPosition.deltaLongitude = path_point.path_position.delta_longitude;
            ASN_SEQUENCE_ADD(&bvcl.pathHistory, pathPoint);
        }
    }

    return msg;
}

} // namespace v2x_stack_btp


