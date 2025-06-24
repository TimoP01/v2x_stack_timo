#include "vam_tx.h"
#include <vanetza/btp/ports.hpp>  // falls benötigt

namespace v2x_stack_btp
{

VamTxNode::VamTxNode(const rclcpp::NodeOptions & options)
: Node("vam_tx_node", options)
{
}

void VamTxNode::onPosition(sensor_msgs::msg::NavSatFix::ConstSharedPtr position)
{
    if (position->status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
        RCLCPP_INFO(this->get_logger(), "Position empfangen: [%.6f, %.6f, %.2f]",
                    position->latitude, position->longitude, position->altitude);
        latest_latitude_ = position->latitude;
        latest_longitude_ = position->longitude;
        latest_altitude_ = position->altitude;
        position_received_ = true;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Kein GPS-Fix.");
    }
}

void VamTxNode::onHeading(std_msgs::msg::Float64::ConstSharedPtr heading)
{
    latest_heading_ = heading->data;
    heading_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Heading empfangen: %.2f Grad", heading->data);
}

void VamTxNode::publish()
{
    if (!position_received_ || !heading_received_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Warte auf erste Positions- und Heading-Daten...");
        return;
    }

    auto msg = std::make_shared<etsi_its_vam_ts_msgs::msg::VAM>();

    msg->header.value.protocol_version.value = 3;
    msg->header.value.message_id.value = 16;
    msg->header.value.station_id.value = 1994;

    msg->vam.vam_parameters.basic_container.reference_position.latitude.value = latest_latitude_;
    msg->vam.vam_parameters.basic_container.reference_position.longitude.value = latest_longitude_;

    msg->vam.vam_parameters.vru_high_frequency_container.heading.value.value = latest_heading_;

    RCLCPP_INFO(this->get_logger(), "VAM gesendet: [%.6f, %.6f | %.2f]",
        latest_latitude_, latest_longitude_, latest_heading_);

    if (!vam_pub_)
    {
        vam_pub_ = this->create_publisher<etsi_its_vam_ts_msgs::msg::VAM>("ros2vam", 10);
    }

    vam_pub_->publish(*msg);
}

} // namespace v2x_stack_btp

// ---------- main ----------
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting VAM TX node");

    auto node = std::make_shared<v2x_stack_btp::VamTxNode>(rclcpp::NodeOptions());

    auto sub_position = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "position", 10, std::bind(&v2x_stack_btp::VamTxNode::onPosition, node, std::placeholders::_1));

    auto sub_heading = node->create_subscription<std_msgs::msg::Float64>(
        "heading", 10, std::bind(&v2x_stack_btp::VamTxNode::onHeading, node, std::placeholders::_1));

    auto timer = node->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&v2x_stack_btp::VamTxNode::publish, node));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
