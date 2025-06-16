#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vanetza/asn1/vam.hpp>
#include <v2x_stack_btp/msg/btp_data_indication.hpp>
#include <ros_etsi_its_msgs/msg/vam.hpp>
#include <cstdint>

namespace v2x_stack_btp
{


class VaRxNode : public rclcpp::Node
{
public:
    explicit VaRxNode(const rclcpp::NodeOptions & options);

    void onIndication(const msg::BtpDataIndication::ConstSharedPtr);

private:
    
    void publish(const vanetza::asn1::r1::Vam);

    uint16_t port_;
    rclcpp::Subscription<msg::BtpDataIndication>::SharedPtr sub_btp_;
    //rclcpp::Publisher<ros_etsi_its_msgs::msg::CAM>pub_cam_;
    std::shared_ptr<rclcpp::Publisher<ros_etsi_its_msgs::msg::VAM>> pub_vam_;
    rclcpp::Node::SharedPtr node_;
};

} // namespace v2x_stack_btp