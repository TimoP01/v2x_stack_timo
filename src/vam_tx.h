#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <v2x_stack_btp/msg/btp_data_indication.hpp>
#include <ros_etsi_its_msgs/msg/cpm.hpp>
#include <cstdint>

namespace v2x_stack_btp
{

class CpRxNode : public rclcpp::Node
{
public:
    //explicit CpRxNode(const rclcpp::NodeOptions & options);
    //void onIndication(const msg::BtpDataIndication::ConstSharedPtr);
    uint16_t port_;
    rclcpp::Subscription<msg::BtpDataIndication>::SharedPtr sub_btp_;    
    std::shared_ptr<rclcpp::Publisher<ros_etsi_its_msgs::msg::CPM>> pub_cpm_;
    rclcpp::Node::SharedPtr node_;
private:
    
    //void publish(const vanetza::asn1::r1::Cpm&);


};
}