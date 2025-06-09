#include "ca_message.h"
#include "ca_rx.h"
#include <vanetza/btp/ports.hpp>

namespace v2x_stack_btp
{

CaRxNode::VaRxNode(const rclcpp::NodeOptions & options)
: Node("va_rx_node", options)
{
}

void VaRxNode::onIndication(msg::BtpDataIndication::ConstSharedPtr indication)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Indication VA");
    if (indication->btp_type == msg::BtpDataIndication::BTP_TYPE_B && indication->destination_port == 2001)
    {
        /*
        vanetza::asn1::r1::Cam cam;
        if (cam.decode(indication->data))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded CA");
            publish(cam);
        }*/
        vanetza::asn1::r1::Vam vam;// = nullptr;
        const std::vector<unsigned char>& payload = indication->data;
        const uint8_t* buffer = payload.data();


        auto ok = vanetza::asn1::decode_per((asn_TYPE_descriptor_t&)(asn_DEF_VAM), (void**)(&vam), (const void*)(buffer), indication->data.size());
    
        if (ok)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Well decoded VA");
            publish(vam);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ASN.1 decoding failed, drop received VAM");            
        }
    }
}

void VaRxNode::publish(const vanetza::asn1::r1::Vam asn1)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entering publish method");
    std::string error_msg;
    auto msg = convertVam(asn1, &error_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Try to Publish VA");


    if (msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "There is VA");
        node_ = std::make_shared<rclcpp::Node>("vam_rx");
        pub_vam_ = node_->create_publisher<ros_etsi_its_msgs::msg::VAM>("vam_received", 20);
    
        pub_vam_->publish(*msg);
    } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "VAM not correct");
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error cause: %s", error_msg.c_str());
    }
}

} // namespace v2x_stack_btp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting VA RX");

   auto node = std::make_shared<v2x_stack_btp::VaRxNode>(rclcpp::NodeOptions());
   auto subscription = node->create_subscription<v2x_stack_btp::msg::BtpDataIndication>("btp_data", 20, std::bind(&v2x_stack_btp::VaRxNode::onIndication, node, std::placeholders::_1));

   rclcpp::spin(node);
   rclcpp::shutdown();

    return 0;
}