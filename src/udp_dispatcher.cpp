#include "udp_dispatcher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <boost/make_shared.hpp>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

namespace v2x_stack_btp
{

UDPdispatcher::UDPdispatcher(const rclcpp::NodeOptions &options)
    : Node("udp_publisher", options), stop_thread_(false)
{
    this->declare_parameter<std::string>("originating_ip", "172.16.2.1");
    this->declare_parameter<int>("originating_port", 4400);
    this->declare_parameter<std::string>("destination_ip", "172.16.2.1");
    this->declare_parameter<int>("destination_port", 4401);

    this->get_parameter("originating_ip", originatingIp);
    this->get_parameter("originating_port", originatingPort);
    this->get_parameter("destination_ip", destinationIP);
    this->get_parameter("destination_port", destinationPort);

    RCLCPP_INFO(this->get_logger(), "UDP Dispatcher receiving on IP: %s, Port: %d", originatingIp.c_str(), originatingPort);
    RCLCPP_INFO(this->get_logger(), "UDP Dispatcher sending to IP: %s, Port: %d", destinationIP.c_str(), destinationPort);
    
    publisher = this->create_publisher<udp_msgs::msg::UdpPacket>("converter/udp/in", 10);
    node_ = std::make_shared<rclcpp::Node>("udp_publisher_node");
    publisher_ = node_->create_publisher<v2x_stack_btp::msg::CohdaInd>("udp_data", 10);

    recv_thread_ = std::thread(&UDPdispatcher::initialize, this);
}

UDPdispatcher::~UDPdispatcher()
{
    stop_thread_ = true;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

void UDPdispatcher::initialize()
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        RCLCPP_FATAL(this->get_logger(), "Error creating UDP socket");
        return;
    }

    struct sockaddr_in host_addr{}, sender_addr{};
    host_addr.sin_family = AF_INET;
    host_addr.sin_port = htons(originatingPort);
    host_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr *)&host_addr, sizeof(host_addr)) < 0) {
        close(sockfd);
        RCLCPP_FATAL(this->get_logger(), "Error binding socket");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "UDP socket bound. Waiting for packets...");

    char buffer[2048];
    socklen_t addr_len = sizeof(sender_addr);

    while (!stop_thread_) {
        ssize_t recv_len = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&sender_addr, &addr_len);

        if (recv_len < 0) {
            if (stop_thread_) break;  // clean exit
            RCLCPP_ERROR(this->get_logger(), "Error receiving data");
            continue;
        }

        if (sender_addr.sin_addr.s_addr == inet_addr(originatingIp.c_str())) {
            if (recv_len < sizeof(tUDPBTPMsgType) + sizeof(tUDPBTPDataIndHdr)) {
                RCLCPP_WARN(this->get_logger(), "Packet too short");
                continue;
            }

            tUDPBTPDataIndMsg *udpPackage = reinterpret_cast<tUDPBTPDataIndMsg *>(buffer);
            publish(udpPackage);

            udp_msgs::msg::UdpPacket ros_udp_msg;
            ros_udp_msg.address = originatingIp;
            ros_udp_msg.data.assign(buffer, buffer + recv_len);
            publisher->publish(ros_udp_msg);
        }
    }

    close(sockfd);
    RCLCPP_INFO(this->get_logger(), "UDP receive thread exited cleanly.");
}

void UDPdispatcher::publish(const tUDPBTPDataIndMsg *ind)
{
    auto ccu_ind = boost::make_shared<v2x_stack_btp::msg::CohdaInd>();

    ccu_ind->type.version = ind->Type.Version;
    ccu_ind->type.msg_id = ind->Type.MsgID;
    ccu_ind->type.msg_length = ntohs(ind->Type.MsgLen);

    ccu_ind->header.btp_type = ind->Hdr.BTPType;
    ccu_ind->header.pkt_transport = ind->Hdr.PktTransport;
    ccu_ind->header.traffic_class = ind->Hdr.TrafficClass;
    ccu_ind->header.max_pkt_life_time = ind->Hdr.MaxPktLifetime;
    ccu_ind->header.dest_port = ntohs(ind->Hdr.DestPort);
    ccu_ind->header.dest_info = ind->Hdr.DestInfo;

    int btpMsgSize = ntohs(ind->Type.MsgLen);
    ccu_ind->payload.resize(btpMsgSize);
    std::copy(ind->Payload, ind->Payload + btpMsgSize, ccu_ind->payload.begin());

    publisher_->publish(*ccu_ind);
}

} // namespace v2x_stack_btp

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<v2x_stack_btp::UDPdispatcher>(rclcpp::NodeOptions{});

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
