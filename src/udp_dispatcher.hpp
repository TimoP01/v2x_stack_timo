#ifndef UDP_DISPATCHER_H_
#define UDP_DISPATCHER_H_

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>

#include "ccu_udp_api.h"
#include "v2x_stack_btp/msg/cohda_ind.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

namespace v2x_stack_btp
{

class UDPdispatcher : public rclcpp::Node
{
public:
    explicit UDPdispatcher(const rclcpp::NodeOptions &options);
    ~UDPdispatcher();

    void publish(const tUDPBTPDataIndMsg* ind);

private:
    void initialize();

    // Threads & Flags
    std::thread recv_thread_;
    std::atomic_bool stop_thread_;

    // Networking
    struct sockaddr_in host_addr, ccu_addr;
    std::string ccu_ip;
    uint16_t ccu_port;
    uint16_t host_port;

    std::string originatingIp;
    int originatingPort;
    std::string destinationIP;
    int destinationPort;

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<v2x_stack_btp::msg::CohdaInd>::SharedPtr publisher_;
    rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr publisher;

    tUDPBTPDataIndMsg receivedPackage;
};

} // namespace v2x_stack_btp

#endif // UDP_DISPATCHER_H_
