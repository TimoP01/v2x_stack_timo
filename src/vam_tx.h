#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "etsi_its_vam_ts_msgs/msg/vam.hpp"

namespace v2x_stack_btp
{

class VamTxNode : public rclcpp::Node
{
public:
    explicit VamTxNode(const rclcpp::NodeOptions & options);

    void onPosition(sensor_msgs::msg::NavSatFix::ConstSharedPtr position);
    void onHeading(std_msgs::msg::Float64::ConstSharedPtr heading);
    void publish();

private:
    rclcpp::Publisher<etsi_its_vam_ts_msgs::msg::VAM>::SharedPtr vam_pub_;

    double latest_latitude_ = 0.0;
    double latest_longitude_ = 0.0;
    double latest_altitude_ = 0.0;
    double latest_heading_ = 0.0;

    bool position_received_ = false;
    bool heading_received_ = false;
};

} // namespace v2x_stack_btp
