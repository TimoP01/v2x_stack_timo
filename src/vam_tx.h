#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <etsi_its_vam_ts_msgs/msg/vam.hpp>

namespace v2x_stack_btp
{

class VamTx : public rclcpp::Node
{
public:
    VamTx();

private:
    void position_update_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void heading_update_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void publish_vam_message(double latitude, double longitude, double altitude, double heading);

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;

    // Publisher
    rclcpp::Publisher<etsi_its_vam_ts_msgs::msg::VAM>::SharedPtr vam_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Daten
    double latest_latitude_ = 0.0;
    double latest_longitude_ = 0.0;
    double latest_altitude_ = 0.0;
    double latest_heading_ = 0.0;

    bool position_received_ = false;
    bool heading_received_ = false;
};

} // namespace v2x_stack_btp
