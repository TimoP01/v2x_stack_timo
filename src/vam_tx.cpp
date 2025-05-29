#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <etsi_its_msgs/etsi_its_vam_ts_msgs/msg vam_msg>

using std::placeholders::_1;

class VamTx : public rclcpp::Node
{
public:
    VamTx() : Node("vam_tx_node")
    {
        // Subscription auf "position"
        position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "position", 10, std::bind(&VamTx::position_update_callback, this, _1));

        // Subscription auf "heading"
        heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "heading", 10, std::bind(&VamTx::heading_update_callback, this, _1));

        // Publisher für ros2vam Nachricht
        vam_pub_ = this->create_publisher<etsi_its_cam_msgs::vam_ts_msgs::VAM>("ros2vam", 10);

        // Timer: alle 1 Sekunde ros2vam Nachricht senden
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&VamTx::publish_vam_message, this));

        RCLCPP_INFO(this->get_logger(), "VamTx Node gestartet.");
    }
    void publish_custom_vam_message(double latitude, double longitude, double altitude, double heading)
    {
        
        etsi_its_cam_msgs::vam_ts_msgs::VAM vam_msg;
        vam_msg.header.value.protocolVersion.value = 3;
        vam_msg.header.value.messageId = 16;
        vam_msg.header.value.stationId.value = ?;

        //payload
        // vam_msg.vam.generationalDeltaTime.value = ?; 
        // vam_msg.vam.vamParameters.basicContainer.stationType.value = ?;
        vam_msg.vam.vamParameters.basicContainer.referencePosition.latitude.value = latitude;
        vam_msg.vam.vamParameters.basicContainer.referencePosition.longitude.value = longitude;
        // vam_msg.vam.vamParameters.basicContainer.referencePosition.altitude.altitudeValue = 800001; 
        // vam_msg.vam.vamParameters.basicContainer.referencePosition.altitude.altitudeConfidence.value = 15; 
        // vam_msg.vam.vamParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisLength.value = 4095;
        // vam_msg.vam.vamParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorAxisLength.value = 4095; 
        // vam_msg.vam.vamParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation.value = 3601; 
        vam_msg.vam.amParameters.VruHighFrequencyContainer.heading.value.value = heading;
        // vam_msg.vam.vamParameters.VruHighFrequencyContainer.heading.confidence.value = ;
        // vam_msg.vam.vamParameters.VruHighFrequencyContainer.speed.speedvalue.value = ; 
        // vam_msg.vam.vamParameters.VruHighFrequencyContainer.speed.speedconfidence.value = ; 

        // vam_msg.vam.vamParameters.VruHighFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationValue.value = ;
        // vam_msg.vam.vamParameters.VruHighFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationConfidence.value = ;

        vam_pub_->publish(vam_msg);

        RCLCPP_INFO(this->get_logger(), "Custom VAM gesendet: [%.6f, %.6f, %.2f | %.2f°]",
                    vam_msg.latitude, vam_msg.longitude, vam_msg.altitude, vam_msg.heading);
    }
private:
    void position_update_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        latest_latitude_ = msg->latitude;
        latest_longitude_ = msg->longitude;
        latest_altitude_ = msg->altitude;
        position_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Position empfangen: [%.6f, %.6f, %.2f]",
                    msg->latitude, msg->longitude, msg->altitude);
    }

    void heading_update_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        latest_heading_ = msg->data;
        heading_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Heading empfangen: %.2f Grad", msg->data);
    }

    void publish_vam_message()
    {
        if (!position_received_ || !heading_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Warte auf erste Position und Heading-Daten...");
            return;
        }

        etsi_its_cam_msgs::vam_ts_msgs::VAM vam_msg;
        vam_msg.latitude = latest_latitude_;
        vam_msg.longitude = latest_longitude_;
        vam_msg.altitude = latest_altitude_;
        vam_msg.heading = latest_heading_;

        vam_pub_->publish(vam_msg);

        RCLCPP_INFO(this->get_logger(), "VAM gesendet: [%.6f, %.6f, %.2f | %.2f°]",
                    vam_msg.latitude, vam_msg.longitude, vam_msg.altitude, vam_msg.heading);
    }

    // Membervariablen
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
    rclcpp::Publisher<etsi_its_cam_msgs::vam_ts_msgs::VAM>::SharedPtr vam_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double latest_latitude_ = 0.0;
    double latest_longitude_ = 0.0;
    double latest_altitude_ = 0.0;
    double latest_heading_ = 0.0;

    bool position_received_ = false;
    bool heading_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VamTx>());
    rclcpp::shutdown();
    return 0;
}