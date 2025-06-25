#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "etsi_its_vam_ts_msgs/msg/vam.hpp"  // Where is the correct path for vam header?

using std::placeholders::_1;

class VamTx : public rclcpp::Node
{
public:
    VamTx() : Node("vam_tx_node")
    {
        // Subscription auf "position"
        position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/NavSatFix", 10, std::bind(&VamTx::position_update_callback, this, _1));

        // Subscription auf "heading"
        heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "heading", 10, std::bind(&VamTx::heading_update_callback, this, _1));

        // Publisher für ros2vam Nachricht

        vam_pub_ = this->create_publisher<etsi_its_vam_ts_msgs::msg::VAM>("etsi_its_conversion/vam_ts/in", 10);
        //publish_vam_message(latest_latitude_, latest_longitude_, latest_altitude_, latest_heading_);

        // Timer: alle 1 Sekunde ros2vam Nachricht senden
        rclcpp::TimerBase::SharedPtr timer;
        timer = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                std::cout << "Timer" << std::endl;
                this->publish_vam_message(latest_latitude_, latest_longitude_, latest_altitude_, latest_heading_);
            });

        RCLCPP_INFO(this->get_logger(), "VamTx Node gestartet.");
    }

    void publish_vam_message(double latitude, double longitude, double altitude, double heading)
    {

         std::cout << "Publish" << std::endl;
        // if (!position_received_ || !heading_received_) {
        //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        //                          "Warte auf erste Position und Heading-Daten...");
        //     return;
        // }

        etsi_its_vam_ts_msgs::msg::VAM vam_msg;
        vam_msg.header.value.protocol_version.value = 3;
        vam_msg.header.value.message_id.value = 16;
        vam_msg.header.value.station_id.value = 1994;

        // vam_msg.vam.vam_parameters = etsi_its_vam_ts_msgs::msg::VamParameters();
        // vam_msg.vam.vam_parameters.basic_container = etsi_its_vam_ts_msgs::msg::BasicContainer();
        // vam_msg.vam.vam_parameters.vru_high_frequency_container = etsi_its_vam_ts_msgs::msg::VruHighFrequencyContainer();
        

        vam_msg.vam.vam_parameters.basic_container.station_type.value = 0;

        // vam_msg.vam.vam_parameters.basic_container.reference_position.latitude.value = latitude;
        // vam_msg.vam.vam_parameters.basic_container.reference_position.longitude.value = longitude;

        // vam_msg.vam.vam_parameters.basic_container.reference_position.altitude.altitude_value.value = altitude;
        // vam_msg.vam.vam_parameters.basic_container.reference_position.altitude.altitude_confidence.value = 0;

        // Statische Positionszuweisung zum testen
        vam_msg.vam.vam_parameters.basic_container.reference_position.latitude.value = 555555;
        vam_msg.vam.vam_parameters.basic_container.reference_position.longitude.value = 999999;
        vam_msg.vam.vam_parameters.basic_container.reference_position.altitude.altitude_value.value = 222;

        vam_msg.vam.vam_parameters.vru_high_frequency_container.heading.value.value = heading;

        etsi_its_vam_ts_msgs::msg::Wgs84AngleConfidence wgs_conf;
        wgs_conf.value = 127;

        vam_msg.vam.vam_parameters.vru_high_frequency_container.heading.set__confidence(wgs_conf);

        // etsi_its_vam_ts_msgs::msg::SpeedConfidence spe_conf;
        // spe_conf.value = 60;
        vam_msg.vam.vam_parameters.vru_high_frequency_container.speed.speed_confidence.value = 60; //Statischer Wert zum Testen
        
        vam_pub_->publish(vam_msg);

        RCLCPP_INFO(this->get_logger(), "Custom VAM gesendet: [%.6d, %.6d, %.6d | %.2d]",
        vam_msg.vam.vam_parameters.basic_container.reference_position.latitude.value,
        vam_msg.vam.vam_parameters.basic_container.reference_position.longitude.value,
        vam_msg.vam.vam_parameters.basic_container.reference_position.altitude.altitude_value.value,
        vam_msg.vam.vam_parameters.vru_high_frequency_container.heading.value.value);
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
        publish_vam_message(latest_latitude_, latest_longitude_, latest_altitude_, latest_heading_);

        RCLCPP_INFO(this->get_logger(), "Heading empfangen: %.2f Grad", msg->data);
    }


    // Membervariablen
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;

    rclcpp::Publisher<etsi_its_vam_ts_msgs::msg::VAM>::SharedPtr vam_pub_;


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

/* Temporary Comments for Console Commands

    Build package:

    colcon build --packages-select v2x_stack_btp

    #################################

    In colcon_ws/trimble:
    ros2 bag play rosbag2_2025_05_23-10_04_24_0.mcap

    ros2 topic echo /NavSatFix 

    oder

    ros2 topic echo /heading

    #################################
    
    In colcon_ws:

    . install/setup.bash        in beiden Terminals

    ros2 launch v2x_stack_btp  btp_launch.py

    ros2 run v2x_stack_btp v2x_stack_btp_va_node
*/