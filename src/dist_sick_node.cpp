
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

inline double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class distNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr adc_sensor_sub;
    sensor_msgs::msg::LaserScan laser_data;


    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr dist_x, dist_y;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dist_pub;

    double dist_range_min, dist_range_max, dist_curr_min, dist_curr_max;
public:
    distNode() : Node("dist_node") {
        
        this->declare_parameter("dist_range_min", (double)0.02);
        dist_range_min = this->get_parameter("dist_range_min").as_double();

        this->declare_parameter("dist_range_max", (double)6.00);
        dist_range_max = this->get_parameter("dist_range_max").as_double();

        this->declare_parameter("dist_curr_min", (double)0.004);
        dist_curr_min = this->get_parameter("dist_curr_min").as_double();

        this->declare_parameter("dist_curr_max", (double)0.02);
        dist_curr_max = this->get_parameter("dist_curr_max").as_double();

        adc_sensor_sub   = this->create_subscription<std_msgs::msg::Float32MultiArray> (
                           "adc_sensor", rclcpp::SensorDataQoS(), std::bind(&distNode::laser_callback, this, std::placeholders::_1));

        dist_x = this->create_publisher<sensor_msgs::msg::Range>("sick_sensor/x", rclcpp::SensorDataQoS());
        dist_y = this->create_publisher<sensor_msgs::msg::Range>("sick_sensor/y", rclcpp::SensorDataQoS());
        dist_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("sick_sensor/float", rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(), "dist_node Start!");
    };

    virtual ~distNode() {
        RCLCPP_INFO(this->get_logger(), "dist_node Stop!");
    };

    void laser_callback(const std_msgs::msg::Float32MultiArray& msg) {
        float x_dist = map(msg.data.at(0), 
                           dist_curr_min , dist_curr_max, 
                           dist_range_min, dist_range_max);
        float y_dist = map(msg.data.at(1), 
                           dist_curr_min , dist_curr_max, 
                           dist_range_min, dist_range_max);

        std_msgs::msg::Float32MultiArray send_msg = std_msgs::msg::Float32MultiArray();
        send_msg.data.push_back(x_dist);
        send_msg.data.push_back(y_dist);
        dist_pub->publish(send_msg);

        auto range = sensor_msgs::msg::Range();
        range.radiation_type = range.INFRARED;
        range.min_range = 0.002;
        range.max_range = 6.000;
        range.field_of_view = 0.0174533;
        range.header.stamp = this->get_clock()->now();

        range.header.frame_id = "sick0";
        range.range = x_dist;
        dist_x->publish(range);
        
        range.header.frame_id = "sick1";
        range.range = y_dist;
        dist_y->publish(range);
    }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<distNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}