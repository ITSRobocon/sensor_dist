
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"

class distNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    sensor_msgs::msg::LaserScan laser_data;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr dist_x, dist_y;

    double laser_head;
public:
    distNode() : Node("dist_node") {

        this->declare_parameter("laser_heading", (double)0.8517207);
        laser_head = this->get_parameter("laser_heading").as_double();

        laser_sub   = this->create_subscription<sensor_msgs::msg::LaserScan> (
                      "scan", 10, std::bind(&distNode::laser_callback, this, std::placeholders::_1));

        dist_x = this->create_publisher<sensor_msgs::msg::Range>("dist_sensor/x", rclcpp::SensorDataQoS());
        dist_y = this->create_publisher<sensor_msgs::msg::Range>("dist_sensor/y", rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(), "dist_node Start!");
        std::thread(&distNode::main, this).detach();
    };

    virtual ~distNode() {
        RCLCPP_INFO(this->get_logger(), "dist_node Stop!");
    };

    void main() {
        rclcpp::Rate main(2);

        while(rclcpp::ok()) {

            if(laser_data.ranges.size() < 10) continue;

            sensor_msgs::msg::Range sensor_x, sensor_y;

            sensor_x.radiation_type = sensor_x.INFRARED;
            sensor_x.header.frame_id = "laser";
            sensor_x.field_of_view = 0.0349066;
            sensor_x.min_range = laser_data.range_min;
            sensor_x.max_range = laser_data.range_max;

            sensor_y = sensor_x;

            float x_to_laser_angle = - (M_PI/2.0) + laser_head;
            float y_to_laser_angle = laser_head;

            // std::cout << "X: " << x_to_laser_angle << std::endl;
            // std::cout << "Y: " << y_to_laser_angle << std::endl;

            
            if (x_to_laser_angle < laser_data.angle_min || 
                x_to_laser_angle > laser_data.angle_max) {
                    sensor_x.range = INFINITY;
            }
            else {
                int x = (x_to_laser_angle - laser_data.angle_min) / laser_data.angle_increment;

                int num = 1;
                double sum = laser_data.ranges.at(x);
                for(int i = 1; i<5; i++) {
                    sum += laser_data.ranges.at(x+i) + laser_data.ranges.at(x-i);
                    num += 2;
                }
                sensor_x.range = sum/(double)num;
            }

            if (y_to_laser_angle < laser_data.angle_min || 
                y_to_laser_angle > laser_data.angle_max) {
                    sensor_y.range = INFINITY;
            }
            else {
                int y = (y_to_laser_angle - laser_data.angle_min) / laser_data.angle_increment;

                int num = 1;
                double sum = laser_data.ranges.at(y);
                for(int i = 1; i<4; i++) {
                    sum += laser_data.ranges.at(y+i) + laser_data.ranges.at(y-i);
                    num += 2;
                }
                sensor_y.range = sum/(double)num;
            }

            dist_x->publish(sensor_x);
            dist_y->publish(sensor_y);

            main.sleep();
        }
    };

    void laser_callback(const sensor_msgs::msg::LaserScan& msg) {
        laser_data = msg;
    }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<distNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}