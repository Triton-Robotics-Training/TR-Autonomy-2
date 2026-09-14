#ifndef AIMCAMERA_H
#define AIMCAMERA_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sensor_msgs/msg/image.hpp"

class AimCamera : public rclcpp::Node {
public:
    AimCamera();

private:

    void timer_callback();

    void robotcam_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void current_angle_callback(std_msgs::msg::Float32::ConstSharedPtr msg);
    void scored_point_callback(std_msgs::msg::Empty::ConstSharedPtr msg);

    std_msgs::msg::Float32 calculate_desired_angle();

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robotcam_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr scored_point_sub_;

    sensor_msgs::msg::Image current_robotcam_;
    std_msgs::msg::Float32 current_angle_;
    std_msgs::msg::Empty current_scored_point_;

    rclcpp::TimerBase::SharedPtr timer_;

};

#endif
