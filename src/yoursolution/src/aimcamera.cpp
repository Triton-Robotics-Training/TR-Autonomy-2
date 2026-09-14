#include "aimcamera.h"

#include <chrono>
#include <functional>
#include <algorithm>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AimCamera>());
    rclcpp::shutdown();
    return 0;
}

AimCamera::AimCamera() : Node("aimcamera") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("desired_angle", 10);

    robotcam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "robotcam", 10, std::bind(&AimCamera::robotcam_callback, this, std::placeholders::_1));

    current_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "current_angle", 10, std::bind(&AimCamera::current_angle_callback, this, std::placeholders::_1));

    scored_point_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "scored_point", 10, std::bind(&AimCamera::scored_point_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AimCamera::timer_callback, this));
}

void AimCamera::timer_callback() {
    std_msgs::msg::Float32 desired_angle = calculate_desired_angle();
    publisher_->publish(desired_angle);
}

void AimCamera::robotcam_callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    current_robotcam_ = *msg;
}

void AimCamera::current_angle_callback(std_msgs::msg::Float32::ConstSharedPtr msg) {
    current_angle_.data = msg->data;
}

void AimCamera::scored_point_callback(std_msgs::msg::Empty::ConstSharedPtr msg) {
    current_scored_point_ = *msg;
}

std_msgs::msg::Float32 AimCamera::calculate_desired_angle() {
    std_msgs::msg::Float32 desired_angle;
    desired_angle.data = current_angle_.data;

    // edge case
    if (current_robotcam_.data.empty()) {
        return desired_angle;
    }

    // image -> BGR8
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(current_robotcam_, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& error) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", error.what());
        return desired_angle;
    }

    // BGR8 -> HSV
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // HSV -> binary mask
    cv::Mat red_one;
    cv::Mat red_two;
    cv::Mat red_mask;

    cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), red_one);
    cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), red_two);

    cv::bitwise_or(red_one, red_two, red_mask);

    /* debugging stuff */
    constexpr bool show_debug_view = true;
    if (show_debug_view) {
        cv::imshow("robot camera", cv_ptr->image);
        cv::imshow("red mask", red_mask);

        cv::waitKey(1);
    }
    /* debugging stuff */

    // binary mask -> x coord (image moments)
    cv::Moments moments = cv::moments(red_mask, true);
    if (moments.m00 == 0.0) {
        return desired_angle;
    }

    const double x = moments.m10 / moments.m00;
    const double image_center = static_cast<double>(cv_ptr->image.cols) / 2.0;

    // params
    constexpr double maximum_angle = 3.14159265359 / 4.0;
    constexpr double gain = 0.5;

    // calculate error and command value
    const double error = x - image_center;
    const double angle_error = (error / image_center) * maximum_angle;

    double command = static_cast<double>(current_angle_.data) - gain * angle_error;
    command = std::clamp(command, -maximum_angle, maximum_angle);

    // convert to ros2 float32
    desired_angle.data = static_cast<float>(command);
    return desired_angle;
}
