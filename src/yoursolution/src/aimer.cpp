#include <cmath>
#include <functional>
#include <memory>

#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"

using Float32 = std_msgs::msg::Float32;
using Image = sensor_msgs::msg::Image;

class Aimer : public rclcpp::Node {
 public:
  Aimer() : Node("aimer") {
    angle_sub_ = this->create_subscription<Float32>(
        "current_angle", 10, std::bind(&Aimer::angle_callback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<Image>(
        "robotcam", 1, std::bind(&Aimer::image_callback, this, std::placeholders::_1));
    angle_pub_ = this->create_publisher<Float32>("desired_angle", 10);
  }

 private:
  void angle_callback(const Float32::SharedPtr msg) {
    current_angle_ = msg->data;
  }

  void image_callback(const Image::SharedPtr msg) {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    cv::Mat hsv, low_red, high_red, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255), low_red);
    cv::inRange(hsv, cv::Scalar(170, 100, 50), cv::Scalar(179, 255, 255), high_red);
    cv::bitwise_or(low_red, high_red, mask);

    Float32 out;

    cv::Moments m = cv::moments(mask, true);
    if (m.m00 < 1) {
      out.data = 0.0f;
      angle_pub_->publish(out);
      return;
    }

    double cube_x = m.m10 / m.m00;
    double offset_px = cube_x - frame.cols / 2.0;

    double focal_px = (frame.cols / 2.0) / std::tan(CV_PI / 4);
    double offset_angle = std::atan(offset_px / focal_px);

    out.data = current_angle_ - offset_angle;
    angle_pub_->publish(out);
  }

  rclcpp::Subscription<Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Publisher<Float32>::SharedPtr angle_pub_;
  float current_angle_ = 0.0f;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Aimer>());
  rclcpp::shutdown();
  return 0;
}