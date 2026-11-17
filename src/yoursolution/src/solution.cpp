#include <algorithm>
#include <cmath>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

class Solution : public rclcpp::Node {
public:
  Solution() : Node("solution") {
    // Flip to +1.0 if the robot turns away from the target.
    sign_ = declare_parameter("sign", -1.0);
    gain_ = declare_parameter("gain", 0.6);
    min_pixels_ = declare_parameter("min_pixels", 50);

    pub_ = create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);

    angle_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/current_angle", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { current_ = msg->data; });

    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/robotcam", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(msg); });
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    // BGR -> HSV, then mask red (hue wraps around 0/180, so two ranges).
    cv::Mat hsv, low, high, mask;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), low);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(179, 255, 255), high);
    cv::bitwise_or(low, high, mask);

    // Remove speckle.
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    // Skip if the target isn't visible (avoids divide-by-zero).
    if (cv::countNonZero(mask) < min_pixels_) {
      return;
    }

    cv::Moments m = cv::moments(mask, true);
    if (m.m00 <= 0.0) {
      return;
    }
    double cx = m.m10 / m.m00;

    // Pinhole camera: 90 deg FOV => focal length = width / 2 pixels.
    double half_w = mask.cols / 2.0;
    double dx = cx - half_w;
    double delta = std::atan2(dx, half_w);

    double desired = current_ + sign_ * gain_ * delta;
    desired = std::clamp(desired, -M_PI / 4.0, M_PI / 4.0);

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(desired);
    pub_->publish(out);
  }

  double sign_, gain_;
  int min_pixels_;
  double current_ = 0.0;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Solution>());
  rclcpp::shutdown();
  return 0;
}