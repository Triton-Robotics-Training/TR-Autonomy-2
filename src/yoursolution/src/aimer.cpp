#include <cmath>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;

class Aimer : public rclcpp::Node
{
public:
  Aimer() : Node("aimer")
  {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/robotcam", 10, std::bind(&Aimer::on_image, this, _1));
    angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/current_angle", 10, std::bind(&Aimer::on_angle, this, _1));
    angle_pub_ = create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);
  }

private:
  void on_angle(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_angle_ = msg->data;
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask);

    cv::imshow("mask", mask);
    cv::waitKey(1);

    cv::Moments m = cv::moments(mask, true);
    if (m.m00 == 0) {
      return;
    }
    double cx = m.m10 / m.m00;

    double offset_px = cx - 320;
    double error_rad = -offset_px * (M_PI / 2) / 640;

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(current_angle_ + error_rad);
    angle_pub_->publish(out);

    RCLCPP_INFO(get_logger(), "cx=%.0f offset=%.0fpx desired=%.3f", cx, offset_px, out.data);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  double current_angle_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Aimer>());
  rclcpp::shutdown();
  return 0;
}