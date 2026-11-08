#include <algorithm>
#include <cstdio>
#include <memory>
 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
 
namespace {
 
// OpenCV hue runs 0-179 (not 0-360), saturation and value run 0-255.
const int kMaxH = 179;
const int kMaxSV = 255;
const char *kCaptureWindow = "Capture";
const char *kDetectionWindow = "Mask";
 
// Trackbars write straight into these. Start on the low red range
// (hue 0-10); tune, press 'p' to print, then repeat for 170-179.
int low_H = 0, low_S = 100, low_V = 100;
int high_H = 10, high_S = kMaxSV, high_V = kMaxSV;
 
// Keep each low bound below its high bound.
void on_low_H(int, void *) {
  low_H = std::min(high_H - 1, low_H);
  cv::setTrackbarPos("Low H", kDetectionWindow, low_H);
}
void on_high_H(int, void *) {
  high_H = std::max(high_H, low_H + 1);
  cv::setTrackbarPos("High H", kDetectionWindow, high_H);
}
void on_low_S(int, void *) {
  low_S = std::min(high_S - 1, low_S);
  cv::setTrackbarPos("Low S", kDetectionWindow, low_S);
}
void on_high_S(int, void *) {
  high_S = std::max(high_S, low_S + 1);
  cv::setTrackbarPos("High S", kDetectionWindow, high_S);
}
void on_low_V(int, void *) {
  low_V = std::min(high_V - 1, low_V);
  cv::setTrackbarPos("Low V", kDetectionWindow, low_V);
}
void on_high_V(int, void *) {
  high_V = std::max(high_V, low_V + 1);
  cv::setTrackbarPos("High V", kDetectionWindow, high_V);
}
 
}  // namespace
 
class HsvTuner : public rclcpp::Node {
public:
  HsvTuner() : Node("hsv_tuner") {
    cv::namedWindow(kCaptureWindow);
    cv::namedWindow(kDetectionWindow);
 
    cv::createTrackbar("Low H", kDetectionWindow, &low_H, kMaxH, on_low_H);
    cv::createTrackbar("High H", kDetectionWindow, &high_H, kMaxH, on_high_H);
    cv::createTrackbar("Low S", kDetectionWindow, &low_S, kMaxSV, on_low_S);
    cv::createTrackbar("High S", kDetectionWindow, &high_S, kMaxSV, on_high_S);
    cv::createTrackbar("Low V", kDetectionWindow, &low_V, kMaxSV, on_low_V);
    cv::createTrackbar("High V", kDetectionWindow, &high_V, kMaxSV, on_high_V);
 
    // The ROS subscription replaces the tutorial's VideoCapture + while loop:
    // this callback runs once per frame instead.
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/robotcam", 10,
        [this](sensor_msgs::msg::Image::SharedPtr msg) { image_callback(msg); });
 
    RCLCPP_INFO(get_logger(), "Keys: p = print thresholds, s = save frame.png, q/Esc = quit");
  }
 
  ~HsvTuner() { cv::destroyAllWindows(); }
 
private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
 
    cv::Mat hsv, mask;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V),
                cv::Scalar(high_H, high_S, high_V), mask);
 
    cv::imshow(kCaptureWindow, cv_ptr->image);
    cv::imshow(kDetectionWindow, mask);
 
    // waitKey is required, or the windows never refresh. It returns -1 when
    // no key is pressed, which & 0xFF turns into 255 (matches nothing below).
    const int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) {
      rclcpp::shutdown();
    } else if (key == 's') {
      cv::imwrite("frame.png", cv_ptr->image);
      RCLCPP_INFO(get_logger(), "Saved frame.png in the current directory");
    } else if (key == 'p') {
      std::printf("cv::inRange(hsv, cv::Scalar(%d, %d, %d), cv::Scalar(%d, %d, %d), mask);\n",
                  low_H, low_S, low_V, high_H, high_S, high_V);
    }
  }
 
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HsvTuner>());
  rclcpp::shutdown();
  return 0;
}
