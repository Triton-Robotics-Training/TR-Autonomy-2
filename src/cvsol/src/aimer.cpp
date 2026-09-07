#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ang_sub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ang_pub;
double pi = 3.141592653589793;
class Aimer : public rclcpp::Node {
public:
  Aimer() : Node("aimer") {
    img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "robotcam", 10, std::bind(&Aimer::img_callback, this, _1));
    ang_pub =
        this->create_publisher<std_msgs::msg::Float32>("desired_angle", 10);
    ang_sub = this->create_subscription<std_msgs::msg::Float32>(
        "current_angle", 10, std::bind(&Aimer::ang_callback, this, _1));
  }

private:
  // make a flag for the current angle
  bool has_ang = false;
  double cur_ang = 0;
  void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // process the image from the topic

    // if no angle, just return
    if (!has_ang) {
      return;
    }

    // toCvCopy returns the shared ptr, so I need -> to get the image
    cv::Mat frame =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    // get the frame center
    int f_c_x = frame.cols / 2;
    // BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(frame, hsv_img, cv::COLOR_BGR2HSV);

    // defining HSV ranges 3 params are H,S,V | Hue is 0 -179
    cv::Scalar lower_red1(0, 100, 100); // essentially just an array
    cv::Scalar upper_red1(20, 255, 255);
    cv::Scalar lower_red2(165, 100, 100);
    cv::Scalar upper_red2(179, 255, 255);

    // get the mask using the color ranges
    cv::Mat mask1;
    cv::Mat mask2;
    // src is the input image, which is called mat(rix) in cv somehow, and dst
    // is the output img
    cv::inRange(hsv_img, lower_red1, upper_red1, mask1);
    cv::inRange(hsv_img, lower_red2, upper_red2, mask2);

    // merge two masks
    cv::Mat mask_all;
    cv::bitwise_or(mask1, mask2, mask_all);

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    // cv::RETR_EXTERNAL gets the outer edge of the blob and ignore what's
    // nested inside. cv::CHAIN_APPROX_SIMPLE saves memory by only recording the
    // important
    cv::findContours(mask_all, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // get largest area
    if (contours.empty()) {
      return;
    }

    int largest_countor_index = 0;
    double largest_area = 0;
    for (size_t i = 0; i < contours.size(); i++) {
      double area = cv::contourArea(
          contours[i]); // get the individual contour and find area

      if (area > largest_area) {
        largest_countor_index = i;
        largest_area = area;
      }
    }

    // some calc 3 thing
    cv::Moments m = cv::moments(contours[largest_countor_index]);
    if (m.m00 != 0) { // m10 and m01 has something todo with the distribution
                      // relative to xy
      int center_x = m.m10 / m.m00;
      int center_y = m.m01 / m.m00;
      RCLCPP_INFO(this->get_logger(), "Target: (%d, %d)", center_x, center_y);

      // publish a desire angle. if it's on the right side of the screen, make
      // it yaw right, vise versa
      int dead_band = (int)(frame.size().width * 0.05); // 5 percent error

      double step_ang = -0.04; // in randians, reversed

      if (abs(center_x - f_c_x) > dead_band) {
        std_msgs::msg::Float32 msg_out;
        if (center_x > f_c_x) {
          msg_out.data = cur_ang + step_ang;
        } else {
          msg_out.data = cur_ang - step_ang;
        }
        ang_pub->publish(msg_out);
      }
    }
  }

  void ang_callback(const std_msgs::msg::Float32 msg) {
    cur_ang = msg.data;
    if (!has_ang) {
      has_ang = true;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<Aimer>();

  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
