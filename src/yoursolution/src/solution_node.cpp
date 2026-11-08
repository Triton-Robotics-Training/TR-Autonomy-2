#include <algorithm> //used to limit target angle to [-pi/2, pi/2]
#include <cmath> //for pixel to angle conversion
#include <memory> //for smart pointers
 
#include <cv_bridge/cv_bridge.h> //converts from the ros2 image format to the OpenCV image format
#include <opencv2/opencv.hpp> //opencv library
#include <rclcpp/rclcpp.hpp> //ROS2 C++ client library
#include <sensor_msgs/msg/image.hpp> //message type of OpenCV image
#include <std_msgs/msg/float32.hpp> //message type for angle
 
class SolutionNode : public rclcpp::Node {
public:
  SolutionNode() : Node("yoursolution") {
    //publisher for desired angle
    pub_ = this->create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);

    //subscriber for current angle
    angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/current_angle", 10,
        [this](std_msgs::msg::Float32::SharedPtr msg) { current_angle_ = msg->data; });

    //subscriber for camera image
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/robotcam", 10,
        [this](sensor_msgs::msg::Image::SharedPtr msg) { image_callback(msg); });

    // How much of the distance between the center of the cube and the center of the image to correct for each frame before remeasuring. Lower values would be smoother movement, but slower to respond. Higher values would be more responsive, but could overshoot and oscillate.
    // The reason to not set to 1 and try to snap exactly to the center is that current angle is only updated at 30Hz, so in a real application the cube could move between frames and the robot would overshoot.
    gain_ = this->declare_parameter("gain", 0.7);
    
    // The range in which the robot will stop moving it it's this far from the x centroid. Don't need to make minor adjustments (jittering) if it's already close enough to the center.
    deadband_px_ = this->declare_parameter("deadband_px", 6.0);

    // The minimum size a cube has to be to be recognized as a target rather than noise
    min_pixels_ = this->declare_parameter("min_pixels", 50);
  }
 
private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); //convert to OpenCV image format from bgr8
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
 
    //Mat is the openCV image datatype (a tensor)
    //create 4 images:
    // low and high are the two thresholded images for the red color. Hue is a wheel and red sits at the seam of the hue range (0/180), so we threshold both ends and combine them
    // hsv is the original image converted to HSV (hue, saturation, color, value) space which is better for color thresholding than RGB because it separates color from brightness
    // mask is the binary segmentation mask after combining the binary masks from low and high.
    cv::Mat hsv, low, high, mask;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV); //populate hsv with the original image converted to HSV color space
 
    //Create the binary segmentation mask by first making a binary segmentation mask of the low and high red hue ranges
    //inRange is a function that takes an input image and thresholds it to create a binary mask where pixels within the specified range are set to 255 (white) and those outside the range are set to 0 (black)
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), low);
    cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(179, 255, 255), high);
    cv::bitwise_or(low, high, mask); //bitwise or means that if the pixel was segmented as part of the target for low or high, it will be part of the final mask
 
    //If there are less than min_pixels_ pixels in the mask, don't continue
    if (cv::countNonZero(mask) < min_pixels_) {
      return;
    }
 
    // calculate the x centroid, the centroid is the average of all points in a shape. It is the balancing point
    // To calculate it, we use moments
    // moments are a set of scalars that provide information about the shape of a binary image. The zeroth moment (m00) is the area (number of pixels) of the shape, the first moments (m10, m01) are the sums of the x and y coordinates of all pixels in the shape, and the centroid can be calculated as (m10/m00, m01/m00)
    cv::Moments m = cv::moments(mask, true);
    if (m.m00 <= 0.0) {
      return;
    }
    const double cx = m.m10 / m.m00; // m00 is the number of pixels in the target mask (blob), m10 is the sum of the x coordinates of all the pixels in the target mask, cx is the average x coordinate
 
    //calculate the pixel offset
    const double half_width = mask.cols / 2.0;  // focal length in px for a 90 deg FOV
    const double offset_px = cx - half_width;   // +: cube is right of center
 
    //If camera is centered within acceptable range of the cube, do nothing
    if (std::abs(offset_px) < deadband_px_) {
      return;
    }
 
    //convert pixel offset to angle using known formula
    const double error_rad = std::atan(offset_px / half_width);
 
    // Decreasing angle looks right, increasing looks left, so subtract.
    double target = current_angle_ - gain_ * error_rad;
    target = std::clamp(target, -0.9, 0.9); //restrict desired angle to [-pi/2, pi/2] so we don't turn more than 90 degrees in either direction
 
    //publish desired angle
    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(target);
    pub_->publish(out);
  }
 
  // private attributes of SolutionNode actually set values for the parameters declared in the constructor. They are initialized to default values, but can be changed by the user at runtime using ROS2 parameter services.
  double gain_{0.7};
  double deadband_px_{6.0};
  int min_pixels_{50};
  double current_angle_{0.0};
 
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};
 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SolutionNode>());
  rclcpp::shutdown();
  return 0;
}