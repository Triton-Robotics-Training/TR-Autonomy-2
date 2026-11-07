#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class YourSolution : public rclcpp::Node {
    public:
        YourSolution() : Node("your_solution") {
            camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/robotcam", 
                10, //queue depth
                std::bind(
                    &YourSolution::camera_callback,
                    this,
                    std::placeholders::_1
                )
            );
            RCLCPP_INFO(this->get_logger(), "YourSolution node started!");
        }
     private: 
        void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                // Convert ROS image into an OpenCV image
                cv_bridge::CvImagePtr cv_image =
                    cv_bridge::toCvCopy(msg, "bgr8");

                cv::Mat image = cv_image->image;
                // Convert BGR image to HSV
                cv::Mat hsv;
                cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
                cv::Mat mask1;
                cv::inRange(
                    hsv,
                    cv::Scalar(0,100,100),
                    cv::Scalar(10,255,255),
                    mask1
                );
                cv::Mat mask2;
                cv::inRange(
                    hsv,
                    cv::Scalar(160,100,100),
                    cv::Scalar(179,255,255),
                    mask2
                );
                // Combine both masks
                cv::Mat mask;
                cv::bitwise_or(mask1, mask2, mask);
                cv::imshow("Camera", image);
                cv::imshow("Red Detection", mask);

                cv::waitKey(1);

                cv::Moments moments = cv::moments(mask, true);
                //check whether any red pixels are detected
                if (moments.m00 > 0) {
                    double targetx = moments.m10 / moments.m00;
                    double image_center = image.cols/2.0;
                    double pixel_offset = targetx - image_center;
                    double fov = M_PI / 2.0;  // 90 degrees
                    double focal_length = (image.cols / 2.0) / std::tan(fov / 2.0);
                    double relative_angle = std::atan((targetx - image_center) / focal_length);
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Target X: %.2f | Image center: %.2f | Offset: %.2f",
                        targetx,
                        image_center,
                        pixel_offset
                    );
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Relative angle: %.4f radians (%.2f degrees)",
                        relative_angle,
                        relative_angle * 180.0 / M_PI
                    );
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "No red target detected!"
                    );
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Received image: %d x %d",
                    image.cols,
                    image.rows
                );
            }
            catch (const cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "cv_bridge exception: %s",
                    e.what()
                );
            }
        }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        camera_subscriber_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<YourSolution>());

    rclcpp::shutdown();

    return 0;
}