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