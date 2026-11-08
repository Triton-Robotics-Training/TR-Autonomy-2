#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/float32.hpp"

#include <cmath>
#include <functional>

class YourSolution : public rclcpp::Node
{
public:
    YourSolution() : Node("yoursolution")
    {
        image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "/robotcam",
            10,
            std::bind(&YourSolution::image_callback, this, std::placeholders::_1)
        );

        angle_subscriber = this->create_subscription<std_msgs::msg::Float32>(
            "/current_angle",
            10,
            std::bind(&YourSolution::angle_callback, this, std::placeholders::_1)
        );

        angle_publisher = this->create_publisher<std_msgs::msg::Float32>(
            "/desired_angle", 10
        );
    }

private:
    double current_angle = 0.0;

    void angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_angle = msg->data;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_image;

        try {
            cv_image = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_image->image;

        cv::Mat red_mask;

        cv::inRange(image, cv::Scalar(0, 0, 100),
                    cv::Scalar(100, 100, 255), red_mask);

        cv::Moments moments = cv::moments(red_mask, true);

        if (moments.m00 == 0) {
            RCLCPP_WARN(this->get_logger(), "No red target detected");
            return;
        }

        double image_center = image.cols / 2.0;
        double target_center = moments.m10 / moments.m00;

        double pixel_offset = target_center - image_center;

        double angle_offset = pixel_offset * (M_PI / 2.0) / image.cols;

        double desired_angle = current_angle - angle_offset;

        desired_angle = std::max(-M_PI / 4.0,
                                 std::min(M_PI / 4.0, desired_angle));

        std_msgs::msg::Float32 angle_msg;
        angle_msg.data = desired_angle;

        angle_publisher->publish(angle_msg);

        RCLCPP_INFO(this->get_logger(), "Current angle: %f", current_angle);
        RCLCPP_INFO(this->get_logger(), "Target center: %f", target_center);
        RCLCPP_INFO(this->get_logger(), "Desired angle: %f", desired_angle);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_publisher;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YourSolution>());
    rclcpp::shutdown();
    return 0;
}
