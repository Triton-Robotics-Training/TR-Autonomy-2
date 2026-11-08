#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

// These are ordinary C++ variables, just like in the OpenCV tutorial.
// Each trackbar changes one of these variables.
const std::string window_detection_name = "HSV Controls";

int low_H = 0, high_H = 10;          // First red hue range
int low_H2 = 170, high_H2 = 179;    // Second red hue range
int low_S = 100, high_S = 255;
int low_V = 70, high_V = 255;

// The callbacks below are nearly identical to the OpenCV inRange tutorial.
// They prevent the lower slider from moving above the upper slider.
static void on_low_H_thresh_trackbar(int, void *) {
    low_H = std::min(high_H - 1, low_H);
    cv::setTrackbarPos("Low H", window_detection_name, low_H);
}

static void on_high_H_thresh_trackbar(int, void *) {
    high_H = std::max(high_H, low_H + 1);
    cv::setTrackbarPos("High H", window_detection_name, high_H);
}

static void on_low_H2_thresh_trackbar(int, void *) {
    low_H2 = std::min(high_H2 - 1, low_H2);
    cv::setTrackbarPos("Low H2", window_detection_name, low_H2);
}

static void on_high_H2_thresh_trackbar(int, void *) {
    high_H2 = std::max(high_H2, low_H2 + 1);
    cv::setTrackbarPos("High H2", window_detection_name, high_H2);
}

static void on_low_S_thresh_trackbar(int, void *) {
    low_S = std::min(high_S - 1, low_S);
    cv::setTrackbarPos("Low S", window_detection_name, low_S);
}

static void on_high_S_thresh_trackbar(int, void *) {
    high_S = std::max(high_S, low_S + 1);
    cv::setTrackbarPos("High S", window_detection_name, high_S);
}

static void on_low_V_thresh_trackbar(int, void *) {
    low_V = std::min(high_V - 1, low_V);
    cv::setTrackbarPos("Low V", window_detection_name, low_V);
}

static void on_high_V_thresh_trackbar(int, void *) {
    high_V = std::max(high_V, low_V + 1);
    cv::setTrackbarPos("High V", window_detection_name, high_V);
}

class RedTargetTracker : public rclcpp::Node {
public:
    RedTargetTracker() : Node("red_target_tracker") {
        // Create the GUI windows and sliders ONCE.
        cv::namedWindow(window_detection_name, cv::WINDOW_NORMAL);
        cv::namedWindow("Camera", cv::WINDOW_NORMAL);
        cv::namedWindow("Red Mask", cv::WINDOW_NORMAL);

        cv::createTrackbar("Low H", window_detection_name,
                           &low_H, 179, on_low_H_thresh_trackbar);
        cv::createTrackbar("High H", window_detection_name,
                           &high_H, 179, on_high_H_thresh_trackbar);
        cv::createTrackbar("Low H2", window_detection_name,
                           &low_H2, 179, on_low_H2_thresh_trackbar);
        cv::createTrackbar("High H2", window_detection_name,
                           &high_H2, 179, on_high_H2_thresh_trackbar);
        cv::createTrackbar("Low S", window_detection_name,
                           &low_S, 255, on_low_S_thresh_trackbar);
        cv::createTrackbar("High S", window_detection_name,
                           &high_S, 255, on_high_S_thresh_trackbar);
        cv::createTrackbar("Low V", window_detection_name,
                           &low_V, 255, on_low_V_thresh_trackbar);
        cv::createTrackbar("High V", window_detection_name,
                           &high_V, 255, on_high_V_thresh_trackbar);

        cv::resizeWindow(window_detection_name, 500, 550);
        cv::resizeWindow("Camera", 640, 640);
        cv::resizeWindow("Red Mask", 640, 640);

        // ROS 2 publisher: send the angle we want the robot to face.
        desired_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/desired_angle", 10);

        // ROS 2 subscriber: read the robot's current angle.
        angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/current_angle", 10,
            std::bind(&RedTargetTracker::angle_callback, this,
                      std::placeholders::_1));

        // ROS 2 subscriber: receive the camera image.
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robotcam", 10,
            std::bind(&RedTargetTracker::image_callback, this,
                      std::placeholders::_1));

        // Optional scoring notifications (the current simulator may not publish them).
        score_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/scored_point", 10,
            std::bind(&RedTargetTracker::score_callback, this,
                      std::placeholders::_1));

        // OpenCV windows need waitKey() to handle mouse/keyboard events.
        gui_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), [this]() {
                int key = cv::waitKey(1);
                if ((key == 's' || key == 'S') && !last_frame_.empty()) {
                    cv::imwrite("camera_frame.png", last_frame_);
                    RCLCPP_INFO(this->get_logger(), "Saved camera_frame.png");
                }
                if (key == 'q' || key == 'Q' || key == 27) {
                    rclcpp::shutdown();
                }
            });

        RCLCPP_INFO(this->get_logger(),
                    "Ready. Move HSV sliders. S=save image, Q=quit.");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_angle_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr score_sub_;
    rclcpp::TimerBase::SharedPtr gui_timer_;

    double current_angle_ = 0.0;
    bool received_angle_ = false;
    cv::Mat last_frame_;

    void angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_angle_ = msg->data;
        received_angle_ = true;
    }

    void score_callback(const std_msgs::msg::Empty::SharedPtr) {
        RCLCPP_INFO(this->get_logger(), "Point scored!");
    }

    void publish_angle(double angle) {
        // The assignment limits targets to +/- pi/4 radians.
        const double max_angle = 3.14159265358979323846 / 4.0;
        angle = std::max(-max_angle, std::min(max_angle, angle));

        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(angle);
        desired_angle_pub_->publish(msg);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            // Convert ROS Image to OpenCV Mat. The simulator uses BGR8.
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Image conversion failed: %s", e.what());
            return;
        }
        if (frame.empty()) return;
        last_frame_ = frame.clone();

        // 1. Convert the camera image from BGR to HSV.
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // 2. Create two red masks from the CURRENT slider values.
        cv::Mat mask1, mask2, red_mask;
        cv::inRange(hsv,
                    cv::Scalar(low_H, low_S, low_V),
                    cv::Scalar(high_H, high_S, high_V),
                    mask1);
        cv::inRange(hsv,
                    cv::Scalar(low_H2, low_S, low_V),
                    cv::Scalar(high_H2, high_S, high_V),
                    mask2);

        // 3. A pixel is red if it matches EITHER range.
        cv::bitwise_or(mask1, mask2, red_mask);

        // 4. Find the average x-position of all white (red) pixels.
        // With 'true', m00 is the number of nonzero pixels in this mask.
        cv::Moments m = cv::moments(red_mask, true);
        int image_center = frame.cols / 2;

        if (received_angle_) {
            if (m.m00 > 100) {  // Avoid dividing by 0 / reacting to a few pixels.
                double red_x = m.m10 / m.m00;
                double error = red_x - image_center;

                // Draw markers so you can see what the code detects.
                cv::circle(frame, cv::Point(static_cast<int>(red_x), frame.rows / 2),
                           7, cv::Scalar(0, 255, 0), -1);

                // The camera has a 90-degree field of view.
                // Right-of-center red pixels need a negative angle change.
                // A 30-pixel deadband is safely inside the 60-pixel scoring range.
                if (std::abs(error) > 30) {
                    double change = std::atan(error / (frame.cols / 2.0));
                    publish_angle(current_angle_ - change);
                }
            } else {
                // The target is always visible when the robot faces 0 radians.
                publish_angle(0.0);
            }
        }

        // 5. Show the camera and the binary mask for tuning.
        cv::line(frame, cv::Point(image_center, 0),
                 cv::Point(image_center, frame.rows - 1),
                 cv::Scalar(255, 255, 0), 2);
        cv::imshow("Camera", frame);
        cv::imshow("Red Mask", red_mask);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RedTargetTracker>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
