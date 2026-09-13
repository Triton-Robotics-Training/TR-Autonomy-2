#include <stdlib.h>
#include <chrono>
#include <memory>
#include <functional>
#include <numbers>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/float32.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "image_transport/image_transport.hpp"

#define _USE_MATH_DEFINES
#include <math.h>
using FltMsg = std_msgs::msg::Float32;
using ImgMsg = sensor_msgs::msg::Image::ConstSharedPtr;
// using PntMsg = std_msgs::msg::Empty;
using namespace rclcpp;
using namespace std::chrono_literals;
using namespace cv;

class AngleFinder : public rclcpp::Node {
	public:
		AngleFinder();
	private:
		void timer_callback();
		void currAng_callback(const std::shared_ptr<const FltMsg> & ang);
		// void point_callback(const std::shared_ptr<const PntMsg> & pnt);
		void cam_callback(const ImgMsg & img);
		TimerBase::SharedPtr timer_;
		Publisher<FltMsg>::SharedPtr pub_;
		image_transport::Subscriber subImg_;
		// Subscription<PntMsg>::SharedPtr subPnt_;
		Subscription<FltMsg>::SharedPtr subAng_;
		cv::Mat cvImg, imgHSV;
		float currAngle; 
		std::chrono::milliseconds REFRESH = 2000ms;
};
