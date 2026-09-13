#include "rotator_node.h"

int main(int argc, char* argv[])
{
	init(argc, argv);
	spin(std::make_shared<AngleFinder>());
	shutdown();
    return 0;
}

AngleFinder::AngleFinder() : Node("anglefinder") {
	pub_ = this->create_publisher<FltMsg>("desired_angle", 10);
	timer_ = this->create_wall_timer(REFRESH,
			std::bind(&AngleFinder::timer_callback, this));

	//subPnt_ = this->create_subscription<PntMsg>("scored_point", 10,
	//		std::bind(&AngleFinder::point_callback, this, 
	//			std::placeholders::_1));
	
	subAng_ = this->create_subscription<FltMsg>("current_angle", 10,
			std::bind(&AngleFinder::currAng_callback, this,
				std::placeholders::_1));

	rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
	subImg_ = image_transport::create_subscription(this, "robotcam",
			std::bind(&AngleFinder::cam_callback, this,
			       	std::placeholders::_1), "raw", custom_qos);
}

void AngleFinder::cam_callback(const ImgMsg & img) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	cvImg = cv_ptr->image;
}

void AngleFinder::currAng_callback(const std::shared_ptr<const FltMsg> & ang) {
	currAngle = ang->data;
}

void AngleFinder::timer_callback() {
	cv::cvtColor(cvImg, imgHSV, cv::COLOR_BGR2HSV);
	cv::Vec3b bgrPixel(0, 0, 255);
	Mat3b bgr (bgrPixel);
	Mat3b hsv;
	cv::cvtColor(bgr, hsv, COLOR_BGR2HSV);
	
	cv::Vec3b hsvPixel(hsv.at<Vec3b>(0, 0));

	int thresh = 40;

	cv::Scalar minHSV = cv::Scalar(hsvPixel.val[0] - thresh, hsvPixel.val[1] - thresh, hsvPixel.val[2] - thresh);
	cv::Scalar maxHSV = cv::Scalar(hsvPixel.val[0] + thresh, hsvPixel.val[1] + thresh, hsvPixel.val[2] + thresh);
	
	cv::Mat maskHSV, resultHSV;
	cv::inRange(imgHSV, minHSV, maxHSV, maskHSV);
	cv::bitwise_and(imgHSV, imgHSV, resultHSV, maskHSV);
	
	cv::Mat thr, gray;
	cv::cvtColor(cvImg, gray, cv::COLOR_BGR2GRAY);
	cv::threshold(gray, thr, 100, 255, cv::THRESH_BINARY_INV);
	
	cv::Moments m = cv::moments(thr, true);
	if (std::abs(m.m00) < 1e-8) {
		throw std::runtime_error("Zero-area blob");
	}
	cv::Point p(m.m10 / m.m00, m.m01 / m.m00);
	
	auto msg = FltMsg();
	// float pi = numbers::pi_v<float>;
	p.x *= (p.x <= 640 / 2 ? 1 : -1);
	std::cout << currAngle << std::endl;
	std::cout << p.x << std::endl;
	msg.data = currAngle + p.x / 2 * M_PI / (4.0 * 640);
	std::cout << "changing to angle: " << msg.data << std::endl;
	pub_->publish(msg);
}


// 1. make subscriber for image topic
// 2. convert that to opencv image
// 3. convert it back to ros2 image and publish to some debugger topic
// 4. apply mask to opencv image and publish opencv image and observe effect
// 5. hard code filter for simply red pixels
// 6. figure out angle with math
