cmake_minimum_required(VERSION 3.8)
project(yoursolution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(aimer src/aimer.cpp)
ament_target_dependencies(aimer rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

# Without this, `ros2 run` can't find the executable
install(TARGETS aimer DESTINATION lib/${PROJECT_NAME})

ament_package()