cmake_minimum_required(VERSION 3.8)
project(solution)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(solution
  src/solution.cpp
)

ament_target_dependencies(solution
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_include_directories(solution PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(solution
  ${OpenCV_LIBS}
)

install(TARGETS solution
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
