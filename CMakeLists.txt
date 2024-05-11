cmake_minimum_required(VERSION 3.5)
project(message_filters_simple_demo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(image_publisher image_publisher.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs)

add_executable(pointcloud_publisher pointcloud_publisher.cpp)
ament_target_dependencies(pointcloud_publisher rclcpp sensor_msgs)

add_executable(message_filters_demo message_filters_demo.cpp)
ament_target_dependencies(message_filters_demo rclcpp sensor_msgs message_filters)

install(TARGETS
  image_publisher
  pointcloud_publisher
  message_filters_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
