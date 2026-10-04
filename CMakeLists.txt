cmake_minimum_required(VERSION 3.8)
project(auto_aim_cpp CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)

add_executable(auto_aim
  src/main.cpp
  src/serial_turret.cpp
  src/plate_detector.cpp
  src/kalman1d.cpp
  src/plate_tracker.cpp
  src/line_guard.cpp
  src/aim_node.cpp
)

target_include_directories(auto_aim PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(auto_aim rclcpp sensor_msgs)
target_link_libraries(auto_aim ${OpenCV_LIBS})

install(TARGETS auto_aim DESTINATION lib/${PROJECT_NAME})