cmake_minimum_required(VERSION 3.11)
project(dogbot_core)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dogbot_msg REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_RTSP REQUIRED gstreamer-rtsp-server-1.0)

add_library(dogbot_core_lib SHARED
  src/hardware/dogbot.cpp
  src/controller/leg_controller.cpp
  src/vision/camera/camera_node.cpp
  src/vision/camera/camera_top_node.cpp
  src/vision/camera/rtsp_stream_node.cpp
  src/vision/following.cpp
  src/vision/color_detect.cpp
  src/vision/color_detect_nonblue.cpp
  src/vision/color_listener.cpp
)
target_include_directories(dogbot_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${GST_RTSP_INCLUDE_DIRS})
ament_target_dependencies(dogbot_core_lib
  dogbot_msg geometry_msgs rclcpp rclcpp_components std_msgs
  sensor_msgs cv_bridge)
target_link_libraries(dogbot_core_lib
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${GST_RTSP_LIBRARIES})

rclcpp_components_register_nodes(dogbot_core_lib
  "dogbot_core::hardware::DogBot"
  "dogbot_core::controller::LegController"
  "dogbot_core::camera::CameraNode"
  "dogbot_core::camera::CameraTopNode"
  "dogbot_core::camera::RtspStreamNode"
  "dogbot_core::vision::FollowingNode"
  "dogbot_core::vision::ColorDetectNode"
  "dogbot_core::vision::ColorDetectNonblueNode"
  "dogbot_core::vision::ColorListenerNode")

install(TARGETS dogbot_core_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
