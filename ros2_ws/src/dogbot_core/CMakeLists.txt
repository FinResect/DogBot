cmake_minimum_required(VERSION 3.11)
project(dogbot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(dogbot_core_lib SHARED
  src/hardware/dogbot.cpp
  src/controller/leg_controller.cpp
)
target_include_directories(dogbot_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(dogbot_core_lib
  geometry_msgs rclcpp rclcpp_components std_msgs)

rclcpp_components_register_nodes(dogbot_core_lib "DogBot" "dogbot_core::controller::LegController")

install(TARGETS dogbot_core_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
