cmake_minimum_required(VERSION 3.11)
project(dogbot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(dogbot_core_lib SHARED
  src/hardware/dogbot.cpp
)
ament_target_dependencies(dogbot_core_lib
  rclcpp rclcpp_components std_msgs)

rclcpp_components_register_nodes(dogbot_core_lib "DogBot")

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
