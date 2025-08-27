cmake_minimum_required(VERSION 3.5)
project(executor_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(callback_group_demo src/callback_group_demo.cpp)
ament_target_dependencies(callback_group_demo rclcpp)

target_compile_features(callback_group_demo PUBLIC c_std_99 cxx_std_17)

install(TARGETS callback_group_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
