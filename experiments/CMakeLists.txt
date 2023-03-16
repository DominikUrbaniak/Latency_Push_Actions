cmake_minimum_required(VERSION 3.8)
project(experiments)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(kinenik REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs)
find_package(geometry_msgs)
find_package(tf2)
find_package(tf2_ros)

add_executable( push_translation_gazebo src/push_translation_gazebo_gpt.cpp )
ament_target_dependencies(push_translation_gazebo rclcpp custom_interfaces gazebo_msgs kinenik geometry_msgs tf2 tf2_ros)

add_executable( push_edge_gazebo src/push_edge_gazebo.cpp )
ament_target_dependencies(push_edge_gazebo rclcpp custom_interfaces gazebo_msgs kinenik geometry_msgs tf2 tf2_ros)

add_executable( push_translation_main src/push_translation_main.cpp )
ament_target_dependencies(push_translation_main rclcpp custom_interfaces gazebo_msgs kinenik geometry_msgs tf2 tf2_ros)

add_executable( push_edge_main src/push_edge_main.cpp )
ament_target_dependencies(push_edge_main rclcpp custom_interfaces gazebo_msgs kinenik geometry_msgs tf2 tf2_ros)

# install the executable in the lib folder to make it detectable through setup.bash
install(TARGETS
  push_translation_gazebo
  push_edge_gazebo
  push_translation_main
  push_edge_main
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
