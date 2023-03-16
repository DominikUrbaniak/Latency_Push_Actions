cmake_minimum_required(VERSION 3.8)
project(main_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs)
find_package(visualization_msgs)
find_package(custom_interfaces)
find_package(gazebo_msgs)
find_package(tf2_ros)
find_package(tf2)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    config
    launch
    meshes
    rviz
    src
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  launch/spawn_cube_launch.py
  launch/spawn_cube.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(world_setup src/world_setup.cpp)
ament_target_dependencies(world_setup rclcpp geometry_msgs visualization_msgs gazebo_msgs custom_interfaces tf2_ros tf2)

add_executable( velocity_controller src/velocity_controller_gpt.cpp )
ament_target_dependencies(velocity_controller rclcpp std_msgs sensor_msgs custom_interfaces tf2 tf2_ros)



install(TARGETS
  world_setup
  velocity_controller
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
