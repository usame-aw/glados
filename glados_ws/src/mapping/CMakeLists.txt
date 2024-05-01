cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(teleop_twist_joy REQUIRED)
#find_package(communicator_library REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2  REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# C++ executables
install(
  DIRECTORY include/mapping
  DESTINATION include

)

add_executable(joystick_teleop src/joystick_controller.cpp)
ament_target_dependencies(joystick_teleop rclcpp sensor_msgs tf2_ros tf2)
target_link_libraries(joystick_teleop serial)

install(TARGETS
  joystick_teleop  
  DESTINATION lib/${PROJECT_NAME}
  
)

# add_executable(mapping_node src/mapping_node.cpp)
# ament_target_dependencies(mapping_node rclcpp sensor_msgs nav_msgs)
# install(TARGETS
#   mapping_node
#   DESTINATION lib/${PROJECT_NAME}
#)

# Python executables
ament_python_install_package(${PROJECT_NAME})

#install(PROGRAMS
#  scripts/mapping_node.py
#  DESTINATION lib/${PROJECT_NAME})

# Include other folders
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


# find_package(<dependency> REQUIRED)

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

ament_package()