cmake_minimum_required(VERSION 3.8)
project(ros2_roomba)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(turtlebot3_gazebo REQUIRED)

# Adding an executable for code to avoid obstacles
add_executable(redirect_from_obstacles src/redirect_from_obstacles.cpp)
ament_target_dependencies(redirect_from_obstacles rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  redirect_from_obstacles
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()
