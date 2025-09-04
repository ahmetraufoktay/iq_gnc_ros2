cmake_minimum_required(VERSION 3.8)
project(iq_gnc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#find_package(catkin REQUIRED COMPONENTS
#  mavros
#  std_msgs
#  geometry_msgs
#  mavros_msgs
#  rospy
#  sensor_msgs
#)

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

include_directories(
  include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(takeoff_without_gnc src/a.cpp)
ament_target_dependencies(takeoff_without_gnc
  rclcpp
  std_msgs
  geometry_msgs
  mavros_msgs
)

add_executable(set_wp src/set_wp.cpp src/gnc_functions.cpp)
ament_target_dependencies(set_wp
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  mavros_msgs
)

# Build square node
add_executable(square src/square.cpp src/gnc_functions.cpp)
ament_target_dependencies(square
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  mavros_msgs
)

add_executable(gnc_example src/gnc_tutorial.cpp src/gnc_functions.cpp)
ament_target_dependencies(gnc_example
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  mavros_msgs
)

install(TARGETS
  takeoff_without_gnc
  set_wp
  square
  gnc_example
  DESTINATION lib/${PROJECT_NAME}
)

#target_link_libraries(square ${catkin_LIBRARIES})

#target_link_libraries(gnc_example ${catkin_LIBRARIES})

#add_executable(sub_sol src/subscriber_sol.cpp)
#target_link_libraries(sub_sol ${catkin_LIBRARIES})

#add_executable(avoidance_sol src/avoidance_sol.cpp)
#target_link_libraries(avoidance_sol ${catkin_LIBRARIES})

# Add tutorial add_executable lines here 

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
