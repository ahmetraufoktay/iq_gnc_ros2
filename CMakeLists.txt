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
find_package(yolov8_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)


add_library(gnc_node_lib src/gnc_node.cpp)
ament_target_dependencies(gnc_node_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  mavros_msgs
)
target_include_directories(gnc_node_lib PUBLIC include)


include_directories(
  include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(takeoff_without_gnc src/takeoff_without_gnc.cpp)
ament_target_dependencies(takeoff_without_gnc
  rclcpp
  std_msgs
  geometry_msgs
  mavros_msgs
)

add_executable(set_wp src/set_wp.cpp)
target_link_libraries(set_wp gnc_node_lib)
ament_target_dependencies(set_wp rclcpp)

add_executable(square src/square.cpp)
target_link_libraries(square gnc_node_lib)
ament_target_dependencies(square rclcpp)

add_executable(gnc_example src/gnc_tutorial.cpp)
target_link_libraries(gnc_example gnc_node_lib)
ament_target_dependencies(gnc_example rclcpp)

add_executable(search_and_rescue src/sr.cpp)
target_link_libraries(search_and_rescue gnc_node_lib)
target_link_libraries(search_and_rescue ${OpenCV_LIBS})
ament_target_dependencies(search_and_rescue
  rclcpp
  yolov8_msgs
  sensor_msgs
  cv_bridge
)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(avoidance src/avoidance.cpp)
target_link_libraries(avoidance gnc_node_lib)
ament_target_dependencies(avoidance 
  rclcpp
  sensor_msgs
)

install(TARGETS
  takeoff_without_gnc
  set_wp
  square
  gnc_example
  search_and_rescue
  avoidance
  DESTINATION lib/${PROJECT_NAME}
)

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
