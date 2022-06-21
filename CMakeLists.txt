cmake_minimum_required(VERSION 3.12)
project(graph_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${GTSAM_INCLUDE_DIR}
)
ament_export_include_directories(include)

add_library(graph_manager STATIC
  src/graph_manager.cc
  src/graph_manager_config.cc
  src/graph_manager_logger.cc
  src/graph_manager_publisher.cc
  src/graph_manager_visualizer.cc)
ament_target_dependencies(graph_manager rclcpp geometry_msgs nav_msgs sensor_msgs message_filters)

add_executable(graph_manager_node src/graph_manager_node.cc)
target_link_libraries(graph_manager_node graph_manager ${Boost_LIBRARIES} gtsam)
ament_target_dependencies(graph_manager_node rclcpp geometry_msgs nav_msgs sensor_msgs message_filters)

install(TARGETS
  TARGETS graph_manager_node
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
