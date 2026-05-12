cmake_minimum_required(VERSION 3.8)
project(rcm_modulation_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(rcm_tracking_node
  src/main.cpp
  src/rcm_tracking_node.cpp
  src/modulation.cpp
  src/controller_placeholders.cpp
)

target_include_directories(rcm_tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rcm_tracking_node Eigen3::Eigen)

ament_target_dependencies(rcm_tracking_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

install(TARGETS rcm_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
