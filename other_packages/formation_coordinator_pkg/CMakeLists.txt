cmake_minimum_required(VERSION 3.8)
project(formation_coordinator_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_custom_interfaces_pkg REQUIRED)

# Include directories
include_directories(include)

# PSO Library
add_library(particle_swarm_optimizer
  src/particle_swarm_optimizer.cpp
)
target_include_directories(particle_swarm_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(formation_coordinator_node
  src/formation_coordinator_node.cpp
)
target_include_directories(formation_coordinator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(formation_coordinator_node
  particle_swarm_optimizer
)
ament_target_dependencies(formation_coordinator_node
  rclcpp
  std_msgs
  geometry_msgs
  my_custom_interfaces_pkg
)

install(TARGETS
  formation_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
