cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()
if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

project(agent_control_pkg LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

option(BUILD_STANDALONE_TOOLS "Build legacy standalone executables" ON)
option(BUILD_TESTS "Build unit tests" ON)
option(ENABLE_CRAZYFLIE "Enable Crazyflie/Crazyswarm2 support (adds dual-output publisher)" OFF)

# Core ROS 2 dependencies (nodes will use these targets)
set(ROS2_REQUIRED_PACKAGES
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  ament_index_cpp
  my_custom_interfaces_pkg
)
foreach(pkg IN LISTS ROS2_REQUIRED_PACKAGES)
  find_package(${pkg} REQUIRED)
endforeach()

# Crazyflie/Crazyswarm2 support (optional)
if(ENABLE_CRAZYFLIE)
  find_package(crazyflie_interfaces REQUIRED)
  message(STATUS "Crazyflie support ENABLED - dual-output publisher will be built")
endif()

# Find yaml-cpp with fallbacks to keep cross-platform compatibility
set(YAML_CPP_REQUIRED_VERSION "0.6.0")
find_package(yaml-cpp ${YAML_CPP_REQUIRED_VERSION} CONFIG)
if(NOT yaml-cpp_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(YAML_CPP yaml-cpp>=${YAML_CPP_REQUIRED_VERSION})
  endif()
endif()

if(NOT yaml-cpp_FOUND AND NOT YAML_CPP_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
  )
  FetchContent_MakeAvailable(yaml-cpp)
endif()

include(GNUInstallDirs)

# ---------------------------------------------------------------------------
# Libraries
# ---------------------------------------------------------------------------
add_library(config_reader_lib
  src/config_reader.cpp
)
target_include_directories(config_reader_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(yaml-cpp_FOUND)
  if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(config_reader_lib PUBLIC yaml-cpp::yaml-cpp)
  elseif(TARGET yaml-cpp)
    target_link_libraries(config_reader_lib PUBLIC yaml-cpp)
  else()
    target_link_libraries(config_reader_lib PUBLIC yaml-cpp)
  endif()
elseif(YAML_CPP_FOUND)
  target_include_directories(config_reader_lib PUBLIC ${YAML_CPP_INCLUDE_DIRS})
  target_link_libraries(config_reader_lib PUBLIC ${YAML_CPP_LIBRARIES})
else()
  target_link_libraries(config_reader_lib PUBLIC yaml-cpp)
endif()

add_library(pid_logic_lib
  src/pid_controller.cpp
)
target_include_directories(pid_logic_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(fuzzy_logic_lib
  src/gt2_fuzzy_logic_system.cpp
)
target_include_directories(fuzzy_logic_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(controller_adapters_lib
  src/controllers/fuzzy_gt2_adapter.cpp
)
target_include_directories(controller_adapters_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(controller_adapters_lib PUBLIC
  pid_logic_lib
  fuzzy_logic_lib
)

# ---------------------------------------------------------------------------
# Legacy standalone tools (keep available for analysis workflows)
# ---------------------------------------------------------------------------
if(BUILD_STANDALONE_TOOLS)
  add_executable(multi_drone_pid_tester
    src/multi_drone_pid_test_main.cpp
  )
  target_include_directories(multi_drone_pid_tester PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(multi_drone_pid_tester PRIVATE
    pid_logic_lib
    fuzzy_logic_lib
    config_reader_lib
    controller_adapters_lib
  )
endif()

# ---------------------------------------------------------------------------
# ROS 2 Nodes
# ---------------------------------------------------------------------------
add_executable(agent_controller_node
  src/ros/agent_controller_node.cpp
)
target_include_directories(agent_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(agent_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  ament_index_cpp
  my_custom_interfaces_pkg
)
target_link_libraries(agent_controller_node
  controller_adapters_lib
  config_reader_lib
  pid_logic_lib
  fuzzy_logic_lib
)

# Crazyflie dual-output support
if(ENABLE_CRAZYFLIE)
  target_compile_definitions(agent_controller_node PRIVATE CRAZYFLIE_SUPPORT)
  ament_target_dependencies(agent_controller_node crazyflie_interfaces)
endif()

add_executable(path_visualizer_node
  src/ros/path_visualizer_node.cpp
)
target_include_directories(path_visualizer_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(path_visualizer_node
  rclcpp
  geometry_msgs
  nav_msgs
)

add_executable(metrics_publisher_node
  src/ros/metrics_publisher_node.cpp
)
target_include_directories(metrics_publisher_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(metrics_publisher_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  my_custom_interfaces_pkg
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTS)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pid_controller
    test/test_pid_controller.cpp
  )
  if(TARGET test_pid_controller)
    target_include_directories(test_pid_controller PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_pid_controller
      pid_logic_lib
    )
  endif()

  ament_add_gtest(test_gt2_fls
    test/test_gt2_fls.cpp
  )
  if(TARGET test_gt2_fls)
    target_include_directories(test_gt2_fls PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_gt2_fls
      fuzzy_logic_lib
    )
  endif()
endif()

# ---------------------------------------------------------------------------
# Installation & Export
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)

set(_exported_targets
  config_reader_lib
  pid_logic_lib
  fuzzy_logic_lib
  controller_adapters_lib
)

if(BUILD_STANDALONE_TOOLS)
  list(APPEND _exported_targets
    multi_drone_pid_tester
  )
endif()

install(TARGETS ${_exported_targets}
  EXPORT agent_control_pkg_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS agent_controller_node path_visualizer_node metrics_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

# Install Python scripts
install(PROGRAMS
  scripts/metrics_summary.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(agent_control_pkg_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

set(_ament_export_deps ${ROS2_REQUIRED_PACKAGES})
if(yaml-cpp_FOUND)
  list(APPEND _ament_export_deps yaml-cpp)
endif()
ament_export_dependencies(${_ament_export_deps})

# ============================================================================
# GAZEBO PLUGIN
# ============================================================================
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(simple_drone_plugin SHARED
  plugins/simple_drone_plugin.cpp
)

target_include_directories(simple_drone_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(simple_drone_plugin
  ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
)

ament_target_dependencies(simple_drone_plugin
  rclcpp
  geometry_msgs
  nav_msgs
  gazebo_ros
)

# Set RPATH so plugin can find ROS2 libraries at runtime
set_target_properties(simple_drone_plugin PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:$ORIGIN"
  BUILD_RPATH "${CMAKE_BINARY_DIR}/lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS simple_drone_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Gazebo assets
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
