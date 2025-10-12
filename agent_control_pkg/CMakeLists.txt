cmake_minimum_required(VERSION 3.16)

# Suppress deprecation warnings from third-party libraries (like yaml-cpp)
if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif()
if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

# Suppress cmake_minimum_required deprecation warnings from FetchContent dependencies
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "Disable deprecated warnings" FORCE)

# Optionally use vcpkg toolchain if available and tests are enabled
option(BUILD_TESTS "Build unit tests" ON)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND BUILD_TESTS)
  if(DEFINED ENV{VCPKG_ROOT})
    set(_VCPKG_TOOLCHAIN "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    if(EXISTS "${_VCPKG_TOOLCHAIN}")
      set(CMAKE_TOOLCHAIN_FILE "${_VCPKG_TOOLCHAIN}" CACHE STRING "Vcpkg toolchain file")
    endif()
  elseif(WIN32)
    # Fallback to a common Windows installation path
    set(_VCPKG_TOOLCHAIN "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
    if(EXISTS "${_VCPKG_TOOLCHAIN}")
      set(CMAKE_TOOLCHAIN_FILE "${_VCPKG_TOOLCHAIN}" CACHE STRING "Vcpkg toolchain file")
    endif()
  endif()
endif()

# Enable compile commands export
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

project(agent_control_standalone_test CXX)

# Suppress GNUInstallDirs warning from third-party dependencies
set(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add include directories for all targets
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Enable testing with CTest (optional)

if(BUILD_TESTS)
    enable_testing()
    include(CTest)
    include(GoogleTest)
    
    # Find GTest package
    find_package(GTest CONFIG QUIET)
    if(NOT GTest_FOUND)
        message(WARNING "GTest not found. Tests will be disabled. Install GTest via vcpkg or set BUILD_TESTS=OFF")
        set(BUILD_TESTS OFF CACHE BOOL "Build unit tests" FORCE)
    endif()
endif()

# Find yaml-cpp package
set(YAML_CPP_REQUIRED_VERSION "0.6.0")
find_package(yaml-cpp ${YAML_CPP_REQUIRED_VERSION} CONFIG)
if(NOT yaml-cpp_FOUND)
    message(STATUS "yaml-cpp not found via find_package, attempting to find it via pkg-config")
    find_package(PkgConfig)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(YAML_CPP yaml-cpp>=${YAML_CPP_REQUIRED_VERSION})
    endif()
endif()

if(NOT yaml-cpp_FOUND AND NOT YAML_CPP_FOUND)
    message(STATUS "yaml-cpp not found, attempting to use vcpkg")
    if(WIN32)
        set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "")
    endif()
    include(FetchContent)
    FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        # CORRECTED TAG: Use '0.8.0' instead of 'yaml-cpp-0.8.0'
        GIT_TAG 0.8.0
    )
    FetchContent_MakeAvailable(yaml-cpp)
endif()

# ... (the rest of the file is correct) ...
# --- Config Reader Library ---
add_library(config_reader_lib
    src/config_reader.cpp
)
target_include_directories(config_reader_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
if(yaml-cpp_FOUND)
    target_link_libraries(config_reader_lib PUBLIC yaml-cpp::yaml-cpp)
elseif(YAML_CPP_FOUND)
    target_include_directories(config_reader_lib PUBLIC ${YAML_CPP_INCLUDE_DIRS})
    target_link_libraries(config_reader_lib PUBLIC ${YAML_CPP_LIBRARIES})
else()
    target_link_libraries(config_reader_lib PUBLIC yaml-cpp)
endif()

# --- PID Logic Library ---
add_library(pid_logic_lib
    src/pid_controller.cpp
)
target_include_directories(pid_logic_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# --- Fuzzy Logic Library (will contain FLS implementation) ---
add_library(fuzzy_logic_lib
    src/gt2_fuzzy_logic_system.cpp
)
target_include_directories(fuzzy_logic_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# --- 2D Drone Dynamics Library ---
add_library(dynamics_2d_lib
    src/drone_dynamics_2d.cpp
)
target_include_directories(dynamics_2d_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# --- Controllers library (adapters, base interfaces) ---
add_library(controllers_lib INTERFACE)
target_include_directories(controllers_lib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(controllers_lib INTERFACE pid_logic_lib)

# Note: Obsolete executables removed (agent_control_main.cpp, fuzzy_test_main.cpp deleted)

# --- Unit Tests ---
if(BUILD_TESTS AND GTest_FOUND)
    add_executable(test_pid_controller
        test/test_pid_controller.cpp
    )
    target_include_directories(test_pid_controller PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_pid_controller PRIVATE
        pid_logic_lib
        GTest::gtest
        GTest::gtest_main
    )

    # Discover tests automatically
    gtest_discover_tests(test_pid_controller)

    # --- FLS Unit Tests ---
    add_executable(test_gt2_fls
        test/test_gt2_fls.cpp
    )
    target_include_directories(test_gt2_fls PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_gt2_fls PRIVATE
        fuzzy_logic_lib
        GTest::gtest
        GTest::gtest_main
    )

    gtest_discover_tests(test_gt2_fls)
    
    message(STATUS "Unit tests enabled and configured")
else()
    message(STATUS "Unit tests disabled (BUILD_TESTS=${BUILD_TESTS}, GTest_FOUND=${GTest_FOUND})")
endif()

# Optional: Install rules
# install(TARGETS pid_standalone_tester fuzzy_standalone_tester DESTINATION bin)
# install(TARGETS pid_logic_lib fuzzy_logic_lib config_reader_lib
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin)
# install(DIRECTORY include/ DESTINATION include)

# Note: Simple PID tuner removed (simple_pid_tuning_main.cpp deleted)

# --- Standalone tester removed (migrating to ROS 2 nodes) ---
# --- Executable for Multi-Drone PID Test (standalone) ---
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
    dynamics_2d_lib
)

# --- Simple dynamics tester ---
add_executable(dynamics_2d_tester
    src/dynamics_2d_test_main.cpp
)
target_include_directories(dynamics_2d_tester PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(dynamics_2d_tester PRIVATE
    dynamics_2d_lib
    pid_logic_lib
    config_reader_lib
    controllers_lib
)

# Note: Modular PID system removed (obsolete directory deleted)

# The execute_process command to copy compile_commands.json has been removed.
# It was failing due to special characters in your file path.
# The file is still created in the build directory (`build/compile_commands.json`),
# and modern editors/tools can be configured to find it there. This change
# resolves the critical build error.
