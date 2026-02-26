cmake_minimum_required(VERSION 3.8)
project(unitree_udp_sender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)

# Unitree SDK configuration
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

# Set unitree SDK path (can be overridden by user)
if(NOT DEFINED UNITREE_SDK_PATH)
  set(UNITREE_SDK_PATH "${CMAKE_SOURCE_DIR}/unitree_legged_sdk")
endif()

set(extra_lib ${UNITREE_SDK_PATH}/lib/cpp/${ARCH}/libunitree_legged_sdk.a)

# Check if SDK exists, and add directories if it does
if(EXISTS "${UNITREE_SDK_PATH}/include")
  include_directories(${UNITREE_SDK_PATH}/include)
  message(STATUS "Found Unitree SDK at: ${UNITREE_SDK_PATH}")
else()
  message(WARNING "Unitree SDK not found at: ${UNITREE_SDK_PATH}")
  message(WARNING "Building without Unitree SDK. The node will not function without the SDK.")
endif()

# Create component node
add_library(unitree_udp_sender_component SHARED
  src/unitree_udp_sender_component.cpp
)

target_include_directories(unitree_udp_sender_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(unitree_udp_sender_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  ros2_unitree_legged_msgs
)

if(EXISTS "${UNITREE_SDK_PATH}/include")
  target_link_libraries(unitree_udp_sender_component ${extra_lib})
endif()

rclcpp_components_register_node(unitree_udp_sender_component
  PLUGIN "aist_intern2023::UnitreeUDPSender"
  EXECUTABLE unitree_udp_sender
)

# Install
install(TARGETS unitree_udp_sender_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
