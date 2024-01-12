cmake_minimum_required(VERSION 3.8)
project(unitree_udp_sender)

find_package(extension_ament REQUIRED)
find_package(unitree_legged_sdk REQUIRED)

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

extension_ament_init()
set(extra_lib libunitree_legged_sdk.so)
link_directories(${CMAKE_SOURCE_DIR}/../unitree_legged_sdk/lib/cpp/${ARCH})

extension_ament_create_node(unitree_udp_sender_component
  NODE_NAME "unitree_udp_sender"
  NODE_PLUGIN "aist_intern2023::UnitreeUDPSender"
)
target_link_libraries(unitree_udp_sender_component ${extra_lib})

extension_ament_final()