cmake_minimum_required(VERSION 3.8)
project(unitree_udp_sender)

find_package(extension_ament REQUIRED)

extension_ament_init()

extension_ament_create_node(unitree_udp_sender_component
  NODE_NAME "unitree_udp_sender"
  NODE_PLUGIN "aist_intern2023::UnitreeUDPSender"
)

extension_ament_final()