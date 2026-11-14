cmake_minimum_required(VERSION 3.8)
project(g1_bridge_mappings)

find_package(ament_cmake REQUIRED)

ament_index_register_resource("ros1_bridge_foreign_mapping")

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()