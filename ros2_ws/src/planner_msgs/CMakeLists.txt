cmake_minimum_required(VERSION 3.8)
project(planner_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PciInitialization.srv"
  "srv/PciGlobal.srv"
)

install(FILES mapping_rules.yaml LICENSE README.md DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
