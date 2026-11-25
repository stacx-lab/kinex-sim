cmake_minimum_required(VERSION 3.10)
project(custom_interfaces)
# The interfaces of a graph's devices that are not standard ROS types: the world's Now and Wait services, and Json,
# the free-form service and action of operations whose description defines their fields. Kinex builds the same package.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "action/Json.action" "srv/Json.srv" "srv/Now.srv" "srv/Wait.srv"
  DEPENDENCIES unique_identifier_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
