cmake_minimum_required(VERSION 3.8)
project(dalnometr_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeAddress.srv"
)

ament_package()
