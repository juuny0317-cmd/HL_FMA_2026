cmake_minimum_required(VERSION 3.8)
project(hl_ku_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCommand.msg"
  "msg/DriveCommand.msg"
  "msg/VehicleFeedback.msg"
  "msg/GnssStatus.msg"
  "msg/PerceptionState.msg"
  "msg/MissionStatus.msg"
  "msg/Obstacle2D.msg"
  "msg/ObstacleArray.msg"
  "msg/PedestrianFrame.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
