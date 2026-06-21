cmake_minimum_required(VERSION 3.8)
project(laplacian_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgentState.msg"
  "msg/MissionCommand.msg"
  "msg/QrDetection.msg"
  "msg/ColorDetection.msg"
  "msg/ControlSetpoint.msg"
  "msg/ControlCommand.msg"
  "msg/CommandResult.msg"
  "msg/Px4Status.msg"
  "msg/LocalizationState.msg"
  "msg/ZoneObservation.msg"
  "msg/ZoneAck.msg"
  DEPENDENCIES std_msgs
)
ament_package()
