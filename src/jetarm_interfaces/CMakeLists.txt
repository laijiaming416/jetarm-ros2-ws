cmake_minimum_required(VERSION 3.8)
project(jetarm_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmState.msg"
  "msg/VoiceText.msg"
  "msg/AsrState.msg"
  "msg/TtsState.msg"
  "msg/Intent.msg"
  "msg/MissionState.msg"
  "srv/MoveJoints.srv"
  "srv/MovePose.srv"
  "srv/Grip.srv"
  "srv/PlayAction.srv"
  "srv/NamedPose.srv"
  "srv/Grasp.srv"
  "srv/GraspPreview.srv"
  "srv/ReadState.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()