cmake_minimum_required(VERSION 2.8.3)
project(chisel_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs message_generation)

SET(CMAKE_BUILD_TYPE Release)

add_message_files(FILES
  ChunkMessage.msg
  ChunkListMessage.msg
  IncrementalChangesMessage.msg
  ChunkIDMessage.msg
  VolumeMessage.msg
)

add_service_files(FILES
  GetAllChunksService.srv
  GetLatestChunksService.srv
  GetDeletedChunksService.srv
  PauseService.srv
  ResetService.srv
  SaveMeshService.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS  message_runtime std_msgs sensor_msgs geometry_msgs)
