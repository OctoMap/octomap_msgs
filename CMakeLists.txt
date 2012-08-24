cmake_minimum_required(VERSION 2.8)
project(octomap_msgs)

find_package(catkin REQUIRED)
find_package(ROS REQUIRED COMPONENTS genmsg)

catkin_stack()
catkin_project(octomap_msgs
  INCLUDE_DIRS include)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES Octomap.msg OctomapWithPose.msg
  )

add_service_files(
  DIRECTORY srv
  FILES GetOctomap.srv BoundingBoxQuery.srv
)

find_package(std_msgs)
find_package(geometry_msgs)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(FILES manifest-to-install.xml
        DESTINATION share/octomap_msgs RENAME manifest.xml)
