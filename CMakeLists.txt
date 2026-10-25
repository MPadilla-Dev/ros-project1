cmake_minimum_required(VERSION 2.8.3)
project(first_demo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
  genmsg
  nav_msgs
)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(demo_topic_publisher src/demo_topic_publisher.cpp)

target_link_libraries(demo_topic_publisher ${catkin_LIBRARIES})
#add_dependencies(demo_topic_publisher first_demo_generate_messages_cpp)

add_executable(demo_topic_subscriber src/demo_topic_subscriber.cpp)

target_link_libraries(demo_topic_subscriber ${catkin_LIBRARIES})
#add_dependencies(demo_topic_subscriber first_demo_generate_messages_cpp)

add_executable(straightLineExamp src/straightLineExamp.cpp)

target_link_libraries(straightLineExamp ${catkin_LIBRARIES})

# Project Nodes
add_executable(talkerGoals src/talkerGoals.cpp)

target_link_libraries(talkerGoals ${catkin_LIBRARIES})

add_executable(moveRobot src/moveRobot.cpp)

target_link_libraries(moveRobot ${catkin_LIBRARIES})



