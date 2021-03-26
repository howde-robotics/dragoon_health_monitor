cmake_minimum_required(VERSION 3.10.2)
project(health_monitor VERSION 0.0.1)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  rostest
  rosunit
  actionlib_msgs
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# add executables from your source code, i.e. compile it
# .py code are not included here as they do not need to be compiled
add_executable(simple_node 
  src/simple_node/simple_node.cpp
)
add_library(myAlgo src/myAlgo.cpp)

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  # add_rostest_gtest(myrostest test/rostest.launch test/rostest.cpp)
  # target_link_libraries(myrostest 
  # ${catkin_LIBRARIES} 
  # ${GTEST_LIBRARIES}
  # )

  catkin_add_gtest(unittest test/unittest.cpp)
  target_link_libraries(unittest 
  myAlgo 
  # ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES} 
  ${GTEST_LIBRARIES}
  )
endif()

target_link_libraries(simple_node
  myAlgo
  ${catkin_LIBRARIES}
  )
