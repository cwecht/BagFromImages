cmake_minimum_required(VERSION 2.8)
project(rosbag_from_images)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp rosbag)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  main.cc
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries (${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
