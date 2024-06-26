cmake_minimum_required(VERSION 2.8.3)
project(line_feature_tracker)

find_package(catkin REQUIRED COMPONENTS
        #OpenCV
        pcl_conversions
        pcl_ros
        cv_bridge
        roscpp
	rospy
        sensor_msgs
        std_msgs
        Eigen3
        camera_model
	message_generation
)
find_package(OpenCV REQUIRED)
add_message_files(
  FILES
  lines2d.msg
  lines_devide.msg
  # Message2.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)



catkin_package()

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/line_descriptor/include/
)


add_executable(line_feature_tracker
        src/feature_tracker_node.cpp
        src/feature_tracker.cpp
        src/parameters.cpp

        src/line_feature/line_feature.cpp
        src/line_feature/config.cpp
        src/line_feature/auxiliar.cpp
        src/line_feature/matching.cpp
        src/line_feature/lineIterator.cpp
        src/line_feature/gridStructure.cpp)

target_link_libraries(line_feature_tracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PROJECT_SOURCE_DIR}/line_descriptor/lib/liblinedesc.so)


