cmake_minimum_required(VERSION 3.19)

project(gofa-kinfu LANGUAGES CXX)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(orocos_kdl REQUIRED)
find_package(YCM REQUIRED)
find_package(YARP 3.12 COMPONENTS os)
if(NOT YARP_FOUND)
    find_package(YARP 4.0 REQUIRED COMPONENTS os)
endif()
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development.Module)
find_package(matplotlib_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ROBOTICSLAB_VISION REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp
                               src/TrajectoryGenerator.cpp
                               src/Robot.cpp
                               include/TrajectoryGenerator.h
                               include/Robot.h)

target_link_libraries(${PROJECT_NAME} Boost::program_options
                                      Boost::disable_autolinking
                                      ${orocos_kdl_LIBRARIES}
                                      YARP::YARP_os
                                      YARP::YARP_init
                                      rclcpp::rclcpp
                                      std_msgs::std_msgs__rosidl_typesupport_cpp
                                      geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
                                      Python3::Module
                                      matplotlib_cpp::matplotlib_cpp
                                      yaml-cpp::yaml-cpp
                                      ROBOTICSLAB::VisionIDL)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include
                                                   ${orocos_kdl_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME})

include(AddUninstallTarget)
