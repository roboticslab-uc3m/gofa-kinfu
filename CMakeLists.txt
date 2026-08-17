cmake_minimum_required(VERSION 3.19)
project(gofa-kinfu LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

find_package(abb_libegm REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread program_options)
find_package(orocos_kdl REQUIRED)
find_package(YCM REQUIRED)
find_package(YARP 3.11 REQUIRED COMPONENTS os)
find_package(Python3 REQUIRED COMPONENTS Development.Module)
find_package(matplotlib_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ROBOTICSLAB_VISION REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp
                               src/TrajectoryGenerator.cpp
                               src/Robot.cpp
                               include/TrajectoryGenerator.h
                               include/Robot.h)

target_link_libraries(${PROJECT_NAME} ${abb_libegm_LIBRARIES}
                                      Boost::thread
                                      Boost::program_options
                                      Boost::disable_autolinking
                                      ${orocos_kdl_LIBRARIES}
                                      YARP::YARP_os
                                      YARP::YARP_init
                                      Python3::Module
                                      matplotlib_cpp::matplotlib_cpp
                                      yaml-cpp::yaml-cpp
                                      ROBOTICSLAB::VisionIDL)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include
                                                   ${abb_libegm_INCLUDE_DIRS}
                                                   ${orocos_kdl_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME})

include(AddUninstallTarget)
