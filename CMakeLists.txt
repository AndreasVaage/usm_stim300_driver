cmake_minimum_required(VERSION 3.5.1)
project(usm_stim300_driver)

set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        message_generation
        )

add_message_files(
  FILES
  UInt8MultiArrayStamped.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS sensor_msgs message_runtime
)

add_library(stim300_driver_lib
        src/driver_stim300.cpp
        src/datagram_parser.cpp
        src/serial_unix.cpp)

target_include_directories(stim300_driver_lib
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${catkin_INCLUDE_DIRS}
        )
target_link_libraries(stim300_driver_lib ${catkin_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        )
catkin_package(
        CATKIN_DEPENDS sensor_msgs
)

add_executable(usm_stim300_driver_node src/stim300_driver_node.cpp)
target_include_directories(usm_stim300_driver_node
        PRIVATE
        ${catkin_INCLUDE_DIRS})
target_link_libraries(usm_stim300_driver_node PRIVATE stim300_driver_lib ${catkin_LIBRARIES})

add_executable(usm_stim300_arduino_node src/stim300_arduino_node.cpp)
target_include_directories(usm_stim300_arduino_node
        PRIVATE
        ${catkin_INCLUDE_DIRS})
target_link_libraries(usm_stim300_arduino_node PRIVATE stim300_driver_lib ${catkin_LIBRARIES})


if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(check_datasheet_constanst
            test/check_stim300_constants.cpp
            src/stim300_constants.h)

    target_link_libraries(check_datasheet_constanst ${catkin_LIBRARIES})
endif()

if (CATKIN_ENABLE_TESTING)
    catkin_add_gmock(check_driver_stim300
            test/check_driver_stim300.cpp
            test/mock_serial_driver.h
            src/serial_driver.h)

    target_link_libraries(check_driver_stim300 stim300_driver_lib ${catkin_LIBRARIES})
endif()


