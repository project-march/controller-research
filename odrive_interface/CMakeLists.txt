cmake_minimum_required(VERSION 2.8.3)
project(odrive_interface)

cmake_policy(SET CMP0054 NEW)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    roslib
    std_msgs
)

set(usb-1.0_LIBRARIES usb-1.0)
set(jsoncpp_LIBRARIES jsoncpp)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime roscpp roslib std_msgs
    DEPENDS ${usb-1.0_LIBRARIES} ${jsoncpp_LIBRARIES}
)


include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} test/odrive_interface_test.cpp src/odrive.cpp src/odrive_endpoint.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${jsoncpp_LIBRARIES}
    ${usb-1.0_LIBRARIES}
    yaml-cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

