cmake_minimum_required(VERSION 3.0.2)
project(odrive_interface)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    roslib
    std_msgs
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(JSONCPP jsoncpp)
pkg_check_modules(LIBUSB1 libusb-1.0)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime roscpp roslib std_msgs
)

link_libraries(${JSONCPP_LIBRARIES} ${LIBUSB1_LIBRARIES})
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/odrive.cpp src/odrive_endpoint.cpp test/${PROJECT_NAME}_test.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${LIBUSB1_LIBRARIES}
    ${JSONCPP_LIBRARIES}
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


