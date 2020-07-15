cmake_minimum_required(VERSION 2.8.3)
project(odrive_interface)

cmake_policy(SET CMP0054 NEW)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    std_msgs
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(jsoncpp REQUIRED jsoncpp )
pkg_check_modules(libusb-1.0 REQUIRED libusb-1.0 )

find_package(jsoncpp REQUIRED)

find_path(LIBUSB_INCLUDE_DIR
    NAMES libusb.h
    PATH_SUFFIXES "include" "libusb" "libusb-1.0")

find_library(LIBUSB_LIBRARY
    NAMES usb
    PATH_SUFFIXES "lib" "lib32" "lib64")

set(jsoncpp_LIBRARIES jsoncpp)
set(usb-1.0_LIBRARIES libusb-1.0)

add_message_files(
    FILES
    odrive_msg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime roscpp std_msgs
    DEPENDS ${jsoncpp_LIBRARIES} ${libusb-1.0_LIBRARIES}
)

link_libraries(${jsoncpp_LIBRARIES} ${libusb-1.0_LIBRARIES})
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/odrive.cpp src/odrive_endpoint.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${libusb-1.0_LIBRARIES} ${catkin_LIBRARIES} ${jsoncpp_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )
