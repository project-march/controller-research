cmake_minimum_required(VERSION 2.8.3)
project(odrive_interface)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    std_msgs
    )

find_package(Boost REQUIRED)

add_message_files(
    FILES
    odrive_msg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(libusb-1.0_LIBRARIES usb-1.0)
set(jsoncpp_LIBRARIES jsoncpp)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime roscpp std_msgs
    DEPENDS ${jsoncpp_LIBRARIES} ${pthread_LIBRARIES} ${rt_LIBRARIES} ${usb-1.0_LIBRARIES}
)

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
