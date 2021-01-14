cmake_minimum_required(VERSION 2.8.3)
project(temperature_gradient_navigation)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        std_msgs
        nav_msgs
        geometry_msgs
	sensor_msgs
        gazebo_msgs
        visualization_msgs
        cv_bridge
        random_numbers
        message_generation
        image_transport
        cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        roscpp
        std_msgs
        nav_msgs
        geometry_msgs
        sensor_msgs
        gazebo_msgs
        message_runtime
        random_numbers
        image_transport
        cv_bridge
    LIBRARIES
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(temperature_gradient_navigation src/temperature_gradient_navigation/temperature_gradient_navigation.cpp)
add_dependencies(temperature_gradient_navigation ${catkin_LIBRARIES} ${catkin_EXPORTED_TARGETS})
target_link_libraries(temperature_gradient_navigation ${catkin_LIBRARIES})

add_executable(temperature_gradient_navigation_node src/temperature_gradient_navigation/temperature_gradient_navigation_node.cpp)
add_dependencies(temperature_gradient_navigation_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(temperature_gradient_navigation_node temperature_gradient_navigation ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
