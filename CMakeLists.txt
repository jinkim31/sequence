cmake_minimum_required(VERSION 3.0.2)
project(sequence)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        )
catkin_package(
        INCLUDE_DIRS inc
        LIBRARIES sequence
        CATKIN_DEPENDS roscpp std_msgs
)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
            src/sequence_core.cpp
            src/sequence.cpp
            src/util.cpp)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)