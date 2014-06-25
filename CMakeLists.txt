#############################################
# PTAM project
#############################################
cmake_minimum_required(VERSION 2.8)

project(PTAM)
set(PTAM_VERSION_MAJOR 0)
set(PTAM_VERSION_MINOR 1)

#############################################
# Dependencies

#OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(SEND_ERROR "Faild to find Opencv")
    return()
else()
    set(PTAM_DEPENDENCY_DIRS ${OpenCV_INCLUDE_DIRS} ${PTAM_DEPENDENCY_DIRS})
    set(PTAM_DEPENDENCY_LIBS ${OpenCV_LIBS} ${PTAM_DEPENDENCY_LIBS})
endif()

find_package(LAPACK)
if(NOT LAPACK_FOUND)
    message(SEND_ERROR "Faild to find Lapack")
    return()
else()
    set(PTAM_DEPENDENCY_DIRS ${LAPACK_INCLUDE_DIR} ${PTAM_DEPENDENCY_DIRS})
    set(PTAM_DEPENDENCY_LIBS ${LAPACK_LIBRARIES} ${PTAM_DEPENDENCY_LIBS})
endif()

set(PTAM_DEPENDENCY_DIRS "${PROJECT_SOURCE_DIR}/3rdparty/TooN" ${PTAM_DEPENDENCY_DIRS})
set(PTAM_DEPENDENCY_DIRS "${PROJECT_SOURCE_DIR}/3rdparty/CVD/src" ${PTAM_DEPENDENCY_DIRS})
#############################################


#############################################
#General compiler flags

add_definitions(-DDEBUG_MODE) #This shows all messages from LOG()
add_definitions(-Wall) #Enable all warnings always
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -mtune=corei7 -march=corei7 -lpthread")

#############################################

add_subdirectory(3rdparty)
add_subdirectory(PTAM)
add_subdirectory(apps)
add_subdirectory(unit_tests)
