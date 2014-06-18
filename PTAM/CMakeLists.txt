##################################
# PTAM library
##################################

include_directories(${PTAM_DEPENDENCY_DIRS})

##################################
# Sources

file(GLOB_RECURSE ptam_cpp src/*.cpp)
file(GLOB_RECURSE ptam_cc src/*.cc)
file(GLOB_RECURSE ptam_cxx src/*.cxx)
file(GLOB_RECURSE ptam_h src/*.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
##################################

##################################
# Target

add_library(ptam ${ptam_cpp} ${ptam_cc} ${ptam_cxx} ${ptam_h})
target_link_libraries(ptam cvd ${PTAM_DEPENDENCY_LIBS})

##################################
