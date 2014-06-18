##################################
# cvd library
##################################

include_directories(${PTAM_DEPENDENCY_DIRS})

##################################
# Sources

file(GLOB_RECURSE cvd_cpp src/*.cpp)
file(GLOB_RECURSE cvd_cc src/*.cc)
file(GLOB_RECURSE cvd_cxx src/*.cxx)
file(GLOB_RECURSE cvd_h include/*.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
##################################

##################################
# Target

add_library(cvd ${cvd_cpp} ${cvd_cc} ${cvd_cxx} ${cvd_h})
target_link_libraries(cvd ${PTAM_DEPENDENCY_LIBS})

##################################
