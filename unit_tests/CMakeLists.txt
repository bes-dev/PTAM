##################################
# cstitcher_unittest
##################################

include_directories(${PTAM_DEPENDENCY_DIRS})

##################################
# Sources

file(GLOB_RECURSE sources_cpp *.cpp)
file(GLOB_RECURSE sources_cc *.cc)
file(GLOB_RECURSE sources_h *.h)

##################################


##################################
# Target
add_executable(ptam_unittest ${sources_cc} ${sources_cpp} ${sources_h})
target_link_libraries(ptam_unittest ptam ${PTAM_DEPENDENCY_LIBS})

##################################
