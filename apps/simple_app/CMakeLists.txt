##################################
# ptam simple app
##################################

include_directories(${PTAM_DEPENDENCY_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/PTAM/src)

##################################
# Sources

file(GLOB_RECURSE sources_cpp *.cpp)
file(GLOB_RECURSE sources_h *.h)
##################################

##################################
# Target

add_executable(simple_app ${sources_cpp} ${sources_h})
target_link_libraries(simple_app ptam ${PTAM_DEPENDENCY_LIBS})
##################################
