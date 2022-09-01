
set(LIB_NAME LineFromPointCloud)

ADD_DEFINITIONS(-DUNICODE)
add_definitions(${PCL_DEFINITIONS})

FILE(GLOB_RECURSE HDRS_FILES *.h *.hpp)
FILE(GLOB_RECURSE SRCS_FILES *.cpp *.c *.cc *.cxx)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

add_library(${LIB_NAME} ${SRCS_FILES} ${HDRS_FILES})
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_subdirectory(bin)
