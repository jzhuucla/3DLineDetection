
set(SEMANTIC_3D_BIN run_semantic3d)

add_executable(${SEMANTIC_3D_BIN} ${CMAKE_CURRENT_SOURCE_DIR}/${SEMANTIC_3D_BIN}.cpp)
target_link_libraries(${SEMANTIC_3D_BIN} ${LIB_NAME} gflags glog)