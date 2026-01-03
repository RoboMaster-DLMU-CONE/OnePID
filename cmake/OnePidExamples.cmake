add_executable(OnePidControllerExample ${PROJECT_SOURCE_DIR}/examples/PidController.cpp)
add_executable(OnePidChainExample ${PROJECT_SOURCE_DIR}/examples/PidChain.cpp)

target_link_libraries(OnePidControllerExample PUBLIC ${PROJECT_NAME}::${PROJECT_NAME})
target_link_libraries(OnePidChainExample PUBLIC ${PROJECT_NAME}::${PROJECT_NAME})
