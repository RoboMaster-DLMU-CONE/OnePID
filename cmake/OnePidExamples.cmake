add_executable(OnePidControllerExample ${PROJECT_SOURCE_DIR}/examples/PIDController.cpp)

target_link_libraries(OnePidControllerExample PUBLIC ${PROJECT_NAME}::${PROJECT_NAME})
