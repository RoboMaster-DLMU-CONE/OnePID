cmake_minimum_required(VERSION 3.25)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)

option(ONE_PID_BUILD_EXAMPLES "Build OnePID Examples" OFF)
option(ONE_PID_BUILD_TESTS "Build OnePID Tests" OFF)
option(ONE_PID_BUILD_BENCHMARK "Build OnePID Benchmark" OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (ZEPHYR_TOOLCHAIN_VARIANT)
  project(onepid VERSION 0.1.0)
else ()
  project(OnePID VERSION 0.1.0)
endif ()

add_library(${PROJECT_NAME} INTERFACE)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR}/include)
if (ZEPHYR_TOOLCHAIN_VARIANT)
  zephyr_link_libraries(${PROJECT_NAME}::${PROJECT_NAME})
endif ()

if (ONE_PID_BUILD_EXAMPLES)
  include(OnePidExamples)
endif ()

if (BUILD_OM_TESTS)
  include(OnePidTests)
endif ()

# include(OnePidGenerateDoc)

