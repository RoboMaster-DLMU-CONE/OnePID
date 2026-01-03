cmake_minimum_required(VERSION 3.25)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)

option(ONE_PID_BUILD_EXAMPLES "Build OnePID Examples" OFF)
option(ONE_PID_BUILD_TESTS "Build OnePID Tests" OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (ZEPHYR_TOOLCHAIN_VARIANT)
  project(onepid VERSION 0.1.0)
else ()
  project(OnePID VERSION 0.1.0)
endif ()

if (ZEPHYR_TOOLCHAIN_VARIANT)
  include(OnePidZephyr)
else ()
  include(OnePidNormal)
endif ()

if (ONE_PID_BUILD_EXAMPLES)
  include(OnePidExamples)
endif ()

if (BUILD_OM_TESTS)
  include(OnePidTests)
endif ()

# include(OnePidGenerateDoc)

