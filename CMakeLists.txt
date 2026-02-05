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

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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

if (NOT ZEPHYR_TOOLCHAIN_VARIANT)
  include(CMakePackageConfigHelpers)

  set(INSTALL_CONFIGDIR "lib/cmake/${PROJECT_NAME}")

  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    INCLUDES DESTINATION include
  )

  install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${INSTALL_CONFIGDIR}
  )

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include
  )

  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
  )

  configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/OnePidConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
  )

  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${INSTALL_CONFIGDIR}
  )
endif ()
