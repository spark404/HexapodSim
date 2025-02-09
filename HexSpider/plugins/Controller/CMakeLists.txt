cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(hexspider_controller)

# --------------------------------------------------------------------------- #
# Compile as C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.

# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-cmake3 REQUIRED)
  set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

  gz_find_package(gz-common5 REQUIRED)
  set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

  gz_find_package(gz-rendering8 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

  gz_find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")
else()  
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()

add_library(HexspiderControllerPlugin
    SHARED
    src/HexspiderControllerPlugin.cc
)

target_include_directories(HexspiderControllerPlugin PRIVATE
        include
)

target_link_libraries(HexspiderControllerPlugin PRIVATE
        gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(
        TARGETS
        HexspiderControllerPlugin
        DESTINATION lib/${PROJECT_NAME}
)
