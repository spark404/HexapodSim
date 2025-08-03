cmake_minimum_required(VERSION 3.28)
project(Controller)

include(ExternalProject)
include(FetchContent)

set(CMAKE_CXX_STANDARD 17)

find_package(gz-transport NAMES gz-transport14)

find_package(cmsis-dsp QUIET)
if (NOT cmsis-dsp_FOUND)
    FetchContent_Declare(
            cmsis-dsp
            GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP
            GIT_TAG main
            OVERRIDE_FIND_PACKAGE
    )
    FetchContent_MakeAvailable(cmsis-dsp)
endif ()

find_package(HexapodMath QUIET)
if (NOT HexapodMath_FOUND)
    FetchContent_Declare(
            HexapodMath
            GIT_REPOSITORY https://github.com/spark404/HexapodMath
            GIT_TAG beb4118ac1a2d5be3d803ecce902c02d2327fb02
            OVERRIDE_FIND_PACKAGE
    )
    FetchContent_MakeAvailable(HexapodMath)
endif ()

add_executable(Controller main.cpp
        Controller.cpp
        calculator.c
)

target_link_libraries(Controller PRIVATE
        gz-transport14::core
        HexapodMath
        CMSISDSP
)
