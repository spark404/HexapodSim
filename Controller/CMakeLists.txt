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
            GIT_TAG e8b53c79cb186cb58f96ad53b70566932db14753
            OVERRIDE_FIND_PACKAGE
    )
    FetchContent_MakeAvailable(HexapodMath)
endif ()

add_executable(Controller main.cpp
        Controller.cpp
        Gait.cpp
        Standup.cpp
        ServoTest.cpp
)

target_link_libraries(Controller PRIVATE
        gz-transport14::core
        HexapodMath
        CMSISDSP
)
