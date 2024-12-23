cmake_minimum_required(VERSION 3.28)
project(Controller)

set(CMAKE_CXX_STANDARD 17)

set(CMSISDSP /Users/hugo/Projects/CMSIS-DSP)
set(HEXAMATH /Users/hugo/Projects/Hexapod/HexapodMath)
# set(CMSISCORE ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS)

file(GLOB_RECURSE HEXAMATH_SOURCES "HexapodMath/Src/*.c")

find_package(gz-transport NAMES gz-transport14)
add_executable(Controller main.cpp
        Controller.cpp
        Gait.cpp
)
target_link_libraries(Controller PRIVATE
        gz-transport14::core
)

add_subdirectory(${CMSISDSP}/Source bin_dsp)
link_directories(${PROJECT_BINARY_DIR}/bin_dsp)

add_subdirectory(${HEXAMATH} hexapod_math_bin)
target_link_libraries(Controller PRIVATE
        CMSISDSP
        HexapodMath
)

#include(FetchContent)
#FetchContent_Declare(
#        googletest
#        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
#)
#FetchContent_MakeAvailable(googletest)
#
#enable_testing()
#
#include(GoogleTest)
#
#add_executable(
#        test_GeoMath
#)
#
#target_link_libraries(
#        test_GeoMath
#        GTest::gtest_main
#)
#
#gtest_discover_tests(test_GeoMath)