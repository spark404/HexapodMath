cmake_minimum_required (VERSION 3.14)
project(HexapodMath)

include(FetchContent)
include(ExternalProject)

ExternalProject_Add(unity-lib
        GIT_REPOSITORY    https://github.com/ThrowTheSwitch/Unity.git
        GIT_TAG           master
        SOURCE_DIR        "${CMAKE_BINARY_DIR}/unity-src"
        BINARY_DIR        "${CMAKE_BINARY_DIR}/unity-build"
        INSTALL_COMMAND   ""
        CMAKE_ARGS        -DUNITY_EXTENSION_FIXTURE=ON
)

FetchContent_Declare(
        cmsis-dsp
        GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP
        GIT_TAG main
        OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(cmsis-dsp)


file(GLOB_RECURSE SOURCES "Src/*.c")
file(GLOB_RECURSE TEST_SOURCES "Test/*.c")

# add_compile_options("$<$<CONFIG:DEBUG>:--coverage>")
# add_link_options("$<$<CONFIG:DEBUG>:--coverage>")

add_library(HexapodMath STATIC)

target_include_directories(HexapodMath
        PUBLIC
        Inc
        PRIVATE
        PrivateInc
        ${cmsis-dsp_SOURCE_DIR}/Include
)

target_sources(HexapodMath
        PRIVATE ${SOURCES}
)

# Workaround to make CMSIS_DSP includes work on Ubuntu
if (HOST)
    target_compile_definitions(HexapodMath PUBLIC __GNUC_PYTHON__)
endif()

enable_testing()

add_executable(HexapodMath_Test
        ${TEST_SOURCES}
)

target_include_directories(HexapodMath_Test
        PRIVATE
        PrivateInc
        Inc
        ${CMAKE_BINARY_DIR}/unity-src/src
        ${CMAKE_BINARY_DIR}/unity-src/extras/fixture/src
        ${CMAKE_BINARY_DIR}/unity-src/extras/memory/src
        ${CMAKE_BINARY_DIR}/cmsis-dsp-src/Include
)

target_link_directories(HexapodMath_Test
        PRIVATE
        ${CMAKE_BINARY_DIR}/unity-build/
)

add_dependencies(HexapodMath_Test
        unity-lib
)

target_link_libraries(HexapodMath_Test
        PRIVATE
        HexapodMath
        CMSISDSP
        unity
)

add_test(NAME HexapodMath_Test
        COMMAND HexapodMath_Test
)

# Workaround to make CMSIS_DSP includes work on Ubuntu
if (HOST)
    target_compile_definitions(HexapodMath_Test PUBLIC __GNUC_PYTHON__)
    target_link_libraries(HexapodMath_Test PRIVATE m)
endif()

