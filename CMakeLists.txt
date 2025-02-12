cmake_minimum_required (VERSION 3.14)
project(HexapodMath)

include(ExternalProject)
ExternalProject_Add(unity
        GIT_REPOSITORY    https://github.com/ThrowTheSwitch/Unity.git
        GIT_TAG           master
        SOURCE_DIR        "${CMAKE_BINARY_DIR}/unity-src"
        BINARY_DIR        "${CMAKE_BINARY_DIR}/unity-build"
        INSTALL_COMMAND   ""
        CMAKE_ARGS        -DUNITY_EXTENSION_FIXTURE=ON
)

# Set the location of the CMSIS_DSP Library
if (NOT DEFINED CMSISDSP)
    set(CMSISDSP /Users/hugo/Projects/CMSIS-DSP)
endif()

if (EXISTS "${CMSISDSP}")
  message(STATUS "Using CMSIS-DSP at ${CMSISDSP}")
else()
  message( FATAL_ERROR "No CMSIS-DSP at ${CMSISDSP}" )
endif()

file(GLOB_RECURSE SOURCES "Src/*.c")
file(GLOB_RECURSE TEST_SOURCES "Test/*.c")

# add_compile_options("$<$<CONFIG:DEBUG>:--coverage>")
# add_link_options("$<$<CONFIG:DEBUG>:--coverage>")

add_library(HexapodMath STATIC)

target_include_directories(HexapodMath
        PUBLIC "Inc"
        PRIVATE "PrivateInc" ${CMSISDSP}/Include
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
        PRIVATE Unity/Inc ${CMSISDSP}/Include PrivateInc Inc
)

target_include_directories(HexapodMath_Test
        PRIVATE Unity/Inc ${CMSISDSP}/Include PrivateInc
)

target_link_directories(HexapodMath_Test
        PRIVATE ${CMSISDSP}/cmake-build-debug/Source
)

add_dependencies(HexapodMath_Test
        unity
)

target_link_libraries(HexapodMath_Test
        PRIVATE CMSISDSP HexapodMath
        ${CMAKE_BINARY_DIR}/unity-build/libunity.a
)


add_test(NAME HexapodMath_Test
        COMMAND HexapodMath_Test
)

# Workaround to make CMSIS_DSP includes work on Ubuntu
if (HOST)
    target_compile_definitions(HexapodMath_Test PUBLIC __GNUC_PYTHON__)
    target_link_libraries(HexapodMath_Test PRIVATE m)
endif()

