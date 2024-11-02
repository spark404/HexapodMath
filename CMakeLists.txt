cmake_minimum_required (VERSION 3.14)
project(HexapodMath)

# Set the location of the CMSIS_DSP Library
if (NOT DEFINED CMSISDSP)
    set(CMSISDSP /Users/hugo/Projects/CMSIS-DSP)
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

enable_testing()

add_library(HexapodMath_Tests STATIC
        ${TEST_SOURCES}
)

target_include_directories(HexapodMath_Tests
        PRIVATE Unity/Inc ${CMSISDSP}/Include PrivateInc Inc
)

add_executable(HexapodMath_Test
        Unity/Src/unity.c
        Unity/Src/unity_fixture.c
        Unity/Src/unity_memory.c
)

target_include_directories(HexapodMath_Test
        PRIVATE Unity/Inc ${CMSISDSP}/Include PrivateInc
)

target_link_directories(HexapodMath_Test
        PRIVATE ${CMSISDSP}/Debug/Source
)

target_link_libraries(HexapodMath_Test
        PUBLIC HexapodMath HexapodMath_Tests
        PRIVATE CMSISDSP
)

add_test(NAME HexapodMath_Test
        COMMAND HexapodMath_Test
)
