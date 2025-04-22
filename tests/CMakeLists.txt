include(ExternalProject)

ExternalProject_Add(unity-lib
        GIT_REPOSITORY    https://github.com/ThrowTheSwitch/Unity.git
        GIT_TAG           master
        SOURCE_DIR        "${CMAKE_BINARY_DIR}/unity-src"
        BINARY_DIR        "${CMAKE_BINARY_DIR}/unity-build"
        INSTALL_COMMAND   ""
        CMAKE_ARGS        -DUNITY_EXTENSION_FIXTURE=ON
)

add_executable(HexapodMath_Test
        AdditionalFunctions_Test.c
        AdditionalFunctions_TestRunner.c
        ForwardKinematics_Test.c
        ForwardKinematics_TestRunner.c
        Hexapod_Test.c
        Hexapod_TestRunner.c
        HexapodMath_Test.c
        HexapodMath_TestRunner.c
        InverseKinematics_Test.c
        InverseKinematics_TestRunner.c
        Matrix3D_Test.c
        Matrix3D_TestRunner.c
        Runner.c
)

target_include_directories(HexapodMath_Test
        PRIVATE
        include
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

# add_compile_options("$<$<CONFIG:DEBUG>:--coverage>")
# add_link_options("$<$<CONFIG:DEBUG>:--coverage>")