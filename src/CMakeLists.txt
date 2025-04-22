include(FetchContent)

FetchContent_Declare(
        cmsis-dsp
        GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP
        GIT_TAG main
        OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(cmsis-dsp)


set(HEADER_LIST
        "${HexapodMath_SOURCE_DIR}/include/hexapodmath/additional_functions.h"
        "${HexapodMath_SOURCE_DIR}/include/hexapodmath/conversion_2d.h"
        "${HexapodMath_SOURCE_DIR}/include/hexapodmath/forward_kinematics.h"
        "${HexapodMath_SOURCE_DIR}/include/hexapodmath/hexapod.h"
        "${HexapodMath_SOURCE_DIR}/include/hexapodmath/inverse_kinematics.h"
        "${HexapodMath_SOURCE_DIR}/include/hexapodmath/matrix_3d.h"
        "${HexapodMath_SOURCE_DIR}/include/hexapodmath/pose.h"
)

add_library(HexapodMath STATIC
        additional_functions.c
        conversion_2d.c
        forward_kinematics.c
        hexapod.c
        inverse_kinematics.c
        matrix_3d.c
        pose.c
)

target_include_directories(HexapodMath
        PUBLIC ../include ${cmsis-dsp_SOURCE_DIR}/Include
)

source_group(
        TREE "${PROJECT_SOURCE_DIR}/include"
        PREFIX "Header Files"
        FILES ${HEADER_LIST}
)

# Workaround to make CMSIS_DSP includes work on Ubuntu
if (HOST)
    target_compile_definitions(HexapodMath PUBLIC __GNUC_PYTHON__)
endif()

