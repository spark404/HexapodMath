cmake_minimum_required(VERSION 3.15...4.0)

project(HexapodMath
        VERSION 0.1
        DESCRIPTION "A library with calculations used in hexapod control software"
        LANGUAGES C CXX
)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    include(CTest)

endif()

add_subdirectory(src)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    add_subdirectory(tests)
endif()







