# ----------------------------------------------------------------------------------------
#  CMAKE File
#  Copyright (C) 2020 - 2026  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsDrvAvrLib C CXX ASM )

add_library( LcsDrvAvrLib STATIC

    LcsDrvAvrLib.cpp
    LcsDrvAvrLib.h
)

target_include_directories( ${PROJECT_NAME} PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}
)



