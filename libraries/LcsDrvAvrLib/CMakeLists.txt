# ----------------------------------------------------------------------------------------
#  CMAKE File
#  Copyright (C) 2020 - 2026  Helmut Fieres
# ----------------------------------------------------------------------------------------
project( LcsDrvAvrLib C CXX ASM )

add_library( LcsDrvAvrLib STATIC

    LcsDrvAvrLib.cpp
)

target_include_directories(LcsDrvAvrLib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(LcsDrvAvrLib PUBLIC cxx_std_17)
