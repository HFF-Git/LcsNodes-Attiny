# ----------------------------------------------------------------------------------------
#  CMAKE File
#  Copyright (C) 2020 - 2026  Helmut Fieres
# ----------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE

    "${CMAKE_SOURCE_DIR}/cmake/toolchain-avr.cmake"
    CACHE STRING "" FORCE)

project(attiny_workspace C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# ----------------------------------------------------------------------------------------
# AVR helper functions used in the projects
# ----------------------------------------------------------------------------------------
function(configure_avr_target TARGET MCU F_CPU OUTPUT_DIR)

    target_compile_options(${TARGET} PRIVATE
        -mmcu=${MCU}
        -DF_CPU=${F_CPU}
        -Os
    )

    target_link_options(${TARGET} PRIVATE
        -mmcu=${MCU}
        -Wl,-Map=${OUTPUT_DIR}/${TARGET}.map
    )

endfunction()

function(add_avr_library name)

    add_library(${name} STATIC ${ARGN})
    configure_avr_target(${name} ${MCU} ${F_CPU} ${OUTPUT_DIR})

endfunction()

function(add_avr_flash_target NAME PORT)

    add_custom_target(${NAME}_flash
        COMMAND pymcuprog write
            -t uart
            -u ${PORT}
            -d ${MCU}
            -f ${BIN_DIR}/${NAME}/${NAME}.hex
        DEPENDS ${NAME}
    )
endfunction()

# ----------------------------------------------------------------------------------------
# Projects and libraries
# ----------------------------------------------------------------------------------------
add_subdirectory( libraries/LcsDrvAvrLib )
add_subdirectory( LcsDrvAvrLib-Test )
add_subdirectory( LcsDrvTurnoutBoard ) 
add_subdirectory( LcsDrvServoBoard ) 
add_subdirectory( LcsDrvSignalBoard )
add_subdirectory( LcsDrvOccDetectBoard)
