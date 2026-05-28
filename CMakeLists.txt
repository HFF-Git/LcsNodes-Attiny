# ----------------------------------------------------------------------------------------
#  CMAKE File
#  Copyright (C) 2020 - 2026  Helmut Fieres
# ----------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.20)

project(attiny_workspace C CXX)

set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin)

# -------------------------------------------------------------------
# Global configuration
# -------------------------------------------------------------------

set(MCU attiny1614 CACHE STRING "Target MCU")
set(F_CPU 20000000UL CACHE STRING "CPU frequency")

# Output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# AVR compiler flags
add_compile_options(
    -mmcu=${MCU}
    -DF_CPU=${F_CPU}
    -Os
)

add_link_options(
    -mmcu=${MCU}
)

# helper function
function(add_avr_flash_target NAME PORT)

    add_custom_target(${NAME}_flash
        COMMAND pymcuprog write
            -t uart
            -u ${PORT}
            -d ${MCU}
            -f ${BIN_DIR}/${NAME}/${NAME}.hex
        DEPENDS ${NAME}.elf
    )
endfunction()



# -------------------------------------------------------------------
# Libraries
# -------------------------------------------------------------------
add_subdirectory(libraries/LcsDrvAvrLib)

# -------------------------------------------------------------------
# Projects
# -------------------------------------------------------------------
add_subdirectory(LcsDrvAvrLib-Test)
