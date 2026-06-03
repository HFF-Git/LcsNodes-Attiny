# ----------------------------------------------------------------------------------------
# Project CMAKE File
# ----------------------------------------------------------------------------------------
set(PROJECT_NAME LcsDrvOccDetectBoard)

set(PROJECT_OUTPUT_DIR ${BIN_DIR}/${PROJECT_NAME})
file(MAKE_DIRECTORY ${PROJECT_OUTPUT_DIR})

set(MCU attiny1614 CACHE STRING "Target MCU")
set(F_CPU 20000000UL CACHE STRING "CPU frequency")

# ----------------------------------------------------------------------------------------
# Build executable
# ----------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}

    main.cpp
)

target_link_libraries(${PROJECT_NAME}

    LcsDrvAvrLib
)

configure_avr_target(

    ${PROJECT_NAME}
    ${MCU}
    ${F_CPU}
    ${PROJECT_OUTPUT_DIR}
)

configure_avr_target(LcsDrvAvrLib 

    ${MCU} 
    ${F_CPU} 
    ${PROJECT_OUTPUT_DIR}
)

# ----------------------------------------------------------------------------------------
# Output location + suffix
# ----------------------------------------------------------------------------------------
set_target_properties(${PROJECT_NAME} PROPERTIES

    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_OUTPUT_DIR}
    SUFFIX ".elf"
)

# ----------------------------------------------------------------------------------------
# Generate HEX
# ----------------------------------------------------------------------------------------
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD

    COMMAND avr-objcopy -O ihex
        ${PROJECT_OUTPUT_DIR}/${PROJECT_NAME}.elf
        ${PROJECT_OUTPUT_DIR}/${PROJECT_NAME}.hex
)

# ----------------------------------------------------------------------------------------
# Print size
# ----------------------------------------------------------------------------------------
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD

    COMMAND avr-size
        ${PROJECT_OUTPUT_DIR}/${PROJECT_NAME}.elf
)

# ----------------------------------------------------------------------------------------
# Flash target
# ----------------------------------------------------------------------------------------
add_custom_target(${PROJECT_NAME}_flash

    COMMAND pymcuprog write
        -t uart
        -u /dev/cu.usbserial-0001
        -d ${MCU}
        -f ${PROJECT_OUTPUT_DIR}/${PROJECT_NAME}.hex
    DEPENDS ${PROJECT_NAME}
)