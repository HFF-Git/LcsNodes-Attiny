
//========================================================================================
//
//
//
//========================================================================================
// cmake toolchain ?

#if 0 

<example>

attiny414-i2c/
├── CMakeLists.txt
├── main.c
└── toolchain-avr.cmake

<toolchain-avr.cmake>

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER avr-gcc)

set(CMAKE_C_FLAGS "-mmcu=attiny414 -Os -DF_CPU=16000000UL")

set(CMAKE_EXE_LINKER_FLAGS "-mmcu=attiny414")


<CMakeLists.txt>

cmake_minimum_required(VERSION 3.13)
project(attiny_i2c C)

add_executable(attiny_i2c.elf
    main.c
)

add_custom_target(flash
    COMMAND avrdude -p t414 -c serialupdi -P /dev/ttyUSB0 -b 115200 -U flash:w:attiny_i2c.elf
    DEPENDS attiny_i2c.elf
)

<usage>

mkdir build
cd build
cmake ..
make
make flash

#endif


//========================================================================================
//
//
//
//========================================================================================
// combined environment

#if 0
// root CmakeLists.txt

cmake_minimum_required(VERSION 3.22)
project(monorepo LANGUAGES C CXX)

add_subdirectory(pico)
add_subdirectory(attiny)

// attiny toolchain file: attiny/cmake/avr-toolchain.cmake

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)
set(CMAKE_ASM_COMPILER avr-gcc)

set(CMAKE_OBJCOPY avr-objcopy)
set(CMAKE_SIZE avr-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

// attiny/CMakeLists.txt file

cmake_minimum_required(VERSION 3.22)

project(attiny414_firmware C)

set(MCU attiny414)
set(F_CPU 20000000UL)

add_executable(attiny414
    src/main.c
    servo/servo.c
)

target_include_directories(attiny414 PRIVATE
    servo
    ${CMAKE_SOURCE_DIR}/common
)

target_compile_options(attiny414 PRIVATE
    -mmcu=${MCU}
    -DF_CPU=${F_CPU}
    -Os
    -Wall
    -std=gnu11
)

target_link_options(attiny414 PRIVATE
    -mmcu=${MCU}
)

add_custom_command(TARGET attiny414 POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
        -O ihex
        $<TARGET_FILE:attiny414>
        attiny414.hex
)

add_custom_target(flash
    COMMAND avrdude -p t414 -c usbtiny -U flash:w:attiny414.hex
    DEPENDS attiny414
)

// CmakePresets.json at REPO root...

{
  "version": 5,
  "configurePresets": [
    {
      "name": "pico",
      "binaryDir": "${sourceDir}/pico/build",
      "generator": "Ninja"
    },
    {
      "name": "attiny",
      "binaryDir": "${sourceDir}/attiny/build",
      "generator": "Ninja",
      "toolchainFile": "attiny/cmake/avr-toolchain.cmake"
    }
  ]
}

#endif
