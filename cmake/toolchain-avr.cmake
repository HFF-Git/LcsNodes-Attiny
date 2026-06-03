# ------------------------------------------------------------
# AVR cross compiler toolchain
# ------------------------------------------------------------

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

# Prevent macOS from injecting -arch arm64
set(CMAKE_OSX_ARCHITECTURES "")

# Compilers
set(AVR_ROOT
    "/Users/helmutfieres/Library/Arduino15/packages/DxCore/tools/avr-gcc/7.3.0-atmel3.6.1-azduino7b1")

set(CMAKE_C_COMPILER "${AVR_ROOT}/bin/avr-gcc")

set(CMAKE_CXX_COMPILER "${AVR_ROOT}/bin/avr-g++")

set(CMAKE_OBJCOPY "${AVR_ROOT}/bin/avr-objcopy")

set(CMAKE_SIZE "${AVR_ROOT}/bin/avr-size")


# AVR tools
set(CMAKE_OBJCOPY avr-objcopy)
set(CMAKE_SIZE avr-size)

# Embedded targets cannot run test executables
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)