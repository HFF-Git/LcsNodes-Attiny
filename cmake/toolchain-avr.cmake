# ------------------------------------------------------------
# AVR cross compiler toolchain
# ------------------------------------------------------------

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

# Prevent macOS from injecting -arch arm64
set(CMAKE_OSX_ARCHITECTURES "")

# Compilers
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

# AVR tools
set(CMAKE_OBJCOPY avr-objcopy)
set(CMAKE_SIZE avr-size)

# Embedded targets cannot run test executables
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)