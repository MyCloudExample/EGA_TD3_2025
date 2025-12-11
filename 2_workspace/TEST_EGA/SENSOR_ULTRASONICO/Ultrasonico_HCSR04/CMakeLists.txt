# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Ultrasonico_HCSR04 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Añadir la subcarpeta donde está la biblioteca BMP280
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../HC_SR04 ${CMAKE_BINARY_DIR}/HC_SR04)

# Add executable. Default name is the project name, version 0.1

add_executable(Ultrasonico_HCSR04 Ultrasonico_HCSR04.c )

pico_set_program_name(Ultrasonico_HCSR04 "Ultrasonico_HCSR04")
pico_set_program_version(Ultrasonico_HCSR04 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Ultrasonico_HCSR04 0)
pico_enable_stdio_usb(Ultrasonico_HCSR04 1)

# Add the standard library to the build
target_link_libraries(Ultrasonico_HCSR04
        pico_stdlib
        HC_SR04
        hardware_timer)

# Add the standard include files to the build
target_include_directories(Ultrasonico_HCSR04 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(Ultrasonico_HCSR04)

