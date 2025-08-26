cmake_minimum_required(VERSION 3.12)
project(freertos)

# Conditionally set Pico core (RP2350_ARM_NTZ for pico2 / RP2040 for pico)
if(${PICO_BOARD} STREQUAL "pico2")
    set(CORE RP2350_ARM_NTZ)
elseif(${PICO_BOARD} STREQUAL "pico2_w")
    set(CORE RP2350_ARM_NTZ)
elseif(${PICO_BOARD} STREQUAL "pico")
    set(CORE RP2040)
endif()

# Add FreeRTOS source files
add_library(freertos STATIC
    event_groups.c
    list.c
    queue.c
    stream_buffer.c
    tasks.c
    timers.c
    portable/MemMang/heap_3.c
    portable/GCC/${CORE}/port.c
)

# Add extra source files when core is RP2350
if(${CORE} STREQUAL "RP2350_ARM_NTZ")
target_sources(freertos PRIVATE
    portable/GCC/${CORE}/mpu_wrappers_v2_asm.c
    portable/GCC/${CORE}/portasm.c
)
endif()

# Include FreeRTOS header files
target_include_directories(freertos PUBLIC
    include
    portable/GCC/${CORE}/include
)

# Add dependencies
target_link_libraries(freertos PUBLIC pico_stdlib hardware_exception)