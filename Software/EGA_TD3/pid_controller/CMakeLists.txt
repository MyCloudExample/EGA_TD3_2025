cmake_minimum_required(VERSION 3.12)
project(pid_controller)

# Crear la biblioteca estática "pid_controller" con los archivos fuente
add_library(pid_controller STATIC
    src/pid_controller.c
)

# Linkeo dependencias de la bibliotecas
target_link_libraries(pid_controller
    pico_stdlib
    hardware_gpio
)

# Incluir las cabeceras de la biblioteca
target_include_directories(pid_controller PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
