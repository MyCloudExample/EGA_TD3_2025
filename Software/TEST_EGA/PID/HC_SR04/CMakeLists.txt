cmake_minimum_required(VERSION 3.12)
project(HC_SR04)

# Crear la biblioteca estática "HC_SR04" con los archivos fuente
add_library(HC_SR04 STATIC
    src/HC_SR04.c
)

# Linkeo dependencias de la bibliotecas
target_link_libraries(HC_SR04
    pico_stdlib
hardware_timer
hardware_gpio
)

# Incluir las cabeceras de la biblioteca
target_include_directories(HC_SR04 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
