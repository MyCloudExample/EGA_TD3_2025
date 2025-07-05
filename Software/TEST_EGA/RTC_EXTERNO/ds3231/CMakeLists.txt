cmake_minimum_required(VERSION 3.12)
project(ds3231)

# Crear la biblioteca estática "ds3231" con los archivos fuente
add_library(ds3231 STATIC
    src/ds3231.c
)

# Linkeo dependencias de la bibliotecas
target_link_libraries(ds3231
    pico_stdlib
	hardware_i2c
)

# Incluir las cabeceras de la biblioteca
target_include_directories(ds3231 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
