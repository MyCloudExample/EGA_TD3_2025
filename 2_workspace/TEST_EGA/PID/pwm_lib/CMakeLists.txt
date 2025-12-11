cmake_minimum_required(VERSION 3.12)
project(pwm_lib)

# Crear la biblioteca estática "pwm_lib" con los archivos fuente
add_library(pwm_lib STATIC
    src/pwm_lib.c
)

# Linkeo dependencias de la bibliotecas
target_link_libraries(pwm_lib
    pico_stdlib
	hardware_pwm
)

# Incluir las cabeceras de la biblioteca
target_include_directories(pwm_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
