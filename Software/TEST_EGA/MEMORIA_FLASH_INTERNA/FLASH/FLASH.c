#include "pico/stdlib.h"
#include "hardware/flash.h"
#include <string.h>
#include <stdio.h>  // Necesario para printf

// Usamos las definiciones del SDK en lugar de redefinirlas
// #define FLASH_PAGE_SIZE 256  // Eliminar - ya definido en flash.h
// #define FLASH_SECTOR_SIZE 4096  // Eliminar - ya definido en flash.h

// Definimos el offset desde el inicio de la flash (reservamos espacio para el programa)
#define FLASH_TARGET_OFFSET (1024 * 1024)  // 1MB de offset
#define DATA_SIZE 512  // Tamaño de nuestros datos

// Escribe datos en la flash
void write_to_flash(const uint8_t *data, uint32_t length) 
{
    // Debemos borrar toda la sector antes de escribir
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);    
    // Escribimos los datos
    flash_range_program(FLASH_TARGET_OFFSET, data, length);
}

// Lee datos de la flash
void read_from_flash(uint8_t *buffer, uint32_t length) 
{
    // La memoria flash está mapeada en la dirección XIP_BASE
    const uint8_t *flash_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(buffer, flash_contents, length);
}

int main() 
{
    stdio_init_all();
    
    // Esperar a que la conexión serial esté lista
    sleep_ms(5000);
    printf("\nIniciando prueba de memoria flash...\n");
    
    // Datos de ejemplo para guardar
    uint8_t write_data[DATA_SIZE];
    uint8_t read_data[DATA_SIZE];
    
    // Llenamos el buffer con datos de prueba
    for (int i = 0; i < DATA_SIZE; i++) 
    {
        write_data[i] = i % 256;
    }
    
    printf("Escribiendo en la flash...\n");
    write_to_flash(write_data, DATA_SIZE);
    
    printf("Leyendo de la flash...\n");
    read_from_flash(read_data, DATA_SIZE);
    
    // Verificamos los datos
    bool success = true;
    for (int i = 0; i < DATA_SIZE; i++) 
    {
        if (write_data[i] != read_data[i]) 
        {
            printf("Error en el byte %d\n", i);
            success = false;
            break;
        }
    }
    
    if (success) 
    {
        printf("Lectura/escritura verificada correctamente!\n");
    } 
    else 
    {
        printf("Hubo errores en la verificacion\n");
    }
    
    // Ejemplo de uso práctico: almacenar configuración
    typedef struct 
    {
        uint32_t magic_number;
        uint8_t config_version;
        uint16_t settings[10];
        char device_name[32];
    } device_config_t;
    
    device_config_t config = 
    {
        .magic_number = 0xABCD1234,
        .config_version = 1,
        .device_name = "Mi Dispositivo Pico"
    };
    
    printf("\nGuardando configuracion del dispositivo...\n");
    write_to_flash((uint8_t *)&config, sizeof(config));
    
    device_config_t loaded_config;
    read_from_flash((uint8_t *)&loaded_config, sizeof(loaded_config));
    
    if (loaded_config.magic_number == 0xABCD1234) 
    {
        printf("Config cargada: %s (v%d)\n", 
               loaded_config.device_name, 
               loaded_config.config_version);
    } 
    else 
    {
        printf("Config invalida o no encontrada\n");
    }
    
    return 0;
}