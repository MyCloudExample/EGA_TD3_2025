#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pff.h"
#include "diskio.h"

// Pines SPI para la Raspberry Pi Pico 2
#define PIN_MISO  00
#define PIN_CS    01
#define PIN_SCK   02
#define PIN_MOSI  03

void init_spi_sd(void) {
    spi_init(spi0, 1000 * 1000); // 1 MHz

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SIO); // CS como GPIO normal
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); // Desactivar CS inicialmente
}

int main() {
    stdio_init_all();
    sleep_ms(5000);  // Esperar conexión USB

    printf("Inicializando SPI para SD...\n");
    init_spi_sd();

    FATFS fs;
    FRESULT fr;
    FILE file;
    char buffer[64];
    UINT br;

    printf("Montando sistema de archivos...\n");
    fr = pf_mount(&fs);
    if (fr != FR_OK) {
        printf("Fallo en montaje: %d\n", fr);
        return 1;
    }

    printf("Abriendo archivo 'test.txt'...\n");
    fr = pf_open("test.txt");
    if (fr != FR_OK) {
        printf("Fallo al abrir archivo: %d\n", fr);
        return 1;
    }

    printf("Contenido del archivo:\n");
    while ((fr = pf_read(buffer, sizeof(buffer)-1, &br)) == FR_OK && br > 0) {
        buffer[br] = '\0'; // Asegurar null terminator
        printf("%s", buffer);
    }

    if (fr != FR_OK) {
        printf("Error de lectura: %d\n", fr);
    }

    printf("\nLectura finalizada.\n");
    return 0;
}