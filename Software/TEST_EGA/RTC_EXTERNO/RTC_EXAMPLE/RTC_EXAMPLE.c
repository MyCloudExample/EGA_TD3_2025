#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ds3231.h"

int main() {
    // Inicializa USB con espera explícita
    stdio_usb_init();
    
    // Espera hasta que el USB esté realmente conectado
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    
    printf("\n\n=== Inicializando sistema ===\n");
    
    // Resto de tu inicialización
    i2c_inst_t *i2c = i2c0;
    if (!ds3231_init(i2c, 8, 9, 100000)) {
        printf("Error al inicializar DS3231!\n");
        while(1); // Bloquea si hay error
    }
    
    printf("DS3231 inicializado correctamente\n");
    
    // Ejemplo de lectura del RTC
    ds3231_time_t current_time;
    while (true) {
        if (ds3231_get_time(i2c, &current_time)) {
            printf("Hora: %02d:%02d:%02d - Fecha: %02d/%02d/20%02d\n",
                  current_time.hours,
                  current_time.minutes,
                  current_time.seconds,
                  current_time.date,
                  current_time.month,
                  current_time.year);
        } else {
            printf("Error leyendo el RTC\n");
        }
        sleep_ms(1000);
    }
    
    return 0;
}