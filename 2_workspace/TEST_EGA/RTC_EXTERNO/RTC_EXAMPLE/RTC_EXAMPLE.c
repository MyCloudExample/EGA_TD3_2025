#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ds3231.h"

int main() {
    // Inicializa USB con espera explícita
    stdio_usb_init();
    i2c_inst_t *i2c = i2c0;
    
    // Espera hasta que el USB esté realmente conectado
    while (!stdio_usb_connected()) 
    {
        sleep_ms(100);
    }
    
    printf("\n\n=== Inicializando sistema ===\n");
    
    if (!ds3231_init(i2c, 8, 9, 100000)) 
    {
        printf("Error al inicializar DS3231!\n");
        while(1); // Bloquea si hay error
    }
    
    printf("DS3231 inicializado correctamente\n");
    
    // NUEVO: Configurar la fecha y hora
    ds3231_time_t new_time = {
        .seconds = 0,    // 0-59
        .minutes = 05,   // 0-59
        .hours = 21,     // 0-23 (formato 24 horas)
        .day = 4,        // 1-7 (día de la semana, 1 = domingo, 2 = lunes, etc.)
        .date = 27,      // 1-31 (día del mes)
        .month = 8,      // 1-12
        .year = 25       // 00-99 (año 2000 + año)
    };
    
    if (ds3231_set_time(i2c, &new_time)) {
        printf("Hora configurada correctamente: %02d:%02d:%02d - %02d/%02d/20%02d\n",
              new_time.hours, new_time.minutes, new_time.seconds,
              new_time.date, new_time.month, new_time.year);
    } else {
        printf("Error al configurar la hora\n");
    }
    
    // Ejemplo de lectura del RTC
    ds3231_time_t current_time;
    while (true) 
    {
        if (ds3231_get_time(i2c, &current_time)) 
        {
            printf("Hora actual: %02d:%02d:%02d - Fecha: %02d/%02d/20%02d\n",
                  current_time.hours,
                  current_time.minutes,
                  current_time.seconds,
                  current_time.date,
                  current_time.month,
                  current_time.year);
        } 
        else
        {
            printf("Error leyendo el RTC\n");
        }
        sleep_ms(1000);
    }
    
    return 0;
}