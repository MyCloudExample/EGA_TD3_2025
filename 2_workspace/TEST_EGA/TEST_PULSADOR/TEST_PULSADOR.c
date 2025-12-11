#include "pico/stdlib.h"
#include <stdio.h>

#define BUTTON_PIN 18  // Cambia esto al pin GPIO que estés usando

int main() {
    // Inicializar la biblioteca estándar
    stdio_init_all();
    
    // Inicializar el pin del botón como entrada
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    
    // No activamos la resistencia pull-up interna ya que usamos una externa
    // gpio_pull_up(BUTTON_PIN);
    
    printf("Pulsador con resistencia pull-up externa - prueba\n");
    
    while (true) {
        // Leer el estado del botón
        // Con pull-up externa, el botón presionado dará 0 (LOW)
        bool button_state = gpio_get(BUTTON_PIN);
        
        if (!button_state) {
            printf("Botón presionado!\n");
        } else {
            printf("Botón no presionado\n");
        }
        
        // Pequeño retardo para evitar rebotes y saturación de la salida
        sleep_ms(200);
    }
    
    return 0;
}