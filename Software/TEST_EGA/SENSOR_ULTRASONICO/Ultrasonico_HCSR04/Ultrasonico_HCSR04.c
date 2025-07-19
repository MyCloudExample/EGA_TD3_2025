#include <stdio.h>
#include "pico/stdlib.h"
#include "HC_SR04.h"

int main() {
    stdio_init_all();
    sleep_ms(2000); // Esperar estabilización USB
    
    hc_sr04_t sensor;
    hc_sr04_init(&sensor, 14, 15); // Trigger: GPIO2, Echo: GPIO3
    
    while(true) {
        float dist = hc_sr04_get_distance_cm(&sensor);
        if(dist > 0) {
            printf("Distancia: %.2f cm\n", dist);
        } else {
            printf("Fuera de rango\n");
        }
        sleep_ms(100);
    }
}