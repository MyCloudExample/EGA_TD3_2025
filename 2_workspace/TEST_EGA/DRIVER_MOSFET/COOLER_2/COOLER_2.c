#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define PIN_PWM 11  // Usando GPIO10
#define PIN_LED 25  // LED onboard para diagnóstico

void pwm_test_init() {
    // Configuración precisa del PWM
    gpio_set_function(PIN_PWM, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PIN_PWM);
    uint channel = pwm_gpio_to_channel(PIN_PWM); // ¡Obtenemos el canal correcto!
    
    printf("Slice: %d, Channel: %d\n", slice_num, channel); // Debug
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 4999);
    pwm_config_set_clkdiv(&config, 1.0f); // Frecuencia ~25kHz
    pwm_init(slice_num, &config, true);
}

int main() {
    stdio_init_all();
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    
    pwm_test_init();
    uint slice_num = pwm_gpio_to_slice_num(PIN_PWM);
    uint channel = pwm_gpio_to_channel(PIN_PWM);
    
    printf("Prueba PWM en GPIO%d\n", PIN_PWM);
    
    while(true) {
        // Barrido completo con feedback visual
        for(int i=0; i<=5000; i+=1000) {
            gpio_put(PIN_LED, 1); // LED ON durante el cambio
            pwm_set_chan_level(slice_num, channel, i); // ¡Usando canal específico!
            printf("Nivel PWM: %d/4999 (%.1f%%)\n", i, (i*100.0f)/4999.0f);
            sleep_ms(2000);
            gpio_put(PIN_LED, 0);
            sleep_ms(200);
        }
    }
}