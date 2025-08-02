#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

#define PIN_PWM 11

int main() {
    stdio_init_all();
    gpio_set_function(PIN_PWM, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PIN_PWM);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 4999);
    pwm_init(slice_num, &config, true);
    
    while (true) {
        // Barrido completo de PWM
        for (int i = 0; i <= 4999; i += 500) {
            pwm_set_gpio_level(PIN_PWM, i);
            printf("PWM: %d/4999 (%.1f%%)\n", i, (i*100.0f)/4999.0f);
            sleep_ms(2000);
        }
    }
}