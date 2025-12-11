#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <stdio.h>  // Para printf

#define LED_PIN 11          // GPIO15 para PWM, pin 21
#define POTENTIOMETER_PIN 26 // GPIO26 (ADC0), pin 31

int main() 
{
    stdio_init_all(); // Inicializar UART para printf

    // Mensaje inicial (para verificar que la consola funciona)
    printf("Iniciando control de LED con potenciómetro...\n");

    // 1. Configurar ADC para el potenciómetro
    adc_init();
    adc_gpio_init(POTENTIOMETER_PIN);
    adc_select_input(0);  // ADC0 (GPIO26)

    // 2. Configurar PWM para el LED
    gpio_init(LED_PIN);
    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(LED_PIN);
    uint channel_num = pwm_gpio_to_channel(LED_PIN);

    // Frecuencia PWM: ~1kHz (wrap = 1000)
    pwm_set_clkdiv(slice_num, 4.0f);
    pwm_set_wrap(slice_num, 1000);
    pwm_set_enabled(slice_num, true);

    while (true) 
    {
        // Leer valor del potenciómetro (0-4095)
        uint16_t adc_value = adc_read();

        // Mapear ADC (0-4095) -> PWM (0-1000)
        uint16_t duty = adc_value / 4; // 4095 / 4 ≈ 1023 (ajustado a 1000)
        if (duty > 1000) duty = 1000;

        // Aplicar el duty cycle al PWM
        pwm_set_chan_level(slice_num, channel_num, duty);

        // Mostrar el duty cycle en la consola (formato: "Duty: XXX/1000 (XX.X%)\n")
        printf("Duty: %d/1000 (%.1f%%)\n", duty, (float)duty / 10.0f);

        // Pequeño delay para evitar saturar la consola
        sleep_ms(200); // 200ms entre lecturas
    }
}