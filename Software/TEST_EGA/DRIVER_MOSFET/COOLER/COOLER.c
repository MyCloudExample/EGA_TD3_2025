#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
/*La finalidad de este codigo es servir de prueba para testear el circuito DRIVER que controlara el ventilador. Para el
* testeo se usara, en primer instacia se usara un led. Luego se procedara a verificar con un cooler que tiene 
* disponible un pin de PWM*/
/*-------------------------------------------------DEFINICIONES----------------------------------------------------------------*/
#define LED_PIN     15         // GPIO pin conectado al LED (a través de resistencia)
#define PWM_WRAP    64500      // 1 kHz - frecuencia adecuada para LEDs
#define PWM_DIV     4.0f       // Brillo inicial al 30% (ajusta este valor)
#define delay       50
/*------------------------------------------------FUNCIONES--------------------------------------------------------------------*/
int main() 
{
    // Inicializar el LED
    gpio_init(LED_PIN);
    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);

    // Obtener el slice y canal del PWM para el pin
    uint slice_num = pwm_gpio_to_slice_num(LED_PIN);
    uint channel_num = pwm_gpio_to_channel(LED_PIN);

    // Configurar frecuencia del PWM (ejemplo: 1kHz)
    pwm_set_clkdiv(slice_num, PWM_DIV);  // Divisor de frecuencia (ajustar según necesidad)
    pwm_set_wrap(slice_num, PWM_WRAP);   // Wrap value para ~1kHz con divisor 4
    pwm_set_enabled(slice_num, true); // Habilitar PWM

    // Bucle principal para variar el brillo
    while (true) 
    {
        for (int duty = 0; duty <= 65535; duty += 1000) {
            pwm_set_chan_level(slice_num, channel_num, duty);
            sleep_ms(delay);
        }
        for (int duty = 65535; duty >= 0; duty -= 1000) {
            pwm_set_chan_level(slice_num, channel_num, duty);
            sleep_ms(delay);
        }
    }
}