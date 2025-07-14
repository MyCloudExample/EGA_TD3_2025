#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
/*Este codgi permite ajustar el PWM y mostrarlo por consola, pero ademas se puede vizualizar las RPM del cooler
* La entradas de la Raspberry Pico 2 soporta 5V, por lo que queda a eleccion si usaar o no un divisor de tension.
* para el hilo de teacometro.*/
// Configuración de pines
#define PWM_PIN 11       // GP15 para PWM
#define TACHO_PIN 10     // GP14 para tacómetro
#define POTENTIOMETER_PIN 26 // GP26 (ADC0) para potenciómetro

// Variables para RPM
volatile uint32_t pulse_count = 0;
uint64_t last_time = 0;

// Interrupción para contar pulsos del tacómetro
void tacho_callback(uint gpio, uint32_t events) 
{
    pulse_count++;
}

// Inicializar PWM
void setup_pwm() 
{
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);

    // Configurar frecuencia PWM a 25 kHz (ideal para coolers)
    pwm_set_clkdiv(slice_num, 1.0); // Divisor de reloj (no cambiar)
    pwm_set_wrap(slice_num, 1249);  // 125 MHz / (1249 + 1) = 25 kHz
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0); // Inicia con duty cycle al 0%
    pwm_set_enabled(slice_num, true);
}

// Inicializar ADC para el potenciómetro
void setup_adc() 
{
    adc_init();
    adc_gpio_init(POTENTIOMETER_PIN);
    adc_select_input(0); // Canal ADC0 (GP26)
}

// Inicializar tacómetro
void setup_tacho() 
{
    gpio_init(TACHO_PIN);
    gpio_set_dir(TACHO_PIN, GPIO_IN);
    gpio_pull_up(TACHO_PIN);
    gpio_set_irq_enabled_with_callback(TACHO_PIN, GPIO_IRQ_EDGE_RISE, true, &tacho_callback);
}

// Calcular RPM
float calculate_rpm() 
{
    uint64_t current_time = time_us_64();
    uint64_t elapsed_time = current_time - last_time;
    last_time = current_time;

    // 2 pulsos por revolución (ajusta según tu cooler)
    float rpm = (pulse_count / 2.0) * (60.0 / (elapsed_time / 1e6));
    pulse_count = 0;
    return rpm;
}

int main() 
{
    stdio_init_all();
    setup_pwm();
    setup_adc();
    setup_tacho();

    while (true) 
    {
        // Leer valor del potenciómetro (0-4095)
        uint16_t pot_value = adc_read();
        
        // Mapear a rango PWM (0-1249 para 25 kHz)
        uint16_t pwm_level = (pot_value * 1249) / 4095;
        pwm_set_chan_level(pwm_gpio_to_slice_num(PWM_PIN), PWM_CHAN_B, pwm_level);

        // Calcular y mostrar RPM
        float rpm = calculate_rpm();
        printf("PWM: %d, RPM: %.2f\n", (pwm_level * 100) / 1249, rpm); // PWM en %

        sleep_ms(100); // Pequeño delay para estabilidad
    }
}