#include <stdio.h>
#include "pico/stdlib.h"
#include "HC_SR04.h"
#include "pwm_lib.h"
#include "pid_controller.h"
#include "hardware/adc.h"

#define PIN_PWM 11
#define PIN_POT 26  // Pin del potenciómetro (ADC0)
#define SENSOR_DELAY_MS 60

int main()
{
    hc_sr04_t sensor; // Defino la variable relacionada con el sensor donde se definiran los pines
    float medicion; // Esta variable almacena el resultado de la medicion del sensor
    float control; // Se utiliza para almacenar el resultado del control PID
    pwm_config_t fan = {.pin = PIN_PWM, .wrap = 4999, .clk_div = 1.0f};  // Configuro el PWM para el FAN fre=25KHz
    float setpoint = 20.0f; // Indico la altura de flotacion
    float filtrado_distancia = 0.0f; // Se usa como auxiliar para guardar la medicion de distancia filtrada
    const float alpha = 0.3f; // Factor de suavizado
    PIDController example = {.Kp = 0.1f, .Ki = 0.0f, .Kd = 0.0f, .tau = 0.1f, .limMin = 0.0f, .limMax = 1.0f};
    
    // Variables para el potenciómetro
    float pot_value = 0.0f;
    const float Kp_min = 0.01f;  // Valor mínimo de Kp
    const float Kp_max = 1.0f;   // Valor máximo de Kp

    stdio_init_all();
    
    // Inicialización del ADC para el potenciómetro
    adc_init();
    adc_gpio_init(PIN_POT);
    adc_select_input(0);  // Selecciona ADC0 (GPIO26)
    
    hc_sr04_init(&sensor, 14, 15);
    PIDController_Init(&example);
    pwm_init_config(&fan);

    while (true) 
    {
        // Leer valor del potenciómetro y mapearlo a Kp
        uint16_t raw_pot = adc_read();
        pot_value = (float)raw_pot / 4095.0f;  // Normalizar a 0-1
        example.Kp = Kp_min + (Kp_max - Kp_min) * pot_value;
        
        medicion = hc_sr04_get_distance_cm(&sensor);
        if(medicion <= 0 || medicion > 400.0f)
        {
            continue;
        }
        
        filtrado_distancia = alpha * medicion + (1 - alpha) * filtrado_distancia;
        control = PIDController_Update(&example, setpoint, filtrado_distancia, SENSOR_DELAY_MS/1000.0f);
        pwm_set_level(&fan, (uint16_t)(control * 4999.0f));
        
        printf("Setpoint: %.2f, medicion: %.2f, Control: %.2f, Kp: %.2f\n", 
               setpoint, filtrado_distancia, control, example.Kp);
        
        sleep_ms(SENSOR_DELAY_MS);
    }
}