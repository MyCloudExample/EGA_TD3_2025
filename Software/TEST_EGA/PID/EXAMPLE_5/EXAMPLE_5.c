#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "HC_SR04.h"
#include "pwm_lib.h"
#include "pid_controller.h"

// --------------------------------------------------
// CONFIGURACIÓN HARDWARE (EXACTAMENTE COMO FUNCIONÓ)
// --------------------------------------------------
#define PIN_PWM 11       // Conectado al ventilador
#define PIN_TRIG 14      // HC-SR04 Trigger
#define PIN_ECHO 15      // HC-SR04 Echo
#define PIN_POT 26       // Opcional para ajustes

// --------------------------------------------------
// PARÁMETROS QUE YA FUNCIONARON
// --------------------------------------------------
#define BALL_DIAMETER_CM 7.8f
#define BALL_WEIGHT_G 9.0f
#define TARGET_HEIGHT 20.0f  // 20cm desde el piso
#define SENSOR_HEIGHT 45.0f  // Sensor a 45cm del piso
#define CONTROL_PERIOD_MS 20  // Intervalo de control

// --------------------------------------------------
// CONSTANTES PID ORIGINALES (NO MODIFICAR)
// --------------------------------------------------
#define BASE_PWM 2500    // Valor base que funcionó
#define KP 2.0f          // Ganancia Proporcional
#define KI 1.2f          // Ganancia Integral  
#define KD 0.5f          // Ganancia Derivativa
#define MIN_PWM 1800     // Mínimo seguro
#define MAX_PWM 3200     // Máximo seguro

int main() {
    // 1. Inicialización (idéntica a la versión funcional)
    stdio_init_all();
    
    // 2. Configuración PWM original
    pwm_config_t fan = {
        .pin = PIN_PWM,
        .wrap = 4999,      // Frecuencia ~25kHz
        .clk_div = 1.0f
    };
    pwm_init_config(&fan);

    // 3. Inicialización sensor ultrasónico
    hc_sr04_t sensor;
    hc_sr04_init(&sensor, PIN_TRIG, PIN_ECHO);

    // 4. Parámetros PID originales
    PIDController pid = {
        .Kp = KP, .Ki = KI, .Kd = KD,
        .tau = 0.1f,
        .limMin = MIN_PWM/4999.0f,
        .limMax = MAX_PWM/4999.0f,
        .integrator = 0.3f,  // Valor inicial que funcionó
        .prevError = 0,
        .differentiator = 0,
        .prevmedicion = TARGET_HEIGHT,
        .out = 0
    };
    PIDController_Init(&pid);

    // 5. Variables de control originales
    float filtered_height = TARGET_HEIGHT;
    const float alpha = 0.5f;  // Factor de filtrado original

    while(true) {
        // A. Medición idéntica a la versión funcional
        float raw_dist = hc_sr04_get_distance_cm(&sensor);
        if(raw_dist < 2.0f || raw_dist > SENSOR_HEIGHT) {
            printf("! Medición inválida: %.2f cm\n", raw_dist);
            continue;
        }
        
        // B. Cálculo de altura filtrada (sin cambios)
        float current_height = SENSOR_HEIGHT - raw_dist;
        filtered_height = alpha * current_height + (1-alpha) * filtered_height;

        // C. Control PID original
        float error = TARGET_HEIGHT - filtered_height;
        float control = PIDController_Update(&pid, TARGET_HEIGHT, filtered_height, CONTROL_PERIOD_MS/1000.0f);
        
        // D. Cálculo de PWM original
        uint16_t pwm_value = BASE_PWM + (int16_t)(control * 1000.0f);
        pwm_value = (pwm_value < MIN_PWM) ? MIN_PWM : (pwm_value > MAX_PWM) ? MAX_PWM : pwm_value;
        
        pwm_set_level(&fan, pwm_value);

        // E. Monitorización (igual a la versión funcional)
        printf("Alt: %.2fcm | PWM: %4d | Err: %.2f\n", 
              filtered_height, pwm_value, error);
        
        sleep_ms(CONTROL_PERIOD_MS);
    }
}