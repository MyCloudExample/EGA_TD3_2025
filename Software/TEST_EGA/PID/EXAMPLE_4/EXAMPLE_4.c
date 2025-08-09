//En esta cuarta version se mejoro el Kp, Ki, Kd ajustandolo segune el diametro y peso de la pelota
// diametro=7.8 cm ; peso=9 gr
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "HC_SR04.h"
#include "pwm_lib.h"
#include "pid_controller.h"

// Hardware
#define PIN_PWM 11
#define PIN_TRIG 14
#define PIN_ECHO 15
#define PIN_POT 26

// Configuración física
#define BALL_DIAMETER_CM 7.8f
#define BALL_WEIGHT_G 9.0f
#define TARGET_HEIGHT 30.0f
#define SENSOR_HEIGHT 45.0f

// Parámetros iniciales (ajustar durante prueba)
#define BASE_PWM 2500       // Incrementado inicialmente
#define KP 2.0f             // Más agresivo
#define KI 1.2f             // Integral aumentado
#define KD 0.5f             // Más amortiguamiento
#define MIN_PWM 1800        // Mínimo absoluto
#define MAX_PWM 4000        // Máximo seguro

int main() {
    stdio_init_all();
    
    // 1. Verificación PWM
    pwm_config_t fan = {
        .pin = PIN_PWM,
        .wrap = 4999,
        .clk_div = 1.0f
    };
    pwm_init_config(&fan);
    
    // Prueba manual del ventilador
    printf("Prueba manual del ventilador:\n");
    for(int pwm = 1500; pwm <= 3000; pwm += 500) {
        pwm_set_level(&fan, pwm);
        printf("PWM: %d, ¿Está soplando? (espera 3 seg)\n", pwm);
        sleep_ms(3000);
    }

    // 2. Verificación sensor
    hc_sr04_t sensor;
    hc_sr04_init(&sensor, PIN_TRIG, PIN_ECHO);
    printf("\nPrueba del sensor:\n");
    for(int i = 0; i < 5; i++) {
        float dist = hc_sr04_get_distance_cm(&sensor);
        printf("Medición %d: %.2f cm\n", i+1, dist);
        sleep_ms(200);
    }

    // 3. Sistema completo
    PIDController pid = {
        .Kp = KP, .Ki = KI, .Kd = KD,
        .tau = 0.1f,
        .limMin = MIN_PWM/4999.0f,
        .limMax = MAX_PWM/4999.0f,
        .integrator = 0.3f,
        .prevError = 0,
        .differentiator = 0,
        .prevmedicion = TARGET_HEIGHT,
        .out = 0
    };
    PIDController_Init(&pid);

    float filtered_height = TARGET_HEIGHT;
    const float alpha = 0.5f;

    while(true) {
        // A. Medición robusta
        float raw_dist = hc_sr04_get_distance_cm(&sensor);
        float current_height = SENSOR_HEIGHT - raw_dist;
        
        if(raw_dist > 1.0f && raw_dist < SENSOR_HEIGHT) {
            filtered_height = alpha * current_height + (1-alpha) * filtered_height;
        } else {
            printf("! Medición inválida: %.2f cm\n", raw_dist);
            continue;
        }

        // B. Control adaptativo
        float error = TARGET_HEIGHT - filtered_height;
        float control = PIDController_Update(&pid, TARGET_HEIGHT, filtered_height, 0.02f);
        
        // C. PWM con compensación no lineal
        uint16_t pwm = BASE_PWM + (int16_t)(error * 1200.0f);
        pwm = (pwm < MIN_PWM) ? MIN_PWM : (pwm > MAX_PWM) ? MAX_PWM : pwm;
        pwm_set_level(&fan, pwm);

        // D. Diagnóstico detallado
        printf("Alt: %.2fcm | PWM: %4d | Err: %.2f | P: %.2f I: %.2f D: %.2f\n",
              filtered_height, pwm, error,
              pid.Kp * error,
              pid.integrator,
              pid.differentiator);
        
        sleep_ms(20);
    }
}