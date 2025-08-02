#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "HC_SR04.h"
#include "pwm_lib.h"
#include "pid_controller.h"

#define PIN_PWM 11
#define SENSOR_HEIGHT 45.0f
#define SETPOINT 20.0f
#define SAMPLE_TIME_MS 30

// Parámetros optimizados
#define MIN_PWM 1800       // Aumentado para contrarrestar gravedad
#define MAX_PWM 4200
#define KP 1.5f            // Ganancia proporcional
#define KI 0.7f            // Ganancia integral (aumentada)
#define KD 0.2f            // Ganancia derivativa
#define ANTI_GRAVITY_BIAS 0.12f  // Compensación adicional

int main() {
    stdio_init_all();
    hc_sr04_t sensor;
    pwm_config_t fan = {.pin = PIN_PWM, .wrap = 4999, .clk_div = 1.0f};
    
    PIDController pid = {
        .Kp = KP,
        .Ki = KI,
        .Kd = KD,
        .tau = 0.1f,
        .limMin = MIN_PWM/4999.0f,
        .limMax = MAX_PWM/4999.0f,
        .integrator = 0,
        .prevError = 0,
        .differentiator = 0,
        .prevmedicion = 0,  // Usando el nombre correcto de tu campo
        .out = 0
    };

    hc_sr04_init(&sensor, 14, 15);
    PIDController_Init(&pid);
    pwm_init_config(&fan);

    float distancia_piso, filtrado = SETPOINT;
    const float alpha = 0.5f;

    while(true) {
        // 1. Medición y filtrado
        float medicion = hc_sr04_get_distance_cm(&sensor);
        if(medicion > 2.0f && medicion < SENSOR_HEIGHT) {
            distancia_piso = SENSOR_HEIGHT - medicion;
            filtrado = alpha * distancia_piso + (1-alpha) * filtrado;
        }

        // 2. Control PID modificado
        float error = SETPOINT - filtrado;
        
        // Término integral con anti-gravedad
        pid.integrator += 0.5f * pid.Ki * SAMPLE_TIME_MS/1000.0f * (error + pid.prevError);
        pid.integrator += ANTI_GRAVITY_BIAS;  // Compensación adicional
        pid.integrator = (pid.integrator > pid.limMax) ? pid.limMax : 
                        (pid.integrator < pid.limMin) ? pid.limMin : pid.integrator;
        
        // Término derivativo (usando prevmedicion)
        pid.differentiator = -(2.0f * pid.Kd * (filtrado - pid.prevmedicion)
                            + (2.0f * pid.tau - SAMPLE_TIME_MS/1000.0f) * pid.differentiator)
                            / (2.0f * pid.tau + SAMPLE_TIME_MS/1000.0f);
        
        // Salida del controlador
        pid.out = (pid.Kp * error) + pid.integrator + pid.differentiator;
        pid.out = (pid.out > pid.limMax) ? pid.limMax : 
                 (pid.out < pid.limMin) ? pid.limMin : pid.out;
        
        // 3. Aplicación del PWM
        uint16_t pwm = (uint16_t)(pid.out * 4999.0f);
        pwm = (pwm < MIN_PWM) ? MIN_PWM : (pwm > MAX_PWM) ? MAX_PWM : pwm;
        pwm_set_level(&fan, pwm);

        // 4. Actualización de estados
        pid.prevError = error;
        pid.prevmedicion = filtrado;  // Actualizando el campo correcto

        // 5. Monitorización
        printf("Alt: %.2fcm | PWM: %4d | Err: %.2f | Int: %.2f | Der: %.2f\n",
              filtrado, pwm, error, pid.integrator, pid.differentiator);
        
        sleep_ms(SAMPLE_TIME_MS);
    }
}