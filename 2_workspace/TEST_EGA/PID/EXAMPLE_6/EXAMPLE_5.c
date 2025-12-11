//En esta quinta version se mejoro el Kp, Ki, Kd ajustandolo segune el diametro y peso de la pelota
// diametro=7.8 cm ; peso=9 gr. Ademas se utiliza la libreria pid_controller. Los ajustes ques e lograron son muy buenos
#include <stdio.h>
#include <stdlib.h>
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
#define TARGET_HEIGHT 20.0f
#define SENSOR_HEIGHT 45.0f

// Parámetros iniciales (ajustar durante prueba)
#define BASE_PWM 3000       // Incrementado inicialmente
#define KP 5.0f             // Más agresivo
#define KI 1.2f             // Integral aumentado
#define KD 2.0f             // Más amortiguamiento
#define MIN_PWM 1800        // Mínimo absoluto
#define MAX_PWM 4000        // Máximo seguro
#define DT 0.04f            //Aumenta el timepo de muestro a 30ms

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
    /*printf("Prueba manual del ventilador:\n");
    for(int pwm = 3500; pwm <= MAX_PWM; pwm += 500) {
        pwm_set_level(&fan, pwm);
        printf("PWM: %d, ¿Está soplando? (espera 3 seg)\n", pwm);
        sleep_ms(3000);
    }*/

    //Verificación sensor
    hc_sr04_t sensor;
    hc_sr04_init(&sensor, PIN_TRIG, PIN_ECHO);
    /*printf("\nPrueba del sensor:\n");
    for(int i = 0; i < 5; i++) {
        float dist = hc_sr04_get_distance_cm(&sensor);
        printf("Medición %d: %.2f cm\n", i+1, dist);
        sleep_ms(200);
    }*/

    //Sistema completo
    PIDController pid = {
        .Kp = KP, .Ki = KI, .Kd = KD,
        .tau = 0.02f,
        .limMin = MIN_PWM,
        .limMax = MAX_PWM,
        .integrator = (MAX_PWM + MIN_PWM)/2.0f,
        .prevError = 0,
        .differentiator = 0,
        .prevmedicion = TARGET_HEIGHT,
        .out = MAX_PWM * 0.75f
    };
    PIDController_Init(&pid);

    float filtered_height = TARGET_HEIGHT;
    const float alpha = 0.25f;
    static uint16_t last_pwm = BASE_PWM;

    /*pwm_set_level(&fan,3500);
    sleep_ms(3000);
    printf("Fin de prueba de elevacion con 3500\n");*/

    while(true) {
        // Medición robusta
        float raw_dist = hc_sr04_get_distance_cm(&sensor);
        
        if(raw_dist <= 2.0f && raw_dist >= SENSOR_HEIGHT -2.0f) 
        {
            printf("Medicion invalida:%.2f cm\n",raw_dist);
            pwm_set_level(&fan,MAX_PWM*0.8f);
            continue;
        } 
        float current_height = SENSOR_HEIGHT - raw_dist;
        filtered_height = alpha * current_height + (1-alpha) * filtered_height;
        // Zona muerta para pequeños errores
        uint16_t pwm;
        if(fabs(TARGET_HEIGHT - filtered_height) < 0.5f) 
        {  // 0.5 cm de margen
            pwm = last_pwm;  // Mantiene el último valor de PWM
        } 
        else 
        {
            float control = PIDController_Update(&pid, TARGET_HEIGHT, filtered_height, DT);
            pwm = (uint16_t)(control);
            if(abs(pwm - last_pwm > 100))
                {
                    pwm = (pwm + last_pwm*2)/3; //Promedio ponderado
                }
        }
        
        pwm = (pwm < MIN_PWM) ? MIN_PWM : (pwm > MAX_PWM) ? MAX_PWM : pwm;
        pwm_set_level(&fan, pwm);
        last_pwm=pwm;

        printf("Alt: %.2fcm | PWM: %4d | Err: %.2f | P: %.1f I: %.1f D: %.1f\n",
              filtered_height, pwm, TARGET_HEIGHT - filtered_height,
              pid.Kp * (TARGET_HEIGHT - filtered_height),
              pid.integrator,
              pid.differentiator);
        //printf("Altura:%.2f,PWM:%d,Error:%.2f\n", filtered_height, pwm, TARGET_HEIGHT - filtered_height);
        //float altura_plotter = ((filtered_height - 5.0f) / (45.0f - 5.0f)) * 100.0f;
        //printf("Altura:%.2f,Min:0.00,Max:100.00\n", altura_plotter);
        sleep_ms((int)(DT*1000));
    }
}