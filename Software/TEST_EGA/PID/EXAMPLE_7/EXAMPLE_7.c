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
#define KP 10.0f             // Más agresivo, 8
#define KI 1.2f             // Integral aumentado
#define KD 3.0f             // Más amortiguamiento
#define MIN_PWM 1800        // Mínimo absoluto
#define MAX_PWM 4000        // Máximo seguro
#define DT 0.01f            //Aumenta el timepo de muestro a 
#define MAX_PWM_STEP 200    //Maxino cambio entre ciclos

int main() {
    stdio_init_all();
    
    pwm_config_t fan = {
        .pin = PIN_PWM,
        .wrap = 4999,
        .clk_div = 1.0f
    };
    pwm_init_config(&fan);

    //Verificación sensor
    hc_sr04_t sensor;
    hc_sr04_init(&sensor, PIN_TRIG, PIN_ECHO);
    //Sistema completo
    PIDController pid = {
        .Kp = KP, .Ki = KI, .Kd = KD,
        .tau = 0.02f,
        .limMin = MIN_PWM,
        .limMax = MAX_PWM,
        .integrator = 0.0f,
        .prevError = 0,
        .differentiator = 0,
        .prevmedicion = TARGET_HEIGHT,
        .out = BASE_PWM
    };
    PIDController_Init(&pid);

    float filtered_height = TARGET_HEIGHT;
    const float alpha = 0.5f;

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
        //Control PID
         float control = PIDController_Update(&pid, TARGET_HEIGHT, filtered_height, DT);
        uint16_t pwm = (uint16_t)(control);
        static uint16_t last_pwm = BASE_PWM;
        if(abs(pwm - last_pwm) > MAX_PWM_STEP)
            {
                pwm = (pwm > last_pwm) ? last_pwm + MAX_PWM_STEP : last_pwm - MAX_PWM_STEP;
            }
        pwm = (pwm < MIN_PWM) ? MIN_PWM : (pwm > MAX_PWM) ? MAX_PWM : pwm;
        pwm_set_level(&fan, pwm);
        last_pwm=pwm;

        printf("Alt:%.2f,PWM:%4d,Err:%.2f,MIN:0.0,MAX:45.0\n", 
        filtered_height, pwm, TARGET_HEIGHT - filtered_height);
        //printf("Alt:%.2f,MIN:0.0,MAX:45.0\n", filtered_height);
        sleep_ms((int)(DT*1000));
    }
}