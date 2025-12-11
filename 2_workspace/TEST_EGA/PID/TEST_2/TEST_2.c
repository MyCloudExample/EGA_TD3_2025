#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

// Configuración Hardware
#define PIN_PWM     11    // Control del ventilador
#define PIN_TRIG    14    // Sensor HC-SR04
#define PIN_ECHO    15
#define PIN_POT     26    // Potenciómetro (ADC0)
#define SAMPLE_TIME_MS 30 // Periodo de control

// Estructura PID
typedef struct {
    float Kp, Ki, Kd;
    float tau;
    float limMin, limMax;
    float integrator;
    float prevError;
    float prevMeasurement;
    float out;
} PIDController;

void PID_Init(PIDController *pid) {
    pid->integrator = 0;
    pid->prevError = 0;
    pid->prevMeasurement = 0;
    pid->out = 0;
}

float PID_Update(PIDController *pid, float setpoint, float measurement, float dt) {
    // Error
    float error = setpoint - measurement;
    
    // Proporcional
    float proportional = pid->Kp * error;
    
    // Integral (con anti-windup)
    pid->integrator += 0.5f * pid->Ki * dt * (error + pid->prevError);
    pid->integrator = (pid->integrator > pid->limMax) ? pid->limMax : 
                     (pid->integrator < pid->limMin) ? pid->limMin : pid->integrator;
    
    // Derivativo (con filtro)
    float differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) + 
                          (2.0f * pid->tau - dt) * pid->out) / (2.0f * pid->tau + dt);
    
    // Salida
    pid->out = proportional + pid->integrator + differentiator;
    pid->out = (pid->out > pid->limMax) ? pid->limMax : 
              (pid->out < pid->limMin) ? pid->limMin : pid->out;
    
    // Guardar estados
    pid->prevError = error;
    pid->prevMeasurement = measurement;
    
    return pid->out;
}

int main() {
    stdio_init_all();
    
    // 1. Configuración PWM
    gpio_set_function(PIN_PWM, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PIN_PWM);
    uint channel = pwm_gpio_to_channel(PIN_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 4999);
    pwm_init(slice_num, &config, true);
    
    // 2. Configuración Sensor Ultrasónico
    gpio_init(PIN_TRIG);
    gpio_set_dir(PIN_TRIG, GPIO_OUT);
    gpio_init(PIN_ECHO);
    gpio_set_dir(PIN_ECHO, GPIO_IN);
    
    // 3. Configuración ADC para potenciómetro
    adc_init();
    adc_gpio_init(PIN_POT);
    adc_select_input(0);
    
    // 4. Inicialización PID
    PIDController pid = {
        .Kp = 0.8f, .Ki = 0.2f, .Kd = 0.05f,
        .tau = 0.1f,
        .limMin = 0.0f, .limMax = 1.0f
    };
    PID_Init(&pid);
    
    // Variables de control
    float setpoint = 15.0f;  // Altura deseada (cm)
    float distance = 0;
    float filtered_distance = setpoint;
    const float alpha = 0.3f; // Factor de filtro
    
    while(1) {
        // A. Leer potenciómetro (opcional para ajustar setpoint)
        float pot_value = adc_read() / 4095.0f;
        //setpoint = 10.0f + pot_value * 20.0f; // Rango 10-30cm
        
        // B. Medición de distancia
        gpio_put(PIN_TRIG, 1);
        sleep_us(10);
        gpio_put(PIN_TRIG, 0);
        
        uint32_t timeout = time_us_32() + 30000;
        while(!gpio_get(PIN_ECHO) && time_us_32() < timeout);
        uint32_t start = time_us_32();
        
        timeout = time_us_32() + 30000;
        while(gpio_get(PIN_ECHO) && time_us_32() < timeout);
        float raw_dist = (time_us_32() - start) * 0.0343 / 2.0f;
        
        // C. Filtrado y validación
        if(raw_dist > 2.0f && raw_dist < 50.0f) {
            filtered_distance = alpha * raw_dist + (1-alpha) * filtered_distance;
        }
        
        // D. Cálculo PID
        float control = PID_Update(&pid, setpoint, filtered_distance, SAMPLE_TIME_MS/1000.0f);
        uint16_t pwm_value = (uint16_t)(control * 4999.0f);
        pwm_set_chan_level(slice_num, channel, pwm_value);
        
        // E. Monitorización
        printf("Set: %.1fcm | Dist: %.1fcm | PWM: %d | Kp: %.1f\n",
              setpoint, filtered_distance, pwm_value, pid.Kp);
        
        sleep_ms(SAMPLE_TIME_MS);
    }
}