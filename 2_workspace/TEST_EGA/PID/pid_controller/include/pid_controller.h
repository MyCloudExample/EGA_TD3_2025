#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct
{
    float Kp;       // Ganancia proporcional
    float Ki;       // Ganancia integral
    float Kd;       // Ganancia derivativa
    float tau;      // Factor de filtrado para el término derivativo

    float limMin;   // Límite mínimo de salida
    float limMax;   // Límite máximo de salida

    float integrator;   // Acumulador del término integral
    float prevError;    // Error anterior (para el término derivativo)
    float differentiator; // Valor filtrado del término derivativo
    float prevmedicion; // Medición anterior (para el término derivativo)

    float out;      // Salida del controlador
} PIDController;

void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement, float dt);


#endif // PID_CONTROLLER_H
