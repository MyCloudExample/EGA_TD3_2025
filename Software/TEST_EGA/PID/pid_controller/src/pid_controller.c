#include "pid_controller.h"

void PIDController_Init(PIDController *pid)
{
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevmedicion = 0.0f;
    pid->out = 0.0f;
}
/*---------------------------------------CALCULO DEL PID---------------------------------------------------------------*/
float PIDController_Update(PIDController *pid, float setpoint, float medicion, float dt)
{
    // Cálculo del error
    float error = setpoint - medicion;
    // Término proporcional
    float proportional = pid->Kp * error;
    // Término integral
    pid->integrator += 0.5f * pid->Ki * dt * (error + pid->prevError);
    // Anti-windup: limitamos el término integral
    if (pid->integrator > pid->limMax)
    {
        pid->integrator = pid->limMax;
    } else if (pid->integrator < pid->limMin)
    {
        pid->integrator = pid->limMin;
    }

    // Término derivativo (con filtro de paso bajo)
    pid->differentiator = -(2.0f * pid->Kd * (medicion - pid->prevmedicion)
                          + (2.0f * pid->tau - dt) * pid->differentiator)
                          / (2.0f * pid->tau + dt);

    // Cálculo de la salida
    pid->out = proportional + pid->integrator + pid->differentiator;

    // Limitamos la salida
    if (pid->out > pid->limMax)
    {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin)
    {
        pid->out = pid->limMin;
    }

    // Almacenamos valores para la próxima iteración
    pid->prevError = error;
    pid->prevmedicion = medicion;

    return pid->out;
}
