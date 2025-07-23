#ifndef PWM_LIB_H
#define PWM_LIB_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Primero define la estructura antes de usarla en las declaraciones de funciones
typedef struct {
    uint pin;
    uint slice;
    uint channel;
    uint wrap;
    uint clk_div;
} pwm_config_t;

// Luego declara las funciones que usan la estructura
void pwm_init_config(pwm_config_t *config);
void pwm_set_level(pwm_config_t *config, uint16_t level);
uint pwm_get_wrap(pwm_config_t *config);
void pwm_set_enabled_state(pwm_config_t *config, bool enabled);

#endif // PWM_LIB_H
