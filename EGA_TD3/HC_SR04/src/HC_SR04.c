#include "HC_SR04.h"
#include <stdio.h>

#define TIMEOUT_US 30000
#define SOUND_SPEED_CM_US 0.0343
#define TRIG_PULSE_US 10

void hc_sr04_init(hc_sr04_t *sensor, uint trig_pin, uint echo_pin) {
    sensor->trig_pin = trig_pin;
    sensor->echo_pin = echo_pin;

    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);

    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
}

float hc_sr04_get_distance_cm(hc_sr04_t *sensor) {
    // Enviar pulso de trigger
    gpio_put(sensor->trig_pin, 1);
    sleep_us(TRIG_PULSE_US);
    gpio_put(sensor->trig_pin, 0);

    // Esperar flanco de subida (eco comienza)
    uint32_t timeout = time_us_32() + TIMEOUT_US;
    while (!gpio_get(sensor->echo_pin) && time_us_32() < timeout);

    if (!gpio_get(sensor->echo_pin)) return -1.0f; // Timeout

    uint32_t start = time_us_32();

    // Esperar flanco de bajada (eco termina)
    timeout = time_us_32() + TIMEOUT_US;
    while (gpio_get(sensor->echo_pin) && time_us_32() < timeout);

    if (gpio_get(sensor->echo_pin)) return -1.0f; // Timeout

    uint32_t duration = time_us_32() - start;
    float distance = SOUND_SPEED_CM_US * duration / 2.0f;

    // Validar rango razonable
    return (distance >= 2.0f && distance <= 400.0f) ? distance : -1.0f;
}
