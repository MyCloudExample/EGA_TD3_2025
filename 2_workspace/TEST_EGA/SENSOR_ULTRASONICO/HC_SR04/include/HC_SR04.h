#ifndef HC_SR04_H
#define HC_SR04_H

#include "pico/stdlib.h"

typedef struct {
    uint trig_pin;
    uint echo_pin;
} hc_sr04_t;

void hc_sr04_init(hc_sr04_t *sensor, uint trig_pin, uint echo_pin);
float hc_sr04_get_distance_cm(hc_sr04_t *sensor);

#endif // HC_SR04_NO_IRQ_H
