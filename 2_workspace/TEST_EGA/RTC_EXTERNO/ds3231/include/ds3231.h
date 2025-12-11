#ifndef DS3231_H
#define DS3231_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define DS3231_ADDR 0x68 // Dirección I2C del DS3231

// Estructura para almacenar fecha y hora
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} ds3231_time_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el DS3231
 * @param i2c Instancia de I2C (i2c0 o i2c1)
 * @param sda_pin Pin SDA
 * @param scl_pin Pin SCL
 * @param baudrate Velocidad de I2C (por defecto 100000)
 * @return true si la inicialización fue exitosa, false en caso contrario
 */
bool ds3231_init(i2c_inst_t *i2c, uint sda_pin, uint scl_pin, uint baudrate);

/**
 * @brief Establece la fecha y hora en el DS3231
 * @param i2c Instancia de I2C
 * @param time Estructura con la fecha y hora a establecer
 * @return true si la operación fue exitosa, false en caso contrario
 */
bool ds3231_set_time(i2c_inst_t *i2c, const ds3231_time_t *time);

/**
 * @brief Obtiene la fecha y hora actual del DS3231
 * @param i2c Instancia de I2C
 * @param time Estructura donde se almacenará la fecha y hora
 * @return true si la operación fue exitosa, false en caso contrario
 */
bool ds3231_get_time(i2c_inst_t *i2c, ds3231_time_t *time);

/**
 * @brief Convierte un valor BCD a decimal
 * @param bcd Valor en formato BCD
 * @return Valor decimal
 */
uint8_t bcd_to_dec(uint8_t bcd);

/**
 * @brief Convierte un valor decimal a BCD
 * @param dec Valor decimal
 * @return Valor en formato BCD
 */
uint8_t dec_to_bcd(uint8_t dec);

#ifdef __cplusplus
}
#endif

#endif // DS3231_H
