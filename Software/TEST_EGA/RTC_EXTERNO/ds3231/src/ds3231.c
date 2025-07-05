#include "ds3231.h"
#include <string.h>

bool ds3231_init(i2c_inst_t *i2c, uint sda_pin, uint scl_pin, uint baudrate) {
    // Inicializar I2C
    i2c_init(i2c, baudrate);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Verificar comunicación con el dispositivo
    uint8_t rxdata;
    int ret = i2c_read_blocking(i2c, DS3231_ADDR, &rxdata, 1, false);
    return ret >= 0;
}

bool ds3231_set_time(i2c_inst_t *i2c, const ds3231_time_t *time) {
    uint8_t data[8];

    // El primer byte es la dirección de registro (0x00 para empezar en segundos)
    data[0] = 0x00;

    // Convertir los valores a BCD y almacenarlos
    data[1] = dec_to_bcd(time->seconds);
    data[2] = dec_to_bcd(time->minutes);
    data[3] = dec_to_bcd(time->hours);
    data[4] = dec_to_bcd(time->day);
    data[5] = dec_to_bcd(time->date);
    data[6] = dec_to_bcd(time->month);
    data[7] = dec_to_bcd(time->year);

    // Escribir los datos en el RTC
    int ret = i2c_write_blocking(i2c, DS3231_ADDR, data, 8, false);
    return ret == 8;
}

bool ds3231_get_time(i2c_inst_t *i2c, ds3231_time_t *time) {
    uint8_t reg = 0x00; // Empezar a leer desde el registro de segundos
    uint8_t data[7];

    // Escribir el registro que queremos leer
    int ret = i2c_write_blocking(i2c, DS3231_ADDR, &reg, 1, true);
    if (ret != 1) return false;

    // Leer los 7 registros de tiempo
    ret = i2c_read_blocking(i2c, DS3231_ADDR, data, 7, false);
    if (ret != 7) return false;

    // Convertir de BCD a decimal y almacenar en la estructura
    time->seconds = bcd_to_dec(data[0] & 0x7F); // Ignorar el bit CH
    time->minutes = bcd_to_dec(data[1]);
    time->hours = bcd_to_dec(data[2] & 0x3F); // Ignorar formato 12/24 horas
    time->day = bcd_to_dec(data[3]);
    time->date = bcd_to_dec(data[4]);
    time->month = bcd_to_dec(data[5] & 0x1F); // Ignorar el bit Century
    time->year = bcd_to_dec(data[6]);

    return true;
}

uint8_t bcd_to_dec(uint8_t bcd) {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}
