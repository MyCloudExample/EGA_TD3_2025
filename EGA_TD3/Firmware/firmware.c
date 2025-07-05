#include <stdio.h>
#include "pico/stdlib.h"
#include "lcd.h"
#include "pwm_lib.h"
#include "HC_SR04.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/*-------------------------------------DEFINICION DE PINES PARA EL PROYECTO-------------------------------------------------*/
#define PIN_SDA     8 //Pin 4 de la placa
#define PIN_SCL     9 //Pin 5 de la placa
#define I2C         i2c0 //Puerto del i2c
#define ADDR        0x27 //Direccion del LCD en I2C
#define FREQ        100000 //Frecuencia de 100KHz para el i2c
#define PIN_PWM     16 //Pin 9 de la placa
#define PIN_RPM     17 //Pin 10 de la placa
#define PIN_TRIG    14 //Pin 6 de la placa
#define PIN_ECHO    15 //Pin 7 de la placa
#define PIN_SCK     0 //Pin 14 de la placa
#define PIN_TX      1 //Pin 15 de la placa
#define PIN_RX      2 //Pin 16 de la placa
#define PIN_CS      3 //Pin 17 de la placa 
/*--------------------------------------VARAIBLES DE RPOGRAMA, COLAS Y SEMAFOROS----------------------------------------------*/
pwm_config_t cooler={.pin=PIN_PWM, .wrap=12499, .clk_div=10};
hc_sr04_t sensor;
SemaphoreHandle_t sem1;
QueueHandle_t que1;
/*--------------------------------------TAREAS DE FREERTOS--------------------------------------------------------------------*/
//-----------------------------------------TAREA DE INICIALIZACION-------------------------------------------------------------
void task_init(void *params) 
{
    // Inicializacion de GPIO para HC-SR04
    hc_sr04_init(&sensor,PIN_TRIG,PIN_ECHO);
    //Inicializacion del I2C
    i2c_init(I2C, FREQ);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    //Inicializo el LCD
    lcd_init(I2C,ADDR);
    lcd_clear();
    lcd_string("STARTING....");
    //Inicializo el PWM
    pwm_init_config(&cooler);
    //Inicializo memoria SD

    // Elimino la tarea para liberar recursos
    vTaskDelete(NULL);
}
//-------------------------------------------------TAREA DE SENSANDO DE LA ALTURA------------------------------------------------
void task_hcsr04(void *params)
{ float valor_medido=0.0;

    while (true)
    {
        valor_medido = hc_sr04_get_distance_cm(&sensor);
        if(valor_medido == -1.0f)
        {
            printf("Distancia fuera de rango\n");
        }
        else
        {
            printf("Distancia= %.2f cm\n",valor_medido);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

}

int main(void) 
{
    stdio_init_all();

    // Creacion de tareas
    xTaskCreate(task_init, "Init", configMINIMAL_STACK_SIZE*2, NULL, 2, NULL);

    // Arranca el scheduler
    vTaskStartScheduler();
    while(1);
}