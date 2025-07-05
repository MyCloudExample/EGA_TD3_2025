#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "lcd.h"
#include "pwm_lib.h"
#include "HC_SR04.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/*-------------------------------------DEFINICION DE PINES PARA EL PROYECTO-------------------------------------------------*/
#define PIN_SDA     8 //Pin 11 de la placa
#define PIN_SCL     9 //Pin 12 de la placa
#define I2C         i2c0 //Puerto del i2c
#define ADDR        0x27 //Direccion del LCD en I2C
#define FREQ        400000 //Frecuencia de 100KHz para el i2c
#define PIN_PWM     16 //Pin 21 de la placa
#define PIN_RPM     17 //Pin 22 de la placa
#define PIN_TRIG    15 //Pin 19 de la placa
#define PIN_ECHO    16 //Pin 20 de la placa
#define PIN_SCK     0 //Pin 1 de la placa
#define PIN_TX      1 //Pin 2 de la placa
#define PIN_RX      2 //Pin 4 de la placa
#define PIN_CS      3 //Pin 5 de la placa 
#define PIN_ADC     26 //Pin 
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
    //Inicializo el ADC
    adc_init();
    adc_gpio_init(PIN_ADC);
    adc_select_input(0);
    //Inicializo el LCD
    lcd_init(I2C,ADDR);
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_string("STARTING....");
    //Inicializo el PWM
    pwm_init_config(&cooler);
    //Inicializo memoria SD
    printf("Tarear elimianda\n");
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
//---------------------------------------------------TAREA PARA INICAR EL SETPOINT------------------------------------------------
void task_SetPoint(void *params)
{ uint32_t valor_adc, altura_int;
  float altura, tension;
  char buffer[30];
  
  lcd_clear();
    while (true)
    {
        valor_adc = adc_read();
        tension = (valor_adc * 3.3f) / 4095; 
        altura = ((valor_adc * 3.3f) / 4095)*10;
        altura_int = altura;
        vTaskDelay(pdMS_TO_TICKS(200));
        lcd_set_cursor(0,0);
        sprintf(buffer,"Altura= %lu cm ",altura_int);
        lcd_string(buffer);
        printf("Valor= %lu \n",valor_adc);
        printf("Tension= %.2f V\n",tension);
        printf("Altura= %.2f cm\n", altura);
    }    
}

int main(void) 
{
    stdio_init_all();

    // Creacion de tareas
    xTaskCreate(task_init, "Init", 256, NULL, 3, NULL);
    //xTaskCreate(task_SetPoint,"SetPoint",256,NULL,2,NULL);
    xTaskCreate(task_hcsr04,"MedicionDeDistancia",256,NULL,2,NULL);

    // Arranca el scheduler
    vTaskStartScheduler();
    while(1);
}