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

//includes del modulo SD
#include "hardware/spi.h"
#include "pff.h"
#include "diskio.h"
#include "string.h"

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
#define PIN_ADC     26 //Pin
// Pines SPI para la Raspberry Pi Pico 2
#define PIN_MISO  00
#define PIN_CS    01
#define PIN_SCK   02
#define PIN_MOSI  03
// Alertas
#define GPIO_LED_MAX 16
#define GPIO_LED_MIN 17
#define ALERTA_TIMEOUT_MS 3000
/*--------------------------------------VARAIBLES DE RPOGRAMA, COLAS Y SEMAFOROS----------------------------------------------*/
pwm_config_t cooler={.pin=PIN_PWM, .wrap=12499, .clk_div=10};
hc_sr04_t sensor;
SemaphoreHandle_t sem_mutex;
QueueHandle_t queue_rtc;
QueueHandle_t queue_hcsr04;
QueueHandle_t queue_setpoint;
QueueHandle_t queue_pwm;
QueueHandle_t queue_altura;
QueueHandle_t queue_max;
QueueHandle_t queue_min;
QueueHandle_t queue_max_salida;
QueueHandle_t queue_min_salida;
/*--------------------------------------TAREAS DE FREERTOS--------------------------------------------------------------------*/
//-----------------------------------------TAREA DE INICIALIZACION-------------------------------------------------------------

void init_spi_sd(void) {
    spi_init(spi0, 1000 * 1000); // 1 MHz

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SIO); // CS como GPIO normal
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); // Desactivar CS inicialmente
    
}

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
    //Inicializo el modulo SD
    printf("Inicializando SPI para SD...\n");
    init_spi_sd();
    //Inicializo el PWM
    pwm_init_config(&cooler);
    //Inicializo memoria SD
    printf("Tarea elimianda\n");
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
//---------------------------------------------------TAREA GUARDIANA LCD----------------------------------------------------------

void task_guardiana_lcd(void *params) {
    uint16_t val_altura = 0, val_max = 0, val_min = 0, val_hcsr04 = 0, val_rtc = 0, val_max_salida = 0, val_min_salida = 0;
    bool alerta_latched = false;
    TickType_t tick_ultima_alerta = 0;

    while (1) {
        xQueueReceive(queue_altura, &val_altura, pdMS_TO_TICKS(100));
        xQueueReceive(queue_max, &val_max, pdMS_TO_TICKS(100));
        xQueueReceive(queue_min, &val_min, pdMS_TO_TICKS(100));
        xQueueReceive(queue_hcsr04, &val_hcsr04, pdMS_TO_TICKS(100));
        xQueueReceive(queue_max_salida, &val_max_salida, pdMS_TO_TICKS(100));
        xQueueReceive(queue_min_salida, &val_min_salida, pdMS_TO_TICKS(100));

        // Mostrar datos
        printf("ALTURA: %.2f cm | PICO MAXIMO: %.2f cm | PICO MINIMO: %.2f cm | RAW HCSR04: %.2f | PICO MAXIMO EN SALIDA: %.2f | PICO MINIMO EN SALIDA: %.2f;", val_altura, val_max, val_min, val_hcsr04, val_max_salida, val_min_salida);

        // Limpio el LCD
        lcd_clear();
        // Muevo el cursor a la segunda fila, tercer columna
        lcd_set_cursor(0, 0);
        // Escribo
        char buffer[20];
        sprintf(buffer, "SETPOINT: %.2f cm", val_altura);
        lcd_string(buffer);
        // Muevo el cursor a la segunda fila, tercer columna
        lcd_set_cursor(1, 0);
        // Escribo
        sprintf(buffer, "MAX: %.2f cm", val_max);
        lcd_string(buffer);
        // Muevo el cursor a la segunda fila, tercer columna
        lcd_set_cursor(2, 0);
        // Escribo
        sprintf(buffer, "HCSR04: %.2f cm", val_hcsr04);
        lcd_string(buffer);
        

        // Si superó el umbral → activa latch y guarda tiempo
        if (val_max_salida > val_max || val_min_salida < val_min) {
            alerta_latched = true;
            tick_ultima_alerta = xTaskGetTickCount();
        }

        // Si está activo y ya pasó el timeout → apagar
        if (alerta_latched) {
            TickType_t ahora = xTaskGetTickCount();
            if ((ahora - tick_ultima_alerta) > pdMS_TO_TICKS(ALERTA_TIMEOUT_MS)) {
                alerta_latched = false;
            }
        }

        // Control de LED
        gpio_put(GPIO_LED_MAX, alerta_latched);
        gpio_put(GPIO_LED_MIN, alerta_latched);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//---------------------------------------------------TAREA GUARDIANA DE MODULO SD-------------------------------------------------
void task_guardiana_sd(void *params) {

    //uint16_t val_altura = 0, val_max = 0, val_min = 0, val_hcsr04 = 0, val_rtc = 0, val_max_salida = 0, val_min_salida = 0;

    vTaskDelay(pdMS_TO_TICKS(5000));

    FATFS fs;
    FRESULT fr;
    UINT escritos;

    while(1) {
        //xQueueReceive(queue_hcsr04, &val_hcsr04, pdMS_TO_TICKS(100));
        //xQueueReceive(queue_altura, &val_altura, pdMS_TO_TICKS(100));
        //xQueueReceive(queue_rtc, &val_rtc, pdMS_TO_TICKS(100));
        //xQueueReceive(queue_setpoint, &val_setpoint, pdMS_TO_TICKS(100));

        // Montar el sistema de archivos
        fr = pf_mount(&fs);
        if (fr != FR_OK) {
            printf("Error al montar: %d\n", fr);
            return;
        }

        // Abrir archivo existente o crear uno nuevo (solo lectura/escritura secuencial)
        fr = pf_open("log.txt");
        if (fr != FR_OK) {
            printf("Error al abrir: %d\n", fr);
            return;
        }        

        const char* mensaje = "Hola Mundo\r\n";

        // Posicionarse al final del archivo (simulación de append)
        fr = pf_lseek(fs.fsize);  // Podés usar pf_lseek(tamaño_anterior) si querés continuar desde el final
        if (fr != FR_OK) {
            printf("Error en lseek: %d\n", fr);
            return;
        }

        printf("valor de lseek: %lu - ", fs.fsize);
        printf("valor de mensaje: %s - ", mensaje);

        // Escribir datos
        fr = pf_write(mensaje, strlen(mensaje), &escritos);
        if (fr != FR_OK) {
            printf("Error escribiendo: %d\n", fr);
            return;
        }

        // Finalizar escritura (con 0 bytes)
        fr = pf_write(0, 0, &escritos);  // Necesario para finalizar buffer en Petit FatFs
        if (fr != FR_OK) {
            printf("Error finalizando escritura: %d\n", fr);
        }

        printf("Escritura completa: %u bytes\n", escritos);

        vTaskDelay(pdMS_TO_TICKS(1000)); 
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
    xTaskCreate(task_SetPoint,"SetPoint",256,NULL,2,NULL);
    xTaskCreate(task_hcsr04,"MedicionDeDistancia",256,NULL,2,NULL);
    xTaskCreate(task_guardiana_sd,"guardianaSD",256,NULL,2,NULL);
    xTaskCreate(task_guardiana_lcd,"guardianaLCD",256,NULL,2,NULL);

    // Arranca el scheduler
    vTaskStartScheduler();
    while(1);
}