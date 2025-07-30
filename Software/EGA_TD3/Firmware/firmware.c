#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "lcd.h"
#include "pwm_lib.h"
#include "HC_SR04.h"
#include "ds3231.h"
#include "hardware/uart.h"
//Caebceras de FreeRTOS
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
// Pines de HC-SR04
#define PIN_TRIG    14 //Pin 19 de la placa
#define PIN_ECHO    15 //Pin 20 de la placa
// PIN de potenciometro de setpoint
#define PIN_ADC     26 //Pin
// Pines UART1 
#define PIN_TX  4
#define PIN_RX  5
#define UART_ID uart1
#define UART_BAUDRATE 115200
// Alertas
#define GPIO_LED_MAX 16
#define GPIO_LED_MIN 17
#define ALERTA_TIMEOUT_MS 3000
// Boton de cambio de pagina, conmuta en la opciones de task_setpoint
#define PIN_PAGINA  18 // Pin 24 de la placa
#define DEBOUNCE_TIME_MS 50
#define MULTI_PRESS_TIMEOUT 300
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
QueueHandle_t cola_paginas;
typedef struct
{
    uint32_t setpoint;
    float setpoint_min;
    float setpoint_max;
}estructura_setpoint;

/*--------------------------------------TAREAS DE FREERTOS--------------------------------------------------------------------*/
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
    //COnfiguro pines de los leds banderas
    gpio_init(GPIO_LED_MAX); //Inicio el pin 16
    gpio_set_dir(GPIO_LED_MAX, GPIO_OUT); //Se configura como salida
    gpio_put(GPIO_LED_MAX, 0); // Se coloca un 0 a la salida
    gpio_init(GPIO_LED_MIN); //Inicio el pin 17
    gpio_set_dir(GPIO_LED_MIN, GPIO_OUT); //Se configura como salida
    gpio_put(GPIO_LED_MIN, 0); // Se coloca un 0 a la salida
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
            printf("Tarea: task_hcssr04, Distancia= %.2f cm\n",valor_medido);
            xQueueSend(queue_hcsr04,&valor_medido,pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
//---------------------------------------------------TAREA GUARDIANA LCD----------------------------------------------------------
void task_guardiana_lcd(void *pvParameter) 
{
    float val_hcsr04=0.0f;
    estructura_setpoint recepcion_lcd;
    char buffer[30];

    while (true) 
    {
        xQueueReceive(queue_setpoint, &recepcion_lcd, pdMS_TO_TICKS(100));
        xQueueReceive(queue_hcsr04, &val_hcsr04, pdMS_TO_TICKS(100));

        // Para pruebas de testeo
        //printf("Tarea: task_guradiana_lcd, Altura: %.2f cm\n",val_hcsr04); //Datos del ultrasonico
        // Limpio el LCD
        lcd_clear();
        // Muevo el cursor a la fila 0, columna 0
        lcd_set_cursor(0, 0);
        sprintf(buffer, "T:%lucm ", recepcion_lcd.setpoint);
        lcd_string(buffer);
        // Muevo el cursor a la fila 1, columna 0
        lcd_set_cursor(1, 0);
        sprintf(buffer, "M:%.2fcm | m:%.2f", recepcion_lcd.setpoint_max, recepcion_lcd.setpoint_min);
        lcd_string(buffer);
        // Muevo el cursor a la fila 2, columna 0, deto desde el sensor HC-SR04
        lcd_set_cursor(2, 0);
        sprintf(buffer, "HCSR04: %.2f cm", val_hcsr04);
        lcd_string(buffer);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//---------------------------------------------------TAREA GUARDIANA DE MODULO SD-------------------------------------------------
void task_guardiana_sd(void *params) 
{ estructura_setpoint datasd;
  ds3231_time_t rtcsd;
  char buffer[30];

    uart_init(UART_ID, UART_BAUDRATE);
    gpio_set_function(PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RX, GPIO_FUNC_UART);
    
    while(true) 
    {
        xQueueReceive(queue_setpoint, &datasd, pdMS_TO_TICKS(100));
        sprintf(buffer,"Altura seteada: %lu\n ",datasd.setpoint);
        uart_puts(UART_ID, buffer);
        vTaskDelay(pdMS_TO_TICKS(200));
        sprintf(buffer,"Altura maxima: %.2f\n ",datasd.setpoint_max);
        uart_puts(UART_ID, buffer);
        vTaskDelay(pdMS_TO_TICKS(200));
        sprintf(buffer,"Altura minima: %.2f\n ",datasd.setpoint_min);
        uart_puts(UART_ID, buffer);
        vTaskDelay(pdMS_TO_TICKS(200));

        xQueueReceive(queue_rtc,&rtcsd,pdMS_TO_TICKS(100));
        sprintf(buffer,"date: %d/%d/%d\n ",rtcsd.day,rtcsd.month,rtcsd.year);
        uart_puts(UART_ID, buffer);
        vTaskDelay(pdMS_TO_TICKS(100));
        sprintf(buffer,"hour: %2d:%2d:%2d\n ",rtcsd.hours,rtcsd.minutes,rtcsd.seconds);
        uart_puts(UART_ID, buffer);
    }

}
//---------------------------------------------------TAREA GUARDIANA DE LEDS------------------------------------------------------
void task_guardiana_leds(void *params) {
    bool alerta_latched = false;
    estructura_setpoint recibido_setpoint;
    float recibido_hcsr04 = 0, val_max_setpoint = 0, val_min_setpoint = 0;
    TickType_t tick_ultima_alerta = 0;
    gpio_init(GPIO_LED_MAX); //Inicio el pin 16
    gpio_set_dir(GPIO_LED_MAX, GPIO_OUT); //Se configura como salida
    gpio_put(GPIO_LED_MAX, 0); // Se coloca un 0 a la salida
    gpio_init(GPIO_LED_MIN); //Inicio el pin 17
    gpio_set_dir(GPIO_LED_MIN, GPIO_OUT); //Se configura como salida
    gpio_put(GPIO_LED_MIN, 0); // Se coloca un 0 a la salida

    while(true)
    {
        if (xQueueReceive(queue_setpoint, &recibido_setpoint, portMAX_DELAY) == pdPASS) 
        {
            val_max_setpoint = recibido_setpoint.setpoint_max;
            val_min_setpoint = recibido_setpoint.setpoint_min;
        }

        if (xQueueReceive(queue_hcsr04, &recibido_hcsr04, portMAX_DELAY) == pdPASS) 
        {
            // Si superó el umbral → activa latch y guarda tiempo
            if (recibido_hcsr04 > val_max_setpoint || recibido_hcsr04 < val_min_setpoint) 
            {
                alerta_latched = true;
                tick_ultima_alerta = xTaskGetTickCount();
                printf("task_guardiana_leds | Supero un limite");
            }
        }

        // Si está activo y ya pasó el timeout → apagar
        if (alerta_latched) 
        {
            TickType_t ahora = xTaskGetTickCount();
            if ((ahora - tick_ultima_alerta) > pdMS_TO_TICKS(ALERTA_TIMEOUT_MS)) 
            {
                alerta_latched = false;
            }
        }

        // Control de LED
        gpio_put(GPIO_LED_MAX, 0);
        gpio_put(GPIO_LED_MIN, 0);
        printf("task_guardiana_leds | SetPoinrMax: %.2f | SetPointMin: %.2f | Medicion: %.2f\n", val_max_setpoint,val_min_setpoint,recibido_hcsr04);
        vTaskDelay(pdMS_TO_TICKS(500));

    }

} 
//---------------------------------------------------TAREA PARA INICAR EL SETPOINT------------------------------------------------
void task_SetPoint(void *params)
{ uint32_t valor_adc, valor_altura;
  estructura_setpoint data={.setpoint=0, .setpoint_max=0, .setpoint_min=0};  
  float tension;
  char buffer[30];
  uint8_t pagina=0;

    while (true)
    { 
       if (xQueuePeek(cola_paginas, &pagina, portMAX_DELAY) == pdPASS) 
        {}
       if (pagina == 1) 
       {
        valor_adc = adc_read();
        tension = (valor_adc * 3.3f) / 4095; 
        valor_altura = ((valor_adc * 3.3f) / 4095)*10;
        data.setpoint = valor_altura;
        printf("PAGINA 1 |setpoint= %lu | Valor altura= %lu \n", data.setpoint, valor_altura);
        } 
        if(pagina==2) 
        {
            valor_adc = adc_read();
            tension = (valor_adc * 3.3f) / 4095; 
            valor_altura = ((valor_adc * 3.3f) / 4095)*10;
            data.setpoint_max = valor_altura;
            printf("PAGINA 2 |setpointMax= %.2f | Valor altura= %lu \n", data.setpoint_max, valor_altura);
        }
        if(pagina==3) 
        {
            valor_adc = adc_read();
            tension = (valor_adc * 3.3f) / 4095; 
            valor_altura = ((valor_adc * 3.3f) / 4095)*10;
            data.setpoint_min = valor_altura;
            printf("PAGINA 3 |setpointMin= %.2f | Valor altura= %lu \n", data.setpoint_min, valor_altura);
        }
        if(pagina==0) 
        {
            
            printf("PAGINA 0 |setpoint= %lu | setpoint_max=%.2f | setpoint_min= %.2f \n", data.setpoint,data.setpoint_max,data.setpoint_min);
        }
       xQueueSend(queue_setpoint, &data, 100); //Se quedara aqui ya que no se desopucpa la cola
       vTaskDelay(pdMS_TO_TICKS(100));
}
           
}

void task_monitor_gpio(void *pvParameters) {
    while (1) {
        printf("Estado GPIO24: %d \n", gpio_get(PIN_PAGINA));
        vTaskDelay(pdMS_TO_TICKS(100));
       
    }
}

void configuracion_gpio_boton(void) {
    // Configuración del botón en GPIO 14
    gpio_init(PIN_PAGINA);
    gpio_set_dir(PIN_PAGINA, GPIO_IN);
    gpio_pull_up(PIN_PAGINA);   // configura pull down
    //gpio_set_irq_enabled_with_callback(PIN_PAGINA, GPIO_IRQ_EDGE_RISE, true, &boton_callback);   // cuando detecta evento, \
                                                                                            evento: GPIO_IRQ_EDGE_RISE (flanco ascendente), \
                                                                                            TRUE: habilita la interrupcion para este GPIO, \
                                                                                            va a la dirección de memoria en la que la función “boton_callback” se encuentra
}

void task_debounce_boton(void *pvParameters) {
    bool last_state = 1;
    bool stable_state = 0;
    uint8_t contador=0;
    TickType_t last_debounce_time = 0;
    const TickType_t debounce_delay = pdMS_TO_TICKS(50);  // 50 ms debounce

    
    printf("Antede de la cola Contador= %d\n",contador);
    xQueueOverwrite(cola_paginas, &contador);
    printf("Despues de la cola Contador= %d\n",contador);
    while (1) 
    {
        bool current_state = gpio_get(PIN_PAGINA);
        if (current_state != last_state) 
        {
            last_debounce_time = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - last_debounce_time) > debounce_delay) 
        {
           
            if (current_state != stable_state) 
            {
                stable_state = current_state;
                
                if (stable_state == true) 
                { 
                    contador++;
                    if(contador == 4)
                    {
                        contador = 0;
                    }
                    xQueueOverwrite(cola_paginas, &contador);
                    
                }
            }
        }
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
//----------------------------------------------------TAREA QUE MANIPULA EL RTC---------------------------------------------------
void task_rtc(void *pvParameters)
{
    ds3231_time_t toma_fecha;

    while (true) 
    {
        if (ds3231_get_time(I2C, &toma_fecha)) 
        {
            printf("Hora: %02d:%02d:%02d - Fecha: %02d/%02d/20%02d\n",
                  toma_fecha.hours,
                  toma_fecha.minutes,
                  toma_fecha.seconds,
                  toma_fecha.date,
                  toma_fecha.month,
                  toma_fecha.year);
            xQueueSend(queue_rtc,&toma_fecha,pdMS_TO_TICKS(100)); //Si se usa maxPORT_DELAY se bloqeuara
        } 
        else 
        {
            printf("Error leyendo el RTC\n");
            toma_fecha.hours = 0;
            toma_fecha.minutes = 0;
            toma_fecha.seconds = 0;
            toma_fecha.date = 0;
            toma_fecha.month = 0;
            toma_fecha.year = 0;
            xQueueSend(queue_rtc,&toma_fecha,portMAX_DELAY);     
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
//----------------------------------------------------TAREA DE XXXXXXXXX------------------------------------------------------------

/*---------------------------------------------------PROGRAMA PRINCIPAL-----------------------------------------------------------*/
int main(void) 
{
    stdio_init_all();
    configuracion_gpio_boton();

    // Creacion de colas
    queue_rtc = xQueueCreate(5,sizeof(float));
    queue_hcsr04 = xQueueCreate(5,sizeof(float));
    queue_setpoint = xQueueCreate(5,sizeof(estructura_setpoint));
    queue_pwm = xQueueCreate(5,sizeof(uint16_t));
    queue_altura = xQueueCreate(5,sizeof(uint16_t));
    queue_max_salida = xQueueCreate(5,sizeof(uint16_t));
    queue_min_salida = xQueueCreate(5,sizeof(uint16_t));
    cola_paginas = xQueueCreate(1, sizeof(uint8_t));   // cola que posee una unica posicion para memorizar el cambio de paginas
    //xQueueOverwrite(cola_paginas, &pagina);
    // Creacion de tareas
    xTaskCreate(task_init, "Init", 256, NULL, 3, NULL);
    xTaskCreate(task_SetPoint,"SetPoint",256,NULL,2,NULL);
    //xTaskCreate(task_monitor_gpio,"boton",256,NULL,2,NULL);
    //xTaskCreate(task_hcsr04,"MedicionDeDistancia",256,NULL,2,NULL);
    xTaskCreate(task_guardiana_sd,"guardianaSD",256,NULL,2,NULL);
    //xTaskCreate(task_guardiana_lcd,"guardianaLCD",256,NULL,2,NULL);
    xTaskCreate(task_debounce_boton, "debounce_boton", 1024, NULL, 1, NULL);
    //xTaskCreate(task_guardiana_leds,"guardianaLEDS",256,NULL,2,NULL);
    xTaskCreate(task_rtc,"regsitro_fecha",256,NULL,2,NULL);

    // Arranca el scheduler
    vTaskStartScheduler();
    while(1);
}