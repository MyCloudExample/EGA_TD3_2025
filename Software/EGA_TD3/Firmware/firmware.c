#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "lcd.h"
#include "pwm_lib.h"
#include "HC_SR04.h"
#include "ds3231.h"
#include "pid_controller.h"
#include "hardware/uart.h"
#include "string.h"
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"
//========================================CABECERAS DE FREERTOS=====================================================================
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/*-------------------------------------DEFINICION DE PINES PARA EL PROYECTO---------------------------------------------------------*/
//========================================DEFINICION DE I2C=========================================================================
#define PIN_SDA     8       //Pin 11 de la placa
#define PIN_SCL     9       //Pin 12 de la placa
#define I2C         i2c0    //Puerto del i2c
#define ADDR        0x27    //Direccion del LCD en I2C
#define FREQ        400000  //Frecuencia de 100KHz para el i2c
//========================================DEFINICION DE FAN=========================================================================
#define PIN_PWM     11      //Pin 21 de la placa
#define PIN_RPM     10      //Pin 22 de la placa
//NOTA EN LA PLACA DE PRUEBA SE USA EL PIN 11 PARA EL PWM, PERO EN LA PLACA PCB SE UTILIZA EL PIN 10, RECORDAR CAMBIARLO
//========================================PINES DE HC-SR04==========================================================================
#define PIN_TRIG    14      //Pin 19 de la placa
#define PIN_ECHO    15      //Pin 20 de la placa
//========================================PIN DE POTENCIOMETRO PARA EL SETPOINT=====================================================
#define PIN_ADC     26      //Pin 31 de la placa
//=======================================PINES PARA EL UART=========================================================================
#define PIN_TX  4           //Pin 6 de la placa
#define PIN_RX  5           //Pin 7 de la placa
#define UART_ID uart1       //Se utiliza el pueto UART 1
#define UART_BAUDRATE 115200//Velocidad del UART 1
//========================================BANDERAS DE ALERTAS=======================================================================
#define GPIO_LED_MAX 12     //Pin 16 de la placa
#define GPIO_LED_MIN 13     //Pin 17 de la placa
#define ALERTA_TIMEOUT_MS 3000//Tiempo de encendido, solo se uso para testeo
//=======================================BOTON SE SELECCION PARA LOS SETPOINT=======================================================
#define PIN_PAGINA  18      // Pin 24 de la placa
#define DEBOUNCE_TIME_MS 50//Tiempo para evitar rebotes
#define MULTI_PRESS_TIMEOUT 300//Tiempo para evitar rebotes
//========================================PARAMETROS FISICOS PARA EL CONTROL PID====================================================
#define BALL_DIAMETER_CM 7.8f //Diametro de la pelota
#define BALL_WEIGHT_G 9.0f   //Peso de la pelota
#define TARGET_HEIGHT 20.0f //Para testeo de la tarea task_pid
#define SENSOR_HEIGHT 45.0f //Altura del sensor
//========================================PARAMETROS INICIALES (SE DEBEN AJUSTAR DURANTE LAS PRUEBAS)===============================
#define BASE_PWM 2500       // Incrementado inicialmente
#define KP 25.0f             // Cuanto mayor sea el Kp mas rapida sera la respuesta, si es muy grande habra osiclacion e inestabilidad
#define KI 3.0f             // Elimina error en regimen permanente, si es muy grande la respuesta sera lenta y existira overshot
#define KD 2.5f             // Amortigua las oscilaciones, si es muy alto provocara oscilacion ya que amplifica el ruido
#define MIN_PWM 1800        // Mínimo absoluto
#define MAX_PWM 4000        // Máximo seguro
#define DT      0.01f       //Factor para ajustar el tiempo de muestreo
/*----------------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------VARAIBLES DE RPOGRAMA, COLAS Y SEMAFOROS--------------------------------------------------*/
//========================================VARIABLES QUE NO SON PROPIAS DE FREERTOS==================================================
pwm_config_t cooler={.pin=PIN_PWM, .wrap=4999, .clk_div=1.0f};
hc_sr04_t sensor;
typedef struct
{
    uint32_t setpoint;
    float setpoint_min;
    float setpoint_max;
}estructura_setpoint;
//========================================ELEMENTOS DE FREERTOS=====================================================================
SemaphoreHandle_t sem_mutexi2c;//Para sincronizar el uso del I2C por parte del LCD y el RTC
QueueHandle_t queue_rtc; //Envia datos desde task_rtc a task_pid, task_guardian_lcd
QueueHandle_t queue_hcsr04; //Envia datos desde task_hc_sr04 a task_pid, task_guardiana_lcd, task_guardiana_sd
QueueHandle_t queue_setpoint; //Envia datos desde task_setpoint a task_pid, task_guardiana_lcd, task_guardiana_sd
QueueHandle_t queue_leds; //Envia datos a la tarea task_guardiana_leds 
QueueHandle_t queue_sd; //Envia datos a la memoria SD
QueueHandle_t queue_pid; //Envia datos al PID
QueueHandle_t queue_min;
QueueHandle_t queue_max_salida;
QueueHandle_t queue_min_salida;
QueueHandle_t cola_paginas; //Envia datos a la tarea tas_setpoint
/*----------------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------TAREAS DE FREERTOS------------------------------------------------------------------------*/
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
    lcd_string("UTN-FRA | TD3 | G:9");
    for (uint32_t i = 0; i <10000000; i++)
    {}
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
//----------------------------------------TAREA DE SENSANDO DE LA ALTURA------------------------------------------------------------
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
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
//----------------------------------------TAREA GUARDIANA LCD-----------------------------------------------------------------------
void task_guardiana_lcd(void *pvParameter) 
{
    float val_hcsr04=0.0f;
    estructura_setpoint recepcion_lcd;
    char buffer[30];

    while (true) 
    {
        if(xSemaphoreTake(sem_mutexi2c,portMAX_DELAY) == pdTRUE)
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
        }
        xSemaphoreGive(sem_mutexi2c);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//----------------------------------------TAREA GUARDIANA DE MODULO SD--------------------------------------------------------------
void task_guardiana_sd(void *params) 
{ estructura_setpoint datasd;
  ds3231_time_t rtcsd;
  char buffer1[200], buffer2[200];
  FATFS fs;
  FIL fil;
  const char* const filename = "dataloger.txt";
  ds3231_time_t toma_fecha;

    while(true) 
    {
        xQueueReceive(queue_rtc, &toma_fecha,pdMS_TO_TICKS(100));
        xQueueReceive(queue_sd, &datasd,pdMS_TO_TICKS(100));
        vTaskDelay(5000);
        sprintf(buffer1,"Hora: %02d:%02d:%02d,Fecha: %02d/%02d/20%02d",toma_fecha.hours, toma_fecha.minutes, toma_fecha.seconds, toma_fecha.date, toma_fecha.month, toma_fecha.year);
        printf("TEXTO: %s\n",buffer1);
        vTaskDelay(5000);
        sprintf(buffer2,"Setpoint: %lu, SetpointMax: %.2f, SetpointMin: %.2f",datasd.setpoint, datasd.setpoint_max, datasd.setpoint_min);
        printf("Setpoint: %s\n",buffer2);
        vTaskDelay(5000);
        
        FRESULT fr = f_mount(&fs, "", 1);
        if (FR_OK != fr) 
        {
            panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        }

        // Se abre el archivo y se escribe en el archivo
        fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
        if (FR_OK != fr && FR_EXIST != fr) 
        {
            panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
        }
        if (f_printf(&fil, buffer1) < 0 || f_printf(&fil, buffer2)) 
        {
            printf("Escritura fallida\n");
        }

        // Cierra el archivo
        fr = f_close(&fil);
        if (FR_OK != fr) 
        {
            printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
        }

        // Desmonta la memoria SD
        f_unmount("");

        puts("Goodbye, world!");
        for (;;);
    }
}
//----------------------------------------TAREA GUARDIANA DE LEDS-------------------------------------------------------------------
void task_guardiana_leds(void *params) 
{
    estructura_setpoint data;
    float altura = 0;
    gpio_init(GPIO_LED_MAX); //Inicio el pin 16
    gpio_set_dir(GPIO_LED_MAX, GPIO_OUT); //Se configura como salida
    gpio_put(GPIO_LED_MAX, 0); // Se coloca un 0 a la salida
    gpio_init(GPIO_LED_MIN); //Inicio el pin 17
    gpio_set_dir(GPIO_LED_MIN, GPIO_OUT); //Se configura como salida
    gpio_put(GPIO_LED_MIN, 0); // Se coloca un 0 a la salida

    while(true)
    {
        if (xQueueReceive(queue_leds, &data, portMAX_DELAY) == pdPASS) 
        {
            //printf("TARGET:%lu,MAX:%.2f,MIN:%.2f",data.setpoint,data.setpoint_max,data.setpoint_min);
        }

        if (xQueueReceive(queue_hcsr04, &altura, portMAX_DELAY) == pdPASS) 
        {
            if(altura > data.setpoint_max)
            {
                gpio_put(GPIO_LED_MAX,true);
                //vTaskDelay(pdMS_TO_TICKS(100));
            }
            if(altura < data.setpoint_max)
            {
                gpio_put(GPIO_LED_MAX,false);
                //vTaskDelay(pdMS_TO_TICKS(100));
            }
            if(altura < data.setpoint_min)
            {
                gpio_put(GPIO_LED_MIN,true);
                //vTaskDelay(pdMS_TO_TICKS(100));
            }
            if(altura > data.setpoint_min)
            {
                gpio_put(GPIO_LED_MIN,false);
                //vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

    }
}
//----------------------------------------TAREA PARA INICAR EL SETPOINT-------------------------------------------------------------
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
            if(valor_altura > 28)
            {
                data.setpoint = 28;
            }
            if(valor_altura == 0)
            {
                data.setpoint = 5;
            }
            //printf("PAGINA 1 |setpoint= %lu | Valor altura= %lu \n", data.setpoint, valor_altura);
        } 
        if(pagina==2) 
        {
            valor_adc = adc_read();
            tension = (valor_adc * 3.3f) / 4095; 
            valor_altura = ((valor_adc * 3.3f) / 4095)*10;
            data.setpoint_max = valor_altura;
            if(valor_altura > data.setpoint)
            {
                data.setpoint_max = valor_altura;
            }
            else
            {
                data.setpoint_max = data.setpoint + 1;
            }
            
            //printf("PAGINA 2 |setpointMax= %.2f | Valor altura= %lu \n", data.setpoint_max, valor_altura);
        }
        if(pagina==3) 
        {
            valor_adc = adc_read();
            tension = (valor_adc * 3.3f) / 4095; 
            valor_altura = ((valor_adc * 3.3f) / 4095)*10;
           if(valor_altura < data.setpoint)
            {
                data.setpoint_min = valor_altura;
            }
            else
            {
                data.setpoint_min = data.setpoint - 1.0f;
            }
            //printf("PAGINA 3 |setpointMin= %.2f | Valor altura= %lu \n", data.setpoint_min, valor_altura);
        }
        if(pagina==0) 
        {
            
            printf("PAGINA 0 |setpoint= %lu | setpoint_max=%.2f | setpoint_min= %.2f \n", data.setpoint,data.setpoint_max,data.setpoint_min);
        }
       xQueueSend(queue_setpoint, &data, pdMS_TO_TICKS(10)); //Se quedara aqui ya que no se desopucpa la cola
       xQueueSend(queue_pid, &data, pdMS_TO_TICKS(10));
       xQueueSend(queue_leds,&data,pdMS_TO_TICKS(10));
       xQueueSend(queue_sd,&data,pdMS_TO_TICKS(10)); //Envia datos a la task_guardian_sd
       vTaskDelay(pdMS_TO_TICKS(100));
       //printf("Ejecucion \n");
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
//----------------------------------------TAREA QUE MANIPULA EL RTC-----------------------------------------------------------------
void task_rtc(void *pvParameters)
{
    ds3231_time_t toma_fecha;

    while (true) 
    {
        //printf("Fuera del mutex\n");
        if(xSemaphoreTake(sem_mutexi2c,portMAX_DELAY) == pdTRUE)
        {
            //printf("Dentro del mutex\n");
            if (ds3231_get_time(I2C, &toma_fecha)) 
            {
                //printf("Hora: %02d:%02d:%02d - Fecha: %02d/%02d/20%02d\n",toma_fecha.hours, toma_fecha.minutes, toma_fecha.seconds, toma_fecha.date, toma_fecha.month, toma_fecha.year);
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
        }
        xSemaphoreGive(sem_mutexi2c);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
//---------------------------------------TAREA DE CONTROL PID-----------------------------------------------------------------------
void task_pid(void *pvParameters)
{ 
    float TARGET=0.0f;
    estructura_setpoint data;
    //static uint16_t last_pwm = BASE_PWM;
   // 1. Verificación PWM
    pwm_config_t fan = {
        .pin = PIN_PWM,
        .wrap = 4999,
        .clk_div = 1.0f
    };
    pwm_init_config(&fan);
    
    // Prueba manual del ventilador
    /*printf("Prueba manual del ventilador:\n");
    for(int pwm = 3500; pwm <= MAX_PWM; pwm += 500) {
        pwm_set_level(&fan, pwm);
        printf("PWM: %d, ¿Está soplando? (espera 3 seg)\n", pwm);
        sleep_ms(3000);
    }*/

    // 2. Verificación sensor
    hc_sr04_t sensor;
    hc_sr04_init(&sensor, PIN_TRIG, PIN_ECHO);
    /*printf("\nPrueba del sensor:\n");
    for(int i = 0; i < 5; i++) {
        float dist = hc_sr04_get_distance_cm(&sensor);
        printf("Medición %d: %.2f cm\n", i+1, dist);
        sleep_ms(200);
    }*/

    // 3. Sistema completo
    PIDController pid = {
        .Kp = KP, .Ki = KI, .Kd = KD,
        .tau = 0.03f,
        .limMin = MIN_PWM,
        .limMax = MAX_PWM,
        .integrator = (MAX_PWM + MIN_PWM)/2.0f,
        .prevError = 0,
        .differentiator = 0,
        .prevmedicion = TARGET,
        .out = MAX_PWM * 0.7f
    };
    PIDController_Init(&pid);

    float filtered_height = TARGET;
    const float alpha = 0.3f;
    pwm_set_level(&fan,3500);
    sleep_ms(3000);
    printf("Fin de prueba de elevacion con 3500\n");

    while(true) 
    {
        xQueueReceive(queue_pid,&data,pdMS_TO_TICKS(5));
        //xQueueReceive(queue_setpoint, &data, pdMS_TO_TICKS(5));
        printf("Altura de task_setpoint:%lu, %.2f, %.2f\n",data.setpoint, data.setpoint_max,data.setpoint_min);
        TARGET = (float)(data.setpoint);
        //Medición robusta
        float raw_dist = hc_sr04_get_distance_cm(&sensor);
        if(raw_dist <= 2.0f && raw_dist >= SENSOR_HEIGHT -2.0f) 
        {
            //printf("Medicion invalida:%.2f cm\n",raw_dist);
            pwm_set_level(&fan,MAX_PWM*0.8f);
            continue;
        } 
        float current_height = SENSOR_HEIGHT - raw_dist;
        xQueueSend(queue_hcsr04,&current_height,pdMS_TO_TICKS(5));
        filtered_height = alpha * current_height + (1-alpha) * filtered_height;
        //Control PID
        float control = PIDController_Update(&pid, TARGET, filtered_height, 0.02f);
        uint16_t pwm = (int16_t)(control);
        //Limites de seguridad con histeresis
        static uint16_t last_pwm = BASE_PWM;
        if(abs(pwm - last_pwm > 100))
        {
            pwm = (pwm + last_pwm*2)/3; //Promedio ponderado
        }
        pwm = (pwm < MIN_PWM) ? MIN_PWM : (pwm > MAX_PWM) ? MAX_PWM : pwm;
        pwm_set_level(&fan, pwm);
        last_pwm=pwm;

        //printf("Control: %.2f | PWM: %d\n", control, pwm);
        //printf("Alt: %.2fcm | PWM: %4d | Err: %.2f\n", filtered_height, pwm, TARGET_HEIGHT - filtered_height);
        //printf("Altura:%.2f,Maximo:45,Minimo:0,SETPOINT:%lu\n",filtered_height, data.setpoint); //Para Serial Plotter en Arduino IDE
        vTaskDelay(pdMS_TO_TICKS((int)(DT*1000)));
    }    
           
}
/*----------------------------------------PROGRAMA PRINCIPAL------------------------------------------------------------------------*/
int main(void) 
{
    stdio_init_all();
    configuracion_gpio_boton();

    // Creacion de colas
    queue_rtc = xQueueCreate(5,sizeof(float));
    queue_hcsr04 = xQueueCreate(5,sizeof(float));
    queue_setpoint = xQueueCreate(5,sizeof(estructura_setpoint));
    queue_leds = xQueueCreate(5,sizeof(estructura_setpoint));
    queue_sd = xQueueCreate(5,sizeof(estructura_setpoint));
    queue_pid = xQueueCreate(5,sizeof(estructura_setpoint));
    queue_min_salida = xQueueCreate(5,sizeof(uint16_t));
    cola_paginas = xQueueCreate(1, sizeof(uint8_t));   // cola que posee una unica posicion para memorizar el cambio de paginas
    //xQueueOverwrite(cola_paginas, &pagina);
    sem_mutexi2c = xSemaphoreCreateMutex();
    // Creacion de tareas
    xTaskCreate(task_init, "Init", 256, NULL, 3, NULL);
    xTaskCreate(task_SetPoint,"SetPoint",256,NULL,2,NULL);
    //xTaskCreate(task_monitor_gpio,"boton",256,NULL,2,NULL);
    //xTaskCreate(task_hcsr04,"MedicionDeDistancia",256,NULL,2,NULL);
    //xTaskCreate(task_guardiana_sd,"guardianaSD",2048,NULL,2,NULL);
    xTaskCreate(task_guardiana_lcd,"guardianaLCD",256,NULL,2,NULL);
    xTaskCreate(task_debounce_boton, "debounce_boton", 1024, NULL, 1, NULL);
    xTaskCreate(task_guardiana_leds,"guardianaLEDS",256,NULL,2,NULL);
    xTaskCreate(task_rtc,"regsitro_fecha",256,NULL,2,NULL);
    xTaskCreate(task_pid,"control_pid",256,NULL,2,NULL);

    // Arranca el scheduler
    vTaskStartScheduler();
    while(1);
}