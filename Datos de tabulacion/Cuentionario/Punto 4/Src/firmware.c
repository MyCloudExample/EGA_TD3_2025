#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Recurso compartido: Cola de mensajes
QueueHandle_t xColaDatos;

// Estructura para datos de sensor
typedef struct {
    uint8_t id_sensor;
    int16_t valor;
    uint32_t timestamp;
} dato_sensor_t;

// Tarea LECTORA (única)
void vTareaLectora(void *pvParameters) {
    dato_sensor_t dato_recibido;
    
    printf("Lector: Iniciado. Esperando datos...\n");
    
    while (1) {
        // Bloqueo por LECTURA (espera indefinida por datos)
        if (xQueueReceive(xColaDatos, &dato_recibido, portMAX_DELAY) == pdTRUE) {
            printf("Lector: Dato recibido - Sensor %d: %d [%lu ms]\n",
                   dato_recibido.id_sensor, dato_recibido.valor, dato_recibido.timestamp);
            
            // Procesamiento del dato (simulado)
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// Tarea ESCRITORA 1 (Sensor Temperatura)
void vTareaEscritora1(void *pvParameters) {
    dato_sensor_t dato_enviar;
    dato_enviar.id_sensor = 1; // ID Sensor temperatura
    
    printf("Escritor1 (Temp): Iniciado\n");
    
    while (1) {
        // Simular lectura de sensor
        dato_enviar.valor = 20 + (rand() % 10); // 20-30°C
        dato_enviar.timestamp = to_ms_since_boot(get_absolute_time());
        
        // Bloqueo por ESCRITURA (timeout de 100ms)
        if (xQueueSend(xColaDatos, &dato_enviar, pdMS_TO_TICKS(100)) == pdTRUE) {
            printf("Escritor1: Dato enviado - Temp: %d°C\n", dato_enviar.valor);
        } else {
            printf("Escritor1: ERROR - Cola llena, dato perdido\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Muestreo cada 500ms
    }
}

// Tarea ESCRITORA 2 (Sensor Humedad)
void vTareaEscritora2(void *pvParameters) {
    dato_sensor_t dato_enviar;
    dato_enviar.id_sensor = 2; // ID Sensor humedad
    
    printf("Escritor2 (Hum): Iniciado\n");
    
    while (1) {
        // Simular lectura de sensor
        dato_enviar.valor = 40 + (rand() % 30); // 40-70%
        dato_enviar.timestamp = to_ms_since_boot(get_absolute_time());
        
        // Bloqueo por ESCRITURA (timeout de 100ms)
        if (xQueueSendToBack(xColaDatos, &dato_enviar, pdMS_TO_TICKS(100)) == pdTRUE) {
            printf("Escritor2: Dato enviado - Hum: %d%%\n", dato_enviar.valor);
        } else {
            printf("Escritor2: ERROR - Cola llena, dato perdido\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(300)); // Muestreo cada 300ms
    }
}

// Tarea ESCRITORA 3 (Sensor Presión)
void vTareaEscritora3(void *pvParameters) {
    dato_sensor_t dato_enviar;
    dato_enviar.id_sensor = 3; // ID Sensor presión
    
    printf("Escritor3 (Pres): Iniciado\n");
    
    while (1) {
        // Simular lectura de sensor
        dato_enviar.valor = 900 + (rand() % 100); // 900-1000 hPa
        dato_enviar.timestamp = to_ms_since_boot(get_absolute_time());
        
        // Bloqueo por ESCRITURA (timeout de 100ms)
        if (xQueueSendToFront(xColaDatos, &dato_enviar, pdMS_TO_TICKS(100)) == pdTRUE) {
            printf("Escritor3: Dato enviado - Pres: %d hPa\n", dato_enviar.valor);
        } else {
            printf("Escritor3: ERROR - Cola llena, dato perdido\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(700)); // Muestreo cada 700ms
    }
}

// Tarea Monitor: Muestra estado de la cola
void vTareaMonitor(void *pvParameters) {
    while (1) {
        UBaseType_t espacios_libres = uxQueueSpacesAvailable(xColaDatos);
        UBaseType_t mensajes_en_cola = uxQueueMessagesWaiting(xColaDatos);
        
        printf("\n[MONITOR] Cola: %d msgs, %d espacios libres\n",
               mensajes_en_cola, espacios_libres);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Esperar inicialización USB
    
    printf("\n=== PATRÓN: MÚLTIPLES ESCRITORES - UN LECTOR ===\n\n");
    
    // Crear cola con capacidad para 10 elementos
    xColaDatos = xQueueCreate(10, sizeof(dato_sensor_t));
    
    if (xColaDatos == NULL) {
        printf("Error: No se pudo crear la cola\n");
        return 1;
    }
    
    printf("Cola creada (10 elementos de %d bytes)\n\n", sizeof(dato_sensor_t));
    
    // Crear tareas
    xTaskCreate(vTareaLectora, "Lector", 512, NULL, 3, NULL);      // Alta prioridad
    xTaskCreate(vTareaEscritora1, "Escritor1", 512, NULL, 2, NULL);
    xTaskCreate(vTareaEscritora2, "Escritor2", 512, NULL, 2, NULL);
    xTaskCreate(vTareaEscritora3, "Escritor3", 512, NULL, 2, NULL);
    xTaskCreate(vTareaMonitor, "Monitor", 512, NULL, 1, NULL);
    
    printf("Tareas creadas:\n");
    printf("- 1 Lector (prio 3)\n");
    printf("- 3 Escritores (prio 2)\n");
    printf("- 1 Monitor (prio 1)\n\n");
    
    vTaskStartScheduler();
    
    while (1) {}
    return 0;
}