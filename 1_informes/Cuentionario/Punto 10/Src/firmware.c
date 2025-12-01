#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Recursos compartidos (semáforos mutex)
SemaphoreHandle_t xMutexA;
SemaphoreHandle_t xMutexB;
QueueHandle_t xColaEventos;

// Estructura para mensajes de la cola
typedef struct {
    char mensaje[30];
    uint32_t timestamp;
} evento_t;

// Tarea 1: Intenta tomar Mutex A luego Mutex B
void vTarea1(void *pvParameters) {
    evento_t evento;
    
    printf("Tarea 1: Iniciada\n");
    
    while (1) {
        printf("Tarea 1: Intentando tomar Mutex A...\n");
        
        if (xSemaphoreTake(xMutexA, pdMS_TO_TICKS(1000)) == pdTRUE) {
            printf("Tarea 1: Mutex A tomado. Esperando 500ms...\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            
            printf("Tarea 1: Intentando tomar Mutex B...\n");
            if (xSemaphoreTake(xMutexB, pdMS_TO_TICKS(1000)) == pdTRUE) {
                printf("Tarea 1: ¡ÉXITO! Ambos mutex tomados\n");
                
                // Procesamiento con ambos recursos
                printf("Tarea 1: Ejecutando sección crítica...\n");
                vTaskDelay(pdMS_TO_TICKS(200));
                
                // Enviar mensaje de éxito
                evento_t evt = {"Tarea1: Recursos obtenidos", to_ms_since_boot(get_absolute_time())};
                xQueueSend(xColaEventos, &evt, 0);
                
                // Liberar mutex en orden inverso
                xSemaphoreGive(xMutexB);
                printf("Tarea 1: Mutex B liberado\n");
            } else {
                printf("Tarea 1: FALLO - No se pudo tomar Mutex B\n");
                
                // Enviar mensaje de fallo
                evento_t evt = {"Tarea1: Fallo Mutex B", to_ms_since_boot(get_absolute_time())};
                xQueueSend(xColaEventos, &evt, 0);
            }
            
            xSemaphoreGive(xMutexA);
            printf("Tarea 1: Mutex A liberado\n");
        } else {
            printf("Tarea 1: FALLO - No se pudo tomar Mutex A\n");
        }
        
        printf("Tarea 1: Ciclo completado. Esperando 1.5s...\n\n");
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// Tarea 2: Intenta tomar Mutex B luego Mutex A (ORDEN INVERSO - CAUSA DE DEADLOCK)
void vTarea2(void *pvParameters) {
    printf("Tarea 2: Iniciada\n");
    
    while (1) {
        printf("Tarea 2: Intentando tomar Mutex B...\n");
        
        if (xSemaphoreTake(xMutexB, pdMS_TO_TICKS(1000)) == pdTRUE) {
            printf("Tarea 2: Mutex B tomado. Esperando 500ms...\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            
            printf("Tarea 2: Intentando tomar Mutex A...\n");
            if (xSemaphoreTake(xMutexA, pdMS_TO_TICKS(1000)) == pdTRUE) {
                printf("Tarea 2: ¡ÉXITO! Ambos mutex tomados\n");
                
                // Procesamiento con ambos recursos
                printf("Tarea 2: Ejecutando sección crítica...\n");
                vTaskDelay(pdMS_TO_TICKS(200));
                
                // Enviar mensaje de éxito
                evento_t evt = {"Tarea2: Recursos obtenidos", to_ms_since_boot(get_absolute_time())};
                xQueueSend(xColaEventos, &evt, 0);
                
                // Liberar mutex en orden inverso
                xSemaphoreGive(xMutexA);
                printf("Tarea 2: Mutex A liberado\n");
            } else {
                printf("Tarea 2: FALLO - No se pudo tomar Mutex A\n");
                
                // Enviar mensaje de fallo
                evento_t evt = {"Tarea2: Fallo Mutex A", to_ms_since_boot(get_absolute_time())};
                xQueueSend(xColaEventos, &evt, 0);
            }
            
            xSemaphoreGive(xMutexB);
            printf("Tarea 2: Mutex B liberado\n");
        } else {
            printf("Tarea 2: FALLO - No se pudo tomar Mutex B\n");
        }
        
        printf("Tarea 2: Ciclo completado. Esperando 1.5s...\n\n");
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// Tarea Monitor: Recibe eventos de la cola y detecta deadlocks
void vTareaMonitor(void *pvParameters) {
    evento_t evento;
    uint32_t ultimo_evento = 0;
    uint32_t tiempo_actual = 0;
    
    printf("Monitor: Iniciado. Detectando deadlocks...\n");
    
    while (1) {
        // Revisar la cola de eventos
        if (xQueueReceive(xColaEventos, &evento, pdMS_TO_TICKS(100)) == pdTRUE) {
            printf("EVENTO: %s [%lu ms]\n", evento.mensaje, evento.timestamp);
            ultimo_evento = evento.timestamp;
        }
        
        // Detectar posible deadlock (silencia prolongada)
        tiempo_actual = to_ms_since_boot(get_absolute_time());
        if ((tiempo_actual - ultimo_evento) > 3000 && ultimo_evento != 0) {
            printf("\n¡¡¡ ALERTA DE DEADLOCK DETECTADO !!!\n");
            printf("Tiempo sin eventos: %lums\n", tiempo_actual - ultimo_evento);
            printf("Las tareas están bloqueadas mutuamente\n\n");
            ultimo_evento = tiempo_actual; // Resetear para no repetir alerta
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main() {
    stdio_init_all();
    
    // Esperar a que la conexión serial esté lista
    sleep_ms(2000);
    printf("\n=== DEMOSTRACIÓN DE DEADLOCK (PUNTO MUERTO) ===\n\n");
    
    // Crear los mutex
    xMutexA = xSemaphoreCreateMutex();
    xMutexB = xSemaphoreCreateMutex();
    
    // Crear la cola de eventos
    xColaEventos = xQueueCreate(10, sizeof(evento_t));
    
    if (xMutexA == NULL || xMutexB == NULL || xColaEventos == NULL) {
        printf("Error: No se pudieron crear los recursos del sistema\n");
        return 1;
    }
    
    printf("Recursos creados exitosamente:\n");
    printf("- Mutex A y Mutex B\n");
    printf("- Cola de eventos (10 elementos)\n\n");
    
    printf("ESCENARIO DE DEADLOCK:\n");
    printf("Tarea 1: Toma A → Intenta tomar B\n");
    printf("Tarea 2: Toma B → Intenta tomar A\n");
    printf("RESULTADO: Ambas se bloquean mutuamente\n\n");
    
    // Crear las tareas
    xTaskCreate(vTarea1, "Tarea 1", 512, NULL, 2, NULL);
    xTaskCreate(vTarea2, "Tarea 2", 512, NULL, 2, NULL);
    xTaskCreate(vTareaMonitor, "Monitor", 512, NULL, 1, NULL);
    
    printf("Tareas iniciadas. Observar la salida serial...\n");
    printf("El deadlock ocurrirá cuando:\n");
    printf("1. Tarea 1 tome Mutex A y sea interrumpida\n");
    printf("2. Tarea 2 tome Mutex B e intente tomar Mutex A\n");
    printf("3. Tarea 1 reintente y necesite Mutex B\n\n");
    
    vTaskStartScheduler();
    
    // Nunca deberíamos llegar aquí
    while (1) {
        // El scheduler debería mantener el control
    }
    
    return 0;
}