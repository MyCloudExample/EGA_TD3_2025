#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Recursos compartidos
QueueHandle_t xColaDatos;
SemaphoreHandle_t xMutexRecurso;

// Estructura para datos de la cola
typedef struct {
    uint32_t valor;
    char tarea_nombre[20];
} dato_cola_t;

// Tarea de ALTA prioridad - Procesa datos
void vTareaAltaPrioridad(void *pvParameters) {
    dato_cola_t dato_recibido;
    
    printf("Tarea Alta Prioridad: Iniciada (Prioridad: 3)\n");
    
    while (1) {
        // Esperar dato de la cola
        if (xQueueReceive(xColaDatos, &dato_recibido, portMAX_DELAY) == pdTRUE) {
            printf("Tarea Alta: Dato recibido - %s: %lu\n", 
                   dato_recibido.tarea_nombre, dato_recibido.valor);
            
            // Intentar acceder al recurso compartido
            printf("Tarea Alta: Intentando tomar mutex...\n");
            uint32_t inicio = to_ms_since_boot(get_absolute_time());
            
            if (xSemaphoreTake(xMutexRecurso, pdMS_TO_TICKS(2000)) == pdTRUE) {
                uint32_t fin = to_ms_since_boot(get_absolute_time());
                uint32_t tiempo_espera = fin - inicio;
                
                printf("Tarea Alta: Mutex tomado después de %lums (INVERSIÓN DETECTADA!)\n", 
                       tiempo_espera);
                
                // Procesamiento del recurso
                printf("Tarea Alta: Procesando recurso...\n");
                vTaskDelay(pdMS_TO_TICKS(100)); // Simular procesamiento
                
                // Liberar mutex
                xSemaphoreGive(xMutexRecurso);
                printf("Tarea Alta: Mutex liberado\n");
            } else {
                printf("Tarea Alta: TIMEOUT - No se pudo acceder al recurso\n");
            }
        }
    }
}

// Tarea de MEDIA prioridad - Tarea intermedia
void vTareaMediaPrioridad(void *pvParameters) {
    printf("Tarea Media Prioridad: Iniciada (Prioridad: 2)\n");
    
    while (1) {
        printf("Tarea Media: Ejecutando trabajo no crítico...\n");
        vTaskDelay(pdMS_TO_TICKS(800)); // Trabajo periódico
        
        // Esta tarea puede bloquear a la de alta prioridad
        printf("Tarea Media: Realizando trabajo extenso...\n");
        for (int i = 0; i < 5; i++) {
            vTaskDelay(pdMS_TO_TICKS(200)); // Trabajo CPU intensivo
            printf("Tarea Media: Ciclo %d/5\n", i + 1);
        }
        printf("Tarea Media: Trabajo completado\n");
    }
}

// Tarea de BAJA prioridad - Genera datos
void vTareaBajaPrioridad(void *pvParameters) {
    dato_cola_t dato_a_enviar;
    uint32_t contador = 0;
    
    printf("Tarea Baja Prioridad: Iniciada (Prioridad: 1)\n");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1500)); // Generar datos cada 1.5s
        
        // Tomar el mutex para acceder al recurso
        printf("Tarea Baja: Intentando tomar mutex...\n");
        
        if (xSemaphoreTake(xMutexRecurso, portMAX_DELAY) == pdTRUE) {
            printf("Tarea Baja: Mutex tomado. Accediendo recurso...\n");
            
            // Simular uso extenso del recurso (CAUSA DE INVERSIÓN)
            printf("Tarea Baja: Usando recurso extensamente...\n");
            for (int i = 0; i < 3; i++) {
                vTaskDelay(pdMS_TO_TICKS(500)); // Uso prolongado
                printf("Tarea Baja: Usando recurso (%d/3)\n", i + 1);
            }
            
            // Preparar dato para la cola
            dato_a_enviar.valor = contador++;
            snprintf(dato_a_enviar.tarea_nombre, sizeof(dato_a_enviar.tarea_nombre), 
                    "BajaPrioridad");
            
            // Enviar dato a la cola (esto despertará a la tarea de alta prioridad)
            printf("Tarea Baja: Enviando dato a la cola...\n");
            xQueueSend(xColaDatos, &dato_a_enviar, portMAX_DELAY);
            
            // Continuar usando el recurso después de enviar el dato
            printf("Tarea Baja: Continuando uso del recurso...\n");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Más uso del recurso
            
            // Liberar el mutex
            xSemaphoreGive(xMutexRecurso);
            printf("Tarea Baja: Mutex liberado\n");
        }
    }
}

// Tarea monitor - Muestra el estado del sistema
void vTareaMonitor(void *pvParameters) {
    printf("Tarea Monitor: Iniciada (Prioridad: 1)\n");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        printf("\n=== MONITOR: Estado del Sistema ===\n");
        printf("Tareas activas:\n");
        printf("- Alta Prioridad: Esperando recurso/datos\n");
        printf("- Media Prioridad: Ejecutando periódicamente\n");
        printf("- Baja Prioridad: Usando recurso compartido\n");
        printf("===================================\n\n");
    }
}

int main() {
    stdio_init_all();
    
    // Esperar a que la conexión serial esté lista
    sleep_ms(2000);
    printf("\n=== DEMO: INVERSIÓN DE PRIORIDAD ===\n\n");
    
    // Crear cola de datos
    xColaDatos = xQueueCreate(5, sizeof(dato_cola_t));
    
    // Crear mutex para recurso compartido
    xMutexRecurso = xSemaphoreCreateMutex();
    
    if (xColaDatos == NULL || xMutexRecurso == NULL) {
        printf("Error: No se pudieron crear recursos del sistema\n");
        return 1;
    }
    
    printf("Recursos creados exitosamente:\n");
    printf("- Cola de datos: 5 elementos\n");
    printf("- Mutex para recurso compartido\n\n");
    
    // Crear tareas con diferentes prioridades
    xTaskCreate(vTareaAltaPrioridad, "Alta Prioridad", 512, NULL, 3, NULL);
    xTaskCreate(vTareaMediaPrioridad, "Media Prioridad", 512, NULL, 2, NULL);
    xTaskCreate(vTareaBajaPrioridad, "Baja Prioridad", 512, NULL, 1, NULL);
    xTaskCreate(vTareaMonitor, "Monitor", 512, NULL, 1, NULL);
    
    printf("Tareas creadas con prioridades:\n");
    printf("1. Alta Prioridad: Nivel 3\n");
    printf("2. Media Prioridad: Nivel 2\n");
    printf("3. Baja Prioridad: Nivel 1\n");
    printf("4. Monitor: Nivel 1\n\n");
    
    printf("=== ESCENARIO DE INVERSIÓN ===\n");
    printf("1. Baja prioridad toma el mutex\n");
    printf("2. Baja prioridad envía dato y DESPIERTA alta prioridad\n");
    printf("3. Alta prioridad intenta tomar mutex pero DEBE ESPERAR\n");
    printf("4. Media prioridad EJECUTA y bloquea a ambas\n");
    printf("5. INVERSIÓN: Media (prio2) > Alta (prio3)\n\n");
    
    vTaskStartScheduler();
    
    while (1) {
        // Nunca debería llegar aquí
    }
    
    return 0;
}