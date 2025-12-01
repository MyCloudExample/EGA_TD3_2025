#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Semáforo mutex global
SemaphoreHandle_t xMutex;

// Recurso compartido (variable crítica)
int contador_compartido = 0;

// Tarea 1: Incrementa el contador
void vTareaIncrementar(void *pvParameters) {
    const char *nombre_tarea = (char *)pvParameters;
    
    while (1) {
        printf("%s: Intentando tomar el mutex...\n", nombre_tarea);
        
        // Intentar tomar el mutex (espera máxima de 1000ms)
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            printf("%s: Mutex tomado. Accediendo recurso...\n", nombre_tarea);
            
            // Sección crítica protegida
            int temp = contador_compartido;
            printf("%s: Valor actual: %d\n", nombre_tarea, temp);
            
            // Simular procesamiento
            vTaskDelay(pdMS_TO_TICKS(200));
            
            contador_compartido = temp + 1;
            printf("%s: Nuevo valor: %d\n", nombre_tarea, contador_compartido);
            
            // Liberar el mutex
            xSemaphoreGive(xMutex);
            printf("%s: Mutex liberado\n", nombre_tarea);
        } else {
            printf("%s: Timeout - No se pudo tomar el mutex\n", nombre_tarea);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Tarea 2: Decrementa el contador
void vTareaDecrementar(void *pvParameters) {
    const char *nombre_tarea = (char *)pvParameters;
    
    while (1) {
        printf("%s: Esperando mutex...\n", nombre_tarea);
        
        // Tomar el mutex (espera indefinida)
        xSemaphoreTake(xMutex, portMAX_DELAY);
        printf("%s: Mutex tomado. Accediendo recurso...\n", nombre_tarea);
        
        // Sección crítica protegida
        int temp = contador_compartido;
        printf("%s: Valor actual: %d\n", nombre_tarea, temp);
        
        // Simular procesamiento más largo
        vTaskDelay(pdMS_TO_TICKS(300));
        
        contador_compartido = temp - 1;
        printf("%s: Nuevo valor: %d\n", nombre_tarea, contador_compartido);
        
        // Liberar el mutex
        xSemaphoreGive(xMutex);
        printf("%s: Mutex liberado\n", nombre_tarea);
        
        vTaskDelay(pdMS_TO_TICKS(700));
    }
}

// Tarea 3: Solo lee el contador (sin necesidad de mutex para lectura)
void vTareaLectura(void *pvParameters) {
    const char *nombre_tarea = (char *)pvParameters;
    
    while (1) {
        // Para lectura podemos acceder directamente (dependiendo del contexto)
        printf("%s: Leyendo valor: %d\n", nombre_tarea, contador_compartido);
        
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

int main() {
    // Inicializar stdio
    stdio_init_all();
    
    printf("\n=== Demo de Semáforos Mutex en FreeRTOS ===\n\n");
    
    // Crear el semáforo mutex
    xMutex = xSemaphoreCreateMutex();
    
    if (xMutex == NULL) {
        printf("Error: No se pudo crear el mutex\n");
        return 1;
    }
    
    printf("Mutex creado exitosamente\n");
    
    // Crear las tareas
    xTaskCreate(vTareaIncrementar, "Tarea Incrementar", 256, "Incrementar", 2, NULL);
    xTaskCreate(vTareaDecrementar, "Tarea Decrementar", 256, "Decrementar", 2, NULL);
    xTaskCreate(vTareaLectura, "Tarea Lectura", 256, "Lectura", 1, NULL);
    
    // Iniciar el scheduler de FreeRTOS
    vTaskStartScheduler();
    
    // Nunca deberíamos llegar aquí
    while (1) {
        // El scheduler debería mantener el control
    }
    
    return 0;
}