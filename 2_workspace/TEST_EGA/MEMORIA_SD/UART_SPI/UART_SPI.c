#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdio.h>

#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 4
#define UART_RX_PIN 5

int main() {
    stdio_init_all();
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    while (1) {
        const char *message = "Dato1,Dato2,Dato3\n";  // Formato CSV para facilitar el almacenamiento
        uart_puts(UART_ID, message);
        sleep_ms(1000);  // Enviar cada 1 segundo
        printf("DATO ENVIADO\n");
    }
}