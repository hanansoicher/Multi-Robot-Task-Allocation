#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define UART_TX_PIN 28
#define UART_RX_PIN 29
#define UART_ID uart0
#define BAUD_RATE 9600
#define BUFFER_SIZE 256

static char rx_buffer[BUFFER_SIZE];
static volatile int rx_index = 0;

void setup() {
    stdio_init_all();
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_irqs_enabled(UART_ID, true, true);
    uart_set_fifo_enabled(UART_ID, true);
}

bool wait_for_response(const char* expected, uint timeout_ms) {
    absolute_time_t timeout_time = make_timeout_time_ms(timeout_ms);
    rx_index = 0;
    memset(rx_buffer, 0, BUFFER_SIZE);
    
    while (!time_reached(timeout_time)) {
        while (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            if (rx_index < BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = c;
                rx_buffer[rx_index] = '\0';
                
                if (strstr(rx_buffer, expected) != NULL) {
                    return true;
                }
            }
        }
        sleep_ms(10);
    }
    return false;
}

int main() {
    setup();
    uart_puts(UART_ID, "AT+NAMERobot1");
    uart_puts(UART_ID, "\r\n");
    if (!wait_for_response("OK", 1000)) {
        printf("Failed to set device name\n");
        return 1;
    }
    
    // Set slave mode
    uart_puts(UART_ID, "AT+ROLE0");
    uart_puts(UART_ID, "\r\n");
    if (!wait_for_response("OK", 1000)) {
        printf("Failed to set slave mode\n");
        return 1;
    }
    
    printf("Bluetooth module initialized successfully\n");

    // while (uart_is_readable(UART_ID)) {
    //     char c = uart_getc(UART_ID);
    //     if (rx_index < BUFFER_SIZE - 1) {
    //         rx_buffer[rx_index++] = c;
            
    //         if (c == '\n' || c == '\r') {
    //             rx_buffer[rx_index] = '\0';
    //             printf("Received: %s", rx_buffer);
    //             rx_index = 0;
    //             memset(rx_buffer, 0, BUFFER_SIZE);
    //     }
    //     } else {
    //         rx_index = 0;
    //         memset(rx_buffer, 0, BUFFER_SIZE);
    //     }

    // }
}