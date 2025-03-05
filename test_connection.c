#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pico/stdlib.h>
#include <pololu_3pi_2040_robot.h>

#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 28
#define UART_RX_PIN 29
#define DATA_BITS 8
#define STOP_BITS 1

#define PIO_ID pio0
#define RIGHT_ENCODER_AB 8
#define LEFT_ENCODER_AB 12
#define RIGHT_SM 0
#define LEFT_SM 1

#define DEFAULT_MOTOR_SPEED 1000
#define PID_UPDATE_RATE_MS 10
#define ENCODER_TICKS_PER_CM 40 // Need to calibrate


void init_ble() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    uart_puts(UART_ID, "AT+NAME3piRobot\r\n");
    uart_puts(UART_ID, "AT+UUID0xFFE0\r\n");
    uart_puts(UART_ID, "AT+CHAR0xFFE1\r\n");
    uart_puts(UART_ID, "AT+NOTI1\r\n");
}

int main() {
    stdio_init_all();
    init_ble();
    
    while(true) {
        while(uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            uart_putc(UART_ID, c);
        }
        sleep_ms(10);
    }
}