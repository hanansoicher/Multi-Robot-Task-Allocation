/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <pololu_3pi_2040_robot.h>
#include "hardware/uart.h"
#include <string.h>
#include "imu.h"
#include <hardware/pio.h>
#include <quadrature_encoder.pio.h>
#include <ir_sensors.h>


/// \tag::hello_uart[]

#define UART_ID uart1
#define PIO_ID pio0

#define BAUD_RATE 9600
#define UART_TX_PIN 20
#define UART_RX_PIN 21
#define DATA_BITS 8
#define STOP_BITS 1
#define RESPONSE_TIMEOUT_MS 2000 // Timeout in milliseconds
#define MAX_RETRIES 2          // Maximum number of retries


// Encoder definitions 
#define RIGHT_ENCODER_AB 8
#define LEFT_ENCODER_AB 12
#define RIGHT_SM 0
#define LEFT_SM 1

#define TIMEOUT_COUNT 500 // Define a timeout period as a counter

// Define IMU related data structs
static imu_inst_t imu_instance;
axes_data_t acc_data;

void send_at_command(const char *command) {
    uart_puts(UART_ID, command); // Send the command string
    printf("Sent: %s\n", command);
}

void receive_response(char *buffer, size_t length) {
    size_t index = 0;
    size_t timeout_counter = 0;
    char display_buffer[160]; // Buffer to hold the combined message

    while (index < length - 1) { // Leave space for null terminator
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            buffer[index++] = c;

            // Temp jam
            // display_fill(0);
            // display_text(buffer, 0, 0, DISPLAY_NOW | 1);
            // display_show();

            timeout_counter = 0;
            if (c == '\n') break; // Stop at newline
        } else {
            display_fill(0);
            snprintf(display_buffer, sizeof(display_buffer), "pending: %d %d", index, length);

            display_text(display_buffer, 0, 0, DISPLAY_NOW | 1);
            display_show();
        }

        if (++timeout_counter > TIMEOUT_COUNT) {
            display_fill(0);
            display_text("Timeout", 0, 16, DISPLAY_NOW | 1);
            display_show();
            break;
        }
    }
    buffer[index] = '\0'; // Null-terminate the string

    display_fill(0);
    display_text("rr comp", 0, 16, DISPLAY_NOW | 1);
    display_show();

}

bool receive_response_with_timeout(char *buffer, size_t length, uint32_t timeout_ms) {
    size_t index = 0;
    uint64_t start_time = to_ms_since_boot(get_absolute_time());

    while (index < length - 1) { // Leave space for null terminator
        if (uart_is_readable(UART_ID)) {

            char c = uart_getc(UART_ID);
            if (c >= 32 && c <= 126) { // Printable ASCII range
                buffer[index++] = c;
            }

            if (c == '\n') break; // Stop at newline
        } else {
            uint64_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
            if (elapsed_time >= timeout_ms) {
                return false; // Timeout
            }
        }
    }
    buffer[index] = '\0'; // Null-terminate the string
    return true; // Successfully received a response
}

bool send_and_receive(const char *command, char *response, size_t response_length, int max_retries) {
    int retries = 0;

    while (retries < max_retries) {
        send_at_command(command);

        if (receive_response_with_timeout(response, response_length, RESPONSE_TIMEOUT_MS)) {
            // Successfully received a response
            display_fill(0);
            display_text(response, 0, 32, DISPLAY_NOW | 1);
            display_text("SUCCESS", 0, 48, DISPLAY_NOW | 1);
            display_show();
            return true;
        }

        // Retry logic
        retries++;
        char retry_msg[64];
        snprintf(retry_msg, sizeof(retry_msg), "Retrying %d/%d", retries, max_retries);
        display_fill(0);
        display_text(command, 0, 32, DISPLAY_NOW | 1);
        display_text(retry_msg, 0, 48, DISPLAY_NOW | 1);
        display_show();
    }

    // Exceeded max retries
    display_fill(0);
    char display_buffer[64];
    snprintf(display_buffer, sizeof(display_buffer), "FAIL, got %s", response);
    display_text(display_buffer, 0, 48, DISPLAY_NOW | 1);
    display_show();
    return false;
}


void setup_uart() {
    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

bool prefix(const char *pre, const char *str)
{
    return strncmp(str, pre, strlen(pre)) == 0;
}

int extract_integer_from_string(const char *str) {
    const char *plus_sign = strchr(str, '+');
    if (!plus_sign || *(plus_sign + 1) == '\0') {
        // No '+' found or no characters after '+'
        display_text("Error: Invalid string format", 0, 32, DISPLAY_NOW | 1);
        display_show();
        return 0;
    }

    // Read from the character after '+' to the null terminator
    const char *start = plus_sign + 1;
    char *end;

    // Convert the substring to an integer
    int result = strtol(start, &end, 10);

    if (*end != '\0') {
        // Non-numeric characters found after the number
        display_text("Warning: Additional characters after number", 0, 32, DISPLAY_NOW | 1);
        display_show();
    }

    return result;
}

double extract_double_from_string(const char *str) {
    const char *plus_sign = strchr(str, '+');
    if (!plus_sign || *(plus_sign + 1) == '\0') {
        // No '+' found or no characters after '+'
        fprintf(stderr, "Error: Invalid string format\n");
        return 0.0;
    }

    // Read from the character after '+' to the null terminator
    const char *start = plus_sign + 1;
    char *end;

    // Convert the substring to a double
    double result = strtod(start, &end);

    if (*end != '\0') {
        // Non-numeric characters found after the number
        fprintf(stderr, "Warning: Additional characters after number\n");
    }

    return result;
}

// Angle calculations
int32_t MOTOR_SPEED = 1000;
float angle = 0.0f;
float last_gyro = 0.0f;
uint64_t last_time_read_angle; 

// Distance calculatoins
float traveled_distance = 0.0f;


void update() {

    uint64_t current_time_us = time_us_64();

    imu_read_gyro(&imu_instance, &acc_data);
    float cur_gyro = acc_data.z;

    angle += -(cur_gyro + last_gyro) * ((current_time_us - last_time_read_angle) / 1000000.0) / 2.0;

    last_gyro = cur_gyro;
    last_time_read_angle = current_time_us;
}

void handle_command(const char *command) {
    char return_data [16];
    if(prefix("STATUS+", command)) {
        int which_status = extract_integer_from_string(command);
        uart_puts(UART_ID, "RUNNING\n");
        uart_puts(UART_ID, "COMPLETED\n");
    }
    else if (prefix("MOVE+", command)) {
        double dist = extract_double_from_string(command);

        snprintf(display_buffer, sizeof(display_buffer), "RUNNING: %s dist=%d", command, dist);
        display_text(display_buffer, 0, 32, DISPLAY_NOW | 1);
        display_show();

        // Initialize distance tracking
        float total_distance = 0.0f;

        imu_read_gyro(&imu_instance, &acc_data);
        float initial_gyro = acc_data.z;
        float cur_gyro = acc_data.z;
        float total_accumulated_angle = 0.0f;

        int32_t rcount = -quadrature_encoder_get_count(pio0, RIGHT_SM);
        int32_t lcount = -quadrature_encoder_get_count(pio0, LEFT_SM);
        int32_t start_count = (rcount + lcount) / 2;
        int32_t current_count = start_count;

        // PID control variables
        const float Kp = 2.0f;    // Proportional gain 
        const float Ki = 0.01f;   // Integral gain
        const float Kd = 4.0f;    // Derivative gain
        
        float integral = 0.0f;
        float last_error = 0.0f;
        float target_angle = 0.0f;  // Want to go straight
        
        // Timing variables
        uint32_t last_time = to_ms_since_boot(get_absolute_time());
        float current_angle = 0.0f;

        int dist_sign = (dist >= 0) ? 1 : -1;

        while (abs(current_count - start_count) < (500 * abs(dist))) {
            // Calculate dt
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            float dt = (current_time - last_time) / 1000.0f;  // Convert to seconds
            last_time = current_time;

            imu_read_gyro(&imu_instance, &acc_data);
            current_angle += acc_data.z * dt;  // Integrate gyro data
            
            // PID calculations
            float error = target_angle - current_angle;
            integral = integral + (error * dt);
            float derivative = (error - last_error) / dt;
            
            // Limit integral windup
            integral = fmax(fmin(integral, 10.0f), -10.0f);
            
            float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
            
            float angle_sign = error
            // Apply correction to motor speeds
            int left_speed = (MOTOR_SPEED * dist_sign) + correction;
            int right_speed = (MOTOR_SPEED * dist_sign) - correction;
            
            // Limit motor speeds
            left_speed = fmax(fmin(left_speed, 100), -100);
            right_speed = fmax(fmin(right_speed, 100), -100);
            
            motors_set_speeds(left_speed, right_speed);
            
            // Save error for next iteration
            last_error = error;

            // Recompute encoder counts
            rcount = -quadrature_encoder_get_count(pio0, RIGHT_SM);
            lcount = -quadrature_encoder_get_count(pio0, LEFT_SM);
            current_count = (rcount + lcount) / 2;

            display_fill(0);
            snprintf(display_buffer, sizeof(display_buffer), "Ang:%.2f Cor:%.2f", 
                    current_angle, correction);
            display_text(display_buffer, 0, 16, DISPLAY_NOW | 1);
            display_show();

            sleep_ms(10);  // 100Hz update rate
        }

        // Update total distance traveled
        total_distance += abs(current_count - start_count);

        motors_set_speeds(0, 0);
        uart_puts(UART_ID, "COMPLETED\n");
    }
    else if(prefix("STATUS+", command)) {
        uart_puts(UART_ID, "COMPLETED\n");
    } 
    else if(prefix("SPEED+", command)) {
        int spd = extract_integer_from_string(command);
        MOTOR_SPEED = spd;
        uart_puts(UART_ID, "COMPLETED\n");
    }
    else if(prefix("ENCODER+", command)) {
        int which_sensor = extract_integer_from_string(command);

        axes_data_t encoder_data;
        
        int32_t rcount = -quadrature_encoder_get_count(pio0, RIGHT_SM);
        int32_t lcount = -quadrature_encoder_get_count(pio0, LEFT_SM);

        if(which_sensor == 0) {
            snprintf(return_data, sizeof(return_data), "%d\n", lcount);
            uart_puts(UART_ID, return_data);
        }
        else if(which_sensor == 1) {
            snprintf(return_data, sizeof(return_data), "%d\n", rcount);
            uart_puts(UART_ID, return_data);
        }

        /** Total Distance computations */
        else if(which_sensor == 2) {
            snprintf(return_data, sizeof(return_data), "%.2f\n", traveled_distance);
            uart_puts(UART_ID, return_data);
        }

        else if(which_sensor == 3) {
            traveled_distance = 0.0f;
            uart_puts(UART_ID, return_data);
        }

        else {
            uart_puts(UART_ID, "UNRECOGNIZED\n");
        }
        uart_puts(UART_ID, "COMPLETED\n");
    }
    else if(prefix("ANGLE+", command)) {
        int which_sensor = extract_integer_from_string(command);
        
        if(which_sensor == 0) {
            snprintf(return_data, sizeof(return_data), "%.2f\n", angle);
            uart_puts(UART_ID, return_data);
        }
        
        // Reset the anglel
        else if(which_sensor == 1) {
            last_gyro = 0;
            last_time_read_angle = time_us_64();
            angle = 0;
        }

        else {
            uart_puts(UART_ID, "UNRECOGNIZED\n");
        }

        uart_puts(UART_ID, "COMPLETED\n");
    }
    else if(prefix("GYRO+", command)) {
        int which_sensor = extract_integer_from_string(command);
        
        axes_data_t acc_data;
        
        imu_read_gyro(&imu_instance, &acc_data);
        float cur_gyro = acc_data.z;
        
        if(which_sensor == 0) {
            snprintf(return_data, sizeof(return_data), "%.2f\n", cur_gyro);
            uart_puts(UART_ID, return_data);
        }

        else {
            uart_puts(UART_ID, "UNRECOGNIZED\n");
        }

        uart_puts(UART_ID, "COMPLETED\n");
    }
    else if(prefix("BUMP+", command)) {
        int which_sensor = extract_integer_from_string(command);
        
        if(which_sensor == 0) {
            if(bump_sensor_left_is_pressed()) {
                uart_puts(UART_ID, "TRUE\n");
            }
            else {
                uart_puts(UART_ID, "FALSE\n");
            }
        }

        else if(which_sensor == 1) {
            if(bump_sensor_right_is_pressed()) {
                uart_puts(UART_ID, "TRUE\n");
            }
            else {
                uart_puts(UART_ID, "FALSE\n");
            }
        }

        else {
            uart_puts(UART_ID, "UNRECOGNIZED\n");
        }
        uart_puts(UART_ID, "COMPLETED\n");
    }
    else if(prefix("TURN+", command)) {

        axes_data_t gyro_data;
        float turn_angle = (float) extract_double_from_string(command);

        snprintf(display_buffer, sizeof(display_buffer), "TURNING: %s dist=%.2f", command, turn_angle);
        display_text(display_buffer, 0, 32, DISPLAY_NOW | 1);
        display_show();

        // Initialize distance tracking
        imu_read_gyro(&imu_instance, &acc_data);
        float cur_gyro = acc_data.z;
        float initial_gyro = acc_data.z;

        int dist_sign = (int)(turn_angle / abs(turn_angle));
        motors_set_speeds(dist_sign * MOTOR_SPEED, -dist_sign * MOTOR_SPEED);
        float total_accumulated_angle = 0.0f;

        // while(total_accumulated_angle < turn_angle) {
        int sleep_time = 20;
        while(abs(total_accumulated_angle) < abs(turn_angle)) {
            sleep_ms(20);

            // Recompute encoder counts
            imu_read_gyro(&imu_instance, &acc_data);

            total_accumulated_angle += -(acc_data.z + cur_gyro) * (sleep_time / 1000.0) / 2.0;
            angle += -(acc_data.z + cur_gyro) * (sleep_time / 1000.0) / 2.0;

            cur_gyro = acc_data.z;

            display_fill(0);
            snprintf(display_buffer, sizeof(display_buffer), "%.2f", total_accumulated_angle);
            display_text(display_buffer, 0, 16, DISPLAY_NOW | 1);
            display_show();
        }

        last_gyro = cur_gyro;
        last_time_read_angle = time_us_64();
        motors_set_speeds(0, 0);
        uart_puts(UART_ID, "COMPLETED\n");
    } 
    
    else {

        // Create the combined message
        snprintf(display_buffer, sizeof(display_buffer), "failed: %s", command);
        display_text(display_buffer, 0, 32, DISPLAY_NOW | 1);
        display_show();
        uart_puts(UART_ID, "FAILED\n");
        
    }
}

void configure_hm10() {
    char response[32];
    char display_buffer[160]; // Buffer to hold the combined message

    // Send configuration commands
    last_time_read_angle = time_us_64();

    // Ping the module
    send_and_receive("AT", response, 3, MAX_RETRIES);

    // Force a reset of the state
    send_and_receive("AT+RESET", response, 9, MAX_RETRIES);

    // Force BAUD rate to be 9600 
    send_and_receive("AT+BAUD0", response, 9, MAX_RETRIES); 

    // Configure notification settings
    send_and_receive("AT+NOTI0", response, 9, MAX_RETRIES);

    // Set mode to Transmission Mode (Mode 0)
    send_and_receive("AT+MODE0", response, 9, MAX_RETRIES);

    // Control the advertising interval
    send_and_receive("AT+NAME149_Proj_G0", response, 22, MAX_RETRIES);
    send_and_receive("AT+NAME149_Proj_G8", response, 22, MAX_RETRIES);

    display_text("DONE.", 0, 32, DISPLAY_NOW | 1);
    display_show();
}

int main() {
    // Set up our UART with the required speed.
    display_init();
    setup_uart();
    motors_init();
    bump_sensors_calibrate();

    // Initialize the IMU
    imu_init(&imu_instance);

    PIO pio = PIO_ID;
    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, RIGHT_SM, RIGHT_ENCODER_AB, 0);
    quadrature_encoder_program_init(pio, LEFT_SM, LEFT_ENCODER_AB, 0);

    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART

    // Send out a character without any conversions


    configure_hm10();

    /** Receiver loop */
    char response[32];
    char display_buffer[160]; // Buffer to hold the combined message

    display_fill(0);
    display_text("RECEIVING", 0, 0, DISPLAY_NOW | 1);
    display_show();
    sleep_ms(3000);

    while (1) {
        // Check if data is available on UART
        
        update();

        if (uart_is_readable(UART_ID)) {

            receive_response(response, sizeof(response)); 

            handle_command(response);

            // Create the combined message
            snprintf(display_buffer, sizeof(display_buffer), "exec: %s", response);
            display_fill(0);
            display_text(display_buffer, 0, 48, DISPLAY_NOW | 1);
            display_show();

            sleep_ms(1000);
        } else {
            display_fill(0);
            display_text("Waiting ...", 0, 16, DISPLAY_NOW | 1);
            display_show();
            sleep_ms(1000);
        }
        
    }

    return 0;
}

/// \end::hello_uart[]