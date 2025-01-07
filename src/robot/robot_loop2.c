#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pico/stdlib.h>
#include <pololu_3pi_2040_robot.h>
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "imu.h"
#include "quadrature_encoder.pio.h"
#include "ir_sensors.h"

#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 20
#define UART_RX_PIN 21
#define DATA_BITS 8
#define STOP_BITS 1

#define PIO_ID pio0
#define RIGHT_ENCODER_AB 8
#define LEFT_ENCODER_AB 12
#define RIGHT_SM 0
#define LEFT_SM 1

#define DEFAULT_MOTOR_SPEED 1000
#define PID_UPDATE_RATE_MS 10
#define ENCODER_TICKS_PER_CM 500 // Need to calibrate

typedef struct {
    float current_angle;
    float total_distance;
    int32_t motor_speed;
    imu_inst_t imu;
    uint64_t last_gyro_read_time;
    float last_gyro_value;
} RobotState;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float target;
} PIDController;

static RobotState robot_state = {
    .current_angle = 0.0f,
    .total_distance = 0.0f,
    .motor_speed = DEFAULT_MOTOR_SPEED,
    .last_gyro_read_time = 0,
    .last_gyro_value = 0.0f
};

int main() {
    init_hardware();
    char command_buffer[32];
    int buffer_pos = 0;
    
    while (1) {
        update_robot_state();
        while (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            if (c == '\n' || c == '\r') {
                if (buffer_pos > 0) {
                    command_buffer[buffer_pos] = '\0';
                    handle_command(command_buffer);
                    buffer_pos = 0;
                }
            } else if (buffer_pos < sizeof(command_buffer) - 1) {
                command_buffer[buffer_pos++] = c;
            }
        }
        sleep_ms(10); 
    }
    return 0;
}

void init_hardware(void) {
    stdio_init_all();
    display_init();
    init_uart();
    motors_init();
    bump_sensors_calibrate();
    
    imu_init(&robot_state.imu);
    robot_state.last_gyro_read_time = time_us_64();
    
    init_encoders();
}

void init_uart(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void init_encoders(void) {
    PIO pio = PIO_ID;
    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, RIGHT_SM, RIGHT_ENCODER_AB, 0);
    quadrature_encoder_program_init(pio, LEFT_SM, LEFT_ENCODER_AB, 0);
}

void update_robot_state(void) {
    uint64_t current_time = time_us_64();
    float dt = (current_time - robot_state.last_gyro_read_time) / 1000000.0f;
    
    axes_data_t gyro_data;
    imu_read_gyro(&robot_state.imu, &gyro_data);
    
    // Update angle using trapezoidal integration
    robot_state.current_angle += -(gyro_data.z + robot_state.last_gyro_value) * dt / 2.0f;
    
    robot_state.last_gyro_value = gyro_data.z;
    robot_state.last_gyro_read_time = current_time;
}

bool execute_move(float distance_cm) {
    PIDController pid;
    init_pid_controller(&pid, 2.0f, 0.01f, 4.0f);
    
    int32_t target_ticks = (int32_t)(distance_cm * ENCODER_TICKS_PER_CM);
    int32_t start_count = (quadrature_encoder_get_count(PIO_ID, RIGHT_SM) + quadrature_encoder_get_count(PIO_ID, LEFT_SM)) / -2;
    
    uint32_t last_time = to_ms_since_boot(get_absolute_time());
    float target_angle = robot_state.current_angle;  // Maintain current orientation
    
    while (1) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float dt = (current_time - last_time) / 1000.0f;
        last_time = current_time;
        
        int32_t current_count = (quadrature_encoder_get_count(PIO_ID, RIGHT_SM) + quadrature_encoder_get_count(PIO_ID, LEFT_SM)) / -2;
        int32_t distance_ticks = current_count - start_count;
        
        if (abs(distance_ticks) >= abs(target_ticks)) {
            stop_motors();
            return true;
        }
        
        float correction = compute_pid(&pid, robot_state.current_angle - target_angle, dt);
        
        int direction = (distance_cm >= 0) ? 1 : -1;
        int left_speed = (robot_state.motor_speed * direction) + correction;
        int right_speed = (robot_state.motor_speed * direction) - correction;
        
        left_speed = fmax(fmin(left_speed, robot_state.motor_speed), -robot_state.motor_speed);
        right_speed = fmax(fmin(right_speed, robot_state.motor_speed), -robot_state.motor_speed);
        
        motors_set_speeds(left_speed, right_speed);
        sleep_ms(PID_UPDATE_RATE_MS);
    }
}

bool execute_turn(float angle_degrees) {
    float start_angle = robot_state.current_angle;
    float target_angle = start_angle + angle_degrees;
    int direction = (angle_degrees >= 0) ? 1 : -1;
    
    while (fabs(robot_state.current_angle - target_angle) > 1.0f) {
        motors_set_speeds(direction * robot_state.motor_speed, 
                         -direction * robot_state.motor_speed);
        sleep_ms(PID_UPDATE_RATE_MS);
    }
    
    stop_motors();
    return true;
}

void stop_motors(void) {
    motors_set_speeds(0, 0);
}

bool handle_command(const char* command) {
    char response[32];
    
    if (strncmp(command, "MOVE+", 5) == 0) {
        float distance = atof(command + 5);
        bool success = execute_move(distance);
        uart_puts(UART_ID, success ? "COMPLETED\n" : "FAILED\n");
        return success;
    }
    
    else if (strncmp(command, "TURN+", 5) == 0) {
        float angle = atof(command + 5);
        bool success = execute_turn(angle);
        uart_puts(UART_ID, success ? "COMPLETED\n" : "FAILED\n");
        return success;
    }

    else if (strncmp(command, "WAIT+", 5) == 0) {
        int wait_time = atoi(command + 5);
        sleep_ms(wait_time);
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }
    
    else if (strncmp(command, "SPEED+", 6) == 0) {
        robot_state.motor_speed = atoi(command + 6);
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }
    
    else if (strncmp(command, "STATUS", 6) == 0) {
        uart_puts(UART_ID, "RUNNING\n");
        return true;
    }
    
    uart_puts(UART_ID, "FAILED\n");
    return false;
}

void init_pid_controller(PIDController* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->target = 0.0f;
}

float compute_pid(PIDController* pid, float error, float dt) {
    // Integrate error
    pid->integral += error * dt;
    
    // Limit integral windup
    pid->integral = fmax(fmin(pid->integral, 10.0f), -10.0f);
    
    // Compute derivative
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;
    
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}