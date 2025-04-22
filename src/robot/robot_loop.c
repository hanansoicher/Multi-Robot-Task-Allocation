#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pico/stdlib.h>
#include <pololu_3pi_2040_robot.h>
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "imu.h"
#include <hardware/pio.h>
#include "quadrature_encoder.pio.h"
#include <math.h>
// cmake -G "Unix Makefiles" ..

#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 28
#define UART_RX_PIN 29

#define PIO_ID pio0
#define RIGHT_ENCODER_AB 8
#define LEFT_ENCODER_AB 12
#define RIGHT_SM 0
#define LEFT_SM 1

#define DEFAULT_MOTOR_SPEED 1000
#define ENCODER_TICKS_PER_CM 40 // Need to calibrate

typedef struct {
    float kp;        // Proportional gain
    float ki;        // Integral gain
    float kd;        // Derivative gain
    float integral;  // Accumulated error
    float last_error;
} PIDController;

void pid_init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float compute_pid_correction(PIDController *pid, float error, float dt) {
    pid->integral += error * dt;
    if (pid->integral > 10.0f) pid->integral = 10.0f;
    if (pid->integral < -10.0f) pid->integral = -10.0f;
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

static float gyro_bias = 0.0f;

typedef struct {
    float current_angle;
    float total_distance;
    imu_inst_t imu;
    uint64_t last_gyro_read_time;
    float last_gyro_value;
    float last_left_encoder;
    float last_right_encoder;
} RobotState;

static RobotState robot_state = {
    .current_angle = 0.0f,
    .total_distance = 0.0f,
    .last_gyro_read_time = 0,
    .last_gyro_value = 0.0f,
    .last_left_encoder = 0.0f,
    .last_right_encoder = 0.0f,
};

void update_robot_state() {
    uint64_t current_time = time_us_64();
    float dt = (current_time - robot_state.last_gyro_read_time) / 1000000.0f;
    
    axes_data_t gyro_data;
    imu_read_gyro(&robot_state.imu, &gyro_data);
    gyro_data.z -= gyro_bias;
    robot_state.current_angle += -(gyro_data.z + robot_state.last_gyro_value) * dt / 2.0f;
    robot_state.last_gyro_value = gyro_data.z;
    robot_state.last_gyro_read_time = current_time;
    
    robot_state.last_right_encoder = quadrature_encoder_get_count(PIO_ID, RIGHT_SM);
    robot_state.last_left_encoder = quadrature_encoder_get_count(PIO_ID, LEFT_SM);
}

bool execute_move(float distance_cm) {
    update_robot_state();
    PIDController pid;
    pid_init(&pid, 15.0f, 0.05f, 1.0f);
    float target_angle = robot_state.current_angle;
    int32_t target_ticks = (int32_t)(distance_cm * ENCODER_TICKS_PER_CM);
    int32_t start_right = quadrature_encoder_get_count(PIO_ID, RIGHT_SM);
    int32_t start_left = quadrature_encoder_get_count(PIO_ID, LEFT_SM);
    
    int direction = (distance_cm >= 0) ? 1 : -1;

    int32_t last_debug_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_update_time = to_ms_since_boot(get_absolute_time());
    
    printf("Starting move: distance=%.2f cm, target_ticks=%ld\n", distance_cm, target_ticks);
    
    while (1) {
        update_robot_state();
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        float dt = (current_time - last_update_time) / 1000.0f;
        if (dt < 0.001f) continue;
        last_update_time = current_time;
        int mean_diff = (robot_state.last_right_encoder - start_right + robot_state.last_left_encoder - start_left) / 2;
        if (abs(mean_diff) >= abs(target_ticks)) {
            motors_set_speeds(0, 0);
            printf("Move complete");
            return true;
        }
        
        float angle_error = target_angle - robot_state.current_angle;
        while (angle_error > 180.0f) angle_error -= 360.0f;
        while (angle_error < -180.0f) angle_error += 360.0f;
        float correction = compute_pid_correction(&pid, angle_error, dt);
        
        int32_t left_speed = (DEFAULT_MOTOR_SPEED * direction) + correction;
        int32_t right_speed = (DEFAULT_MOTOR_SPEED * direction) - correction;
        if (left_speed * direction < 200) left_speed = 200 * direction;
        if (right_speed * direction < 200) right_speed = 200 * direction;
        if (left_speed * direction > DEFAULT_MOTOR_SPEED*2) left_speed = DEFAULT_MOTOR_SPEED * direction;
        if (right_speed * direction > DEFAULT_MOTOR_SPEED*2) right_speed = DEFAULT_MOTOR_SPEED * direction;
        motors_set_speeds(left_speed, right_speed);

        if (current_time - last_debug_time > 1000) {
            last_debug_time = current_time;
            printf("Angle: %.2f, Error: %.2f, Corr: %.2f, L: %ld, R: %ld, Dist: %ld/%ld\n", robot_state.current_angle, angle_error, correction, left_speed, right_speed, target_ticks);
        }
        sleep_ms(5);
    }
}

bool execute_turn(float angle_degrees, char direction) {
    int dir = (direction == 'R') ? 1 : -1;
    bool close = false;
    int32_t last_debug_time = to_ms_since_boot(get_absolute_time());
    update_robot_state();
    float start_angle = robot_state.current_angle;
    float target_angle = start_angle + dir*angle_degrees;
    motors_set_speeds(dir*DEFAULT_MOTOR_SPEED, -dir*DEFAULT_MOTOR_SPEED);

    while (1) {
        update_robot_state();
        int32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_debug_time > 1000) {
            last_debug_time = current_time;
            printf("Angle: %.2f, Target: %.2f, Error: %.2f\n", robot_state.current_angle, target_angle, target_angle - robot_state.current_angle);
        }

        float angle_error = target_angle - robot_state.current_angle;
        while (angle_error > 720.0f) angle_error -= 360.0f;
        while (angle_error < -720.0f) angle_error += 360.0f;
        
        if (!close && fabs(angle_error) < 45.0f) {
            close = true;
            motors_set_speeds(dir*DEFAULT_MOTOR_SPEED / 3, -dir*DEFAULT_MOTOR_SPEED / 3);
        }

        if (fabs(angle_error) < 4.0f) {
            motors_set_speeds(0, 0);
            printf("Turn complete: current=%.2f, target=%.2f, remaining=%.2f\n", robot_state.current_angle, target_angle, angle_error);
            return true;
        }      
        sleep_ms(5);
    }
}

void calibrate_movement(float distance) {
    if (distance <= 0) distance = 50.0f;
    
    uint32_t start_time, end_time;
    int32_t start_left, start_right, end_left, end_right;
    float duration_ms;
    float ms_per_cm;
    float left_ticks, right_ticks, avg_ticks;
    float ticks_per_cm;
    char result[200];
    
    update_robot_state();
    start_left = quadrature_encoder_get_count(PIO_ID, LEFT_SM);
    start_right = quadrature_encoder_get_count(PIO_ID, RIGHT_SM);
    start_time = to_ms_since_boot(get_absolute_time());
    
    float start_angle = robot_state.current_angle;

    uint32_t estimated_drive_time = (uint32_t)(distance * 40.0f);
    
    sprintf(result, "Starting calibration: %.1fcm movement for ~%lums\n", distance, estimated_drive_time);
    uart_puts(UART_ID, result);
    
    PIDController pid;
    pid_init(&pid, 15.0f, 0.05f, 1.0f);
    
    motors_set_speeds(DEFAULT_MOTOR_SPEED, DEFAULT_MOTOR_SPEED);
    uint32_t start_drive_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_update_time = start_drive_time;
    uint32_t current_time;
    
    while (1) {
        update_robot_state();
        current_time = to_ms_since_boot(get_absolute_time());
        
        if (current_time - start_drive_time >= estimated_drive_time) {
            motors_set_speeds(0, 0);
            break;
        }
        
        float dt = (current_time - last_update_time) / 1000.0f;
        if (dt >= 0.01f) {
            last_update_time = current_time;
            
            float angle_error = start_angle - robot_state.current_angle;
            while (angle_error > 180.0f) angle_error -= 360.0f;
            while (angle_error < -180.0f) angle_error += 360.0f;
            
            float correction = compute_pid_correction(&pid, angle_error, dt);
            
            int32_t left_speed = DEFAULT_MOTOR_SPEED + correction;
            int32_t right_speed = DEFAULT_MOTOR_SPEED - correction;
            
            if (left_speed < 200) left_speed = 200;
            if (right_speed < 200) right_speed = 200;
            if (left_speed > DEFAULT_MOTOR_SPEED*2) left_speed = DEFAULT_MOTOR_SPEED*2;
            if (right_speed > DEFAULT_MOTOR_SPEED*2) right_speed = DEFAULT_MOTOR_SPEED*2;
            
            motors_set_speeds(left_speed, right_speed);
        }
        
        sleep_ms(5);
    }
    
    end_time = to_ms_since_boot(get_absolute_time());
    end_left = quadrature_encoder_get_count(PIO_ID, LEFT_SM);
    end_right = quadrature_encoder_get_count(PIO_ID, RIGHT_SM);
    
    duration_ms = (float)(end_time - start_time);
    ms_per_cm = duration_ms / distance;
    
    left_ticks = (float)abs(end_left - start_left);
    right_ticks = (float)abs(end_right - start_right);
    avg_ticks = (left_ticks + right_ticks) / 2.0f;
    ticks_per_cm = avg_ticks / distance;
    
    sprintf(result, "Res:\n"
                   "%.2f cm\n"
                   "%.2f ms\n"
                   "%.2f ms/cm\n"
                   "Left: %.1f\n"
                   "Right: %.1f\n"
                   "Avg: %.1f\n"
                   "Ticks/cm: %.2f\n",
                   distance, duration_ms, ms_per_cm,
                   left_ticks, right_ticks, avg_ticks, ticks_per_cm);
    
    uart_puts(UART_ID, result);
    uart_puts(UART_ID, "COMPLETED\n");
}

void calibrate_turn() {
    uint32_t start_time, slow_phase_start, end_time;
    float test_angle = 180.0f;
    bool entered_slow_phase = false;
    robot_state.current_angle = 0.0f;
    start_time = to_ms_since_boot(get_absolute_time());
    
    update_robot_state();
    float start_angle = robot_state.current_angle;
    float target_angle = start_angle + test_angle;
    int dir = 1; // right
    motors_set_speeds(dir*DEFAULT_MOTOR_SPEED, -dir*DEFAULT_MOTOR_SPEED);

    while (1) {
        update_robot_state();
        
        float angle_error = target_angle - robot_state.current_angle;
        float angle_covered = fabs(robot_state.current_angle - start_angle);
        
        if (!entered_slow_phase && fabs(angle_error) < 45.0f) {
            entered_slow_phase = true;
            slow_phase_start = to_ms_since_boot(get_absolute_time());
            motors_set_speeds(dir*DEFAULT_MOTOR_SPEED / 3, -dir*DEFAULT_MOTOR_SPEED / 3);
            
            float fast_phase_duration = slow_phase_start - start_time;
            float fast_phase_degrees = test_angle - 45.0f;
            float fast_deg_per_ms = fast_phase_degrees / fast_phase_duration;
            
            char result[100];
            sprintf(result, "Fast phase: %.2f ms for %.2f degrees (%.4f deg/ms)\n", 
                   fast_phase_duration, fast_phase_degrees, fast_deg_per_ms);
            uart_puts(UART_ID, result);
        }
        
        if (fabs(angle_error) < 2.0f) {
            motors_set_speeds(0, 0);
            end_time = to_ms_since_boot(get_absolute_time());
            
            float slow_phase_duration = end_time - slow_phase_start;
            float slow_phase_degrees = 45.0f;
            float slow_deg_per_ms = slow_phase_degrees / slow_phase_duration;
            
            char result[100];
            sprintf(result, "Slow phase: %.2f ms for %.2f degrees (%.4f deg/ms)\n", 
                   slow_phase_duration, slow_phase_degrees, slow_deg_per_ms);
            uart_puts(UART_ID, result);
            
            float total_duration = end_time - start_time;
            float total_deg_per_ms = test_angle / total_duration;
            
            sprintf(result, "Total: %.2f ms for %.2f degrees (%.4f deg/ms)\n", 
                   total_duration, test_angle, total_deg_per_ms);
            uart_puts(UART_ID, result);
            
            break;
        }
        sleep_ms(5);
    }
    uart_puts(UART_ID, "COMPLETED\n");
}


bool handle_command(const char* command) {
    char response[32];
    printf("Received command: %s\n", command);
    if (strncmp(command, "MOVE+", 5) == 0) {
        float distance = atof(command + 5);
        bool success = execute_move(distance);
        uart_puts(UART_ID, success ? "COMPLETED\n" : "FAILED\n");
        return success;
    }

    else if (strncmp(command, "RIGHT+", 6) == 0) {
        float angle = atof(command + 6);
        bool success = execute_turn(angle, 'R');
        uart_puts(UART_ID, success ? "COMPLETED\n" : "FAILED\n");
        return success;
    }

    else if (strncmp(command, "LEFT+", 5) == 0) {
        float angle = atof(command + 5);
        bool success = execute_turn(angle, 'L');
        uart_puts(UART_ID, success ? "COMPLETED\n" : "FAILED\n");
        return success;
    }

    else if (strncmp(command, "WAIT+", 5) == 0) {
        int wait_time = atoi(command + 5);
        motors_set_speeds(0, 0);
        sleep_ms(wait_time);
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }
    
    else if (strncmp(command, "STATUS", 6) == 0) {
        uart_puts(UART_ID, "RUNNING\n");
        return true;
    }

    else if (strncmp(command, "CALIBRATE_MOVE+", 15) == 0) {
        int distance = atoi(command + 15);
        calibrate_movement(distance);
        return true;
    }
    else if (strncmp(command, "CALIBRATE_TURN", 14) == 0) {
        calibrate_turn();
        return true;
    }
    
    uart_puts(UART_ID, "FAILED\n");
    return false;
}

int main() {
    stdio_init_all();
    uart_init(UART_ID, BAUD_RATE);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    PIO pio = PIO_ID;
    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, RIGHT_SM, RIGHT_ENCODER_AB, 0);
    quadrature_encoder_program_init(pio, LEFT_SM, LEFT_ENCODER_AB, 0);

    motors_init();
    imu_init(&robot_state.imu);

    robot_state.last_gyro_read_time = time_us_64();

    char command_buffer[32];
    int buffer_pos = 0;
    while (1) {
        update_robot_state();
        while (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            printf("Received char: %c (%d)\n", c, (int)c);
            if (c == '\r' || c == '\n') {
                if (buffer_pos > 0) {
                    command_buffer[buffer_pos] = '\0';
                    printf("Processing command: %s\n", command_buffer);
                    handle_command(command_buffer);
                    buffer_pos = 0;
                }
                continue;
            } else if (buffer_pos < sizeof(command_buffer) - 1) {
                command_buffer[buffer_pos++] = c;
            }
        }
        sleep_ms(10); 
    }
    return 0;
}