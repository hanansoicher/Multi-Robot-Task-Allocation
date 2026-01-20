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
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Robot is a pololu 3pi+ RP2040 standard edition which is 96 mm wide (including wheels which are 6.8mm wide and 32mm diameter) and 97mm long

#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 28
#define UART_RX_PIN 29

#define PIO_ID pio0
#define RIGHT_ENCODER_AB 8
#define LEFT_ENCODER_AB 12
#define RIGHT_SM 0
#define LEFT_SM 1

#define MOVE_MOTOR_SPEED 800
#define TURN_MOTOR_SPEED 500
#define MOVE_DISTANCE 15

#define ENCODER_TICKS_PER_CM 35
#define ENCODER_TICKS_PER_DEGREE 2.625f


typedef struct {
    float current_angle;
    imu_inst_t imu;
    uint64_t last_gyro_read_time;
    float last_gyro_value;
    float last_left_encoder;
    float last_right_encoder;
} RobotState;

static RobotState robot_state = {
    .current_angle = 0.0f,
    .last_gyro_read_time = 0,
    .last_gyro_value = 0.0f,
    .last_left_encoder = 0.0f,
    .last_right_encoder = 0.0f,
};

static float gyro_bias = 0.29f; // usually 0.29-0.32
int32_t black_tape_threshold = 700;  // Threshold for detecting black tape (0-1000)
#define LINE_CORRECTION_STRENGTH 400
int8_t last_direction_off = 0; // -1 if left, 1 if right. In case correction wasn't strong enough and both sides lose the tape

void update_robot_state() {
    uint64_t current_time = time_us_64();
    float dt = (current_time - robot_state.last_gyro_read_time) / 1000000.0f;
    
    axes_data_t gyro_data;
    imu_read_gyro(&robot_state.imu, &gyro_data);

    float gyro_value = gyro_data.z - gyro_bias;   // in °/s
    
    if (fabs(gyro_value) > 0.5f) {  // Only integrate if movement > 0.5°/s
        // trapezoidal integration
        robot_state.current_angle += -(gyro_value + robot_state.last_gyro_value) * dt / 2.0f;
    }

    robot_state.last_gyro_value = gyro_value;
    robot_state.last_gyro_read_time = current_time;

    robot_state.last_right_encoder = quadrature_encoder_get_count(PIO_ID, RIGHT_SM);
    robot_state.last_left_encoder = quadrature_encoder_get_count(PIO_ID, LEFT_SM);

    robot_state.last_right_encoder = -robot_state.last_right_encoder;
    robot_state.last_left_encoder = -robot_state.last_left_encoder;
}


void calibrate_line_white() {
    line_sensors_reset_calibration();
    line_sensors_calibrate();

    char buf[128];
    snprintf(buf, sizeof(buf), "W:min=%u,%u,%u max=%u,%u,%u\n", 
            line_sensors_cal_min[1], line_sensors_cal_min[2], line_sensors_cal_min[3],
            line_sensors_cal_max[1], line_sensors_cal_max[2], line_sensors_cal_max[3]);
    uart_puts(UART_ID, buf);
}

void calibrate_line_black() {
    line_sensors_calibrate();

    char buf[128];
    snprintf(buf, sizeof(buf), "B:min=%u,%u,%u max=%u,%u,%u\n", 
            line_sensors_cal_min[1], line_sensors_cal_min[2], line_sensors_cal_min[3],
            line_sensors_cal_max[1], line_sensors_cal_max[2], line_sensors_cal_max[3]);
    uart_puts(UART_ID, buf);

    int32_t white_avg = (line_sensors_cal_min[1] + line_sensors_cal_min[2] + line_sensors_cal_min[3]) / 3;
    int32_t black_avg = (line_sensors_cal_max[1] + line_sensors_cal_max[2] + line_sensors_cal_max[3]) / 3;
    black_tape_threshold = (white_avg + black_avg) / 2;
    
    char thresh_buf[64];
    snprintf(thresh_buf, sizeof(thresh_buf), "THRESHOLD: %u\n", black_tape_threshold);
    uart_puts(UART_ID, thresh_buf);
}

float check_line_position() { // Positive return value causes right turn
    line_sensors_read_calibrated();
    
    float left_sensor = line_sensors_calibrated[1];
    float center_sensor = line_sensors_calibrated[2];
    float right_sensor = line_sensors_calibrated[3];

    bool left_on_tape = left_sensor > black_tape_threshold;
    bool center_on_tape = center_sensor > black_tape_threshold;
    bool right_on_tape = right_sensor > black_tape_threshold;
    
    if (left_on_tape && !right_on_tape) {
        last_direction_off = 1;
        return -LINE_CORRECTION_STRENGTH; // turn left
    } else if (right_on_tape && !left_on_tape) {
        last_direction_off = -1;
        return LINE_CORRECTION_STRENGTH; // turn right
    } else if (!left_on_tape && !right_on_tape) {
        return -LINE_CORRECTION_STRENGTH * last_direction_off;
    } else {
        return 0;
    }
}

bool execute_move(int32_t distance) {
    uint64_t start_time = time_us_64();
    update_robot_state();
    int32_t target_ticks = abs(MOVE_DISTANCE * (ENCODER_TICKS_PER_CM)-10); // subtract 10 because the wheels overshoot by 10 ticks when stopping
    int32_t start_right = robot_state.last_right_encoder;
    int32_t start_left = robot_state.last_left_encoder;

    while (1) {
        update_robot_state();
        int32_t left_ticks = robot_state.last_left_encoder - start_left;
        int32_t right_ticks = robot_state.last_right_encoder - start_right;

        if (abs(left_ticks) + abs(right_ticks) >= 2*target_ticks) {
            motors_set_speeds(0, 0);
            return true;
        }
        float line_error = check_line_position(); 
        int32_t left_speed = MOVE_MOTOR_SPEED + line_error;
        int32_t right_speed = MOVE_MOTOR_SPEED - line_error;

        left_speed = fmax(fmin(left_speed, MOVE_MOTOR_SPEED*2), 200);
        right_speed = fmax(fmin(right_speed, MOVE_MOTOR_SPEED*2), 200);
        motors_set_speeds(left_speed, right_speed);
        sleep_ms(1);
    }
}


bool execute_turn(char direction, int32_t angle) {
    update_robot_state();
    uint64_t start_time = time_us_64();
    int dir = (direction == 'L') ? 1 : -1;

    int32_t start_left = robot_state.last_left_encoder;
    int32_t start_right = robot_state.last_right_encoder;

    int32_t target_ticks = abs(angle * ENCODER_TICKS_PER_DEGREE);

    int32_t left_speed = -TURN_MOTOR_SPEED * dir;
    int32_t right_speed = TURN_MOTOR_SPEED * dir;

    motors_set_speeds(left_speed, right_speed);
    while (1) {
        update_robot_state();
        int32_t left_ticks = abs(robot_state.last_left_encoder - start_left);
        int32_t right_ticks = abs(robot_state.last_right_encoder - start_right);
        
        if (left_ticks >= target_ticks || right_ticks >= target_ticks) {
            motors_set_speeds(0, 0);
            update_robot_state();
            sleep_ms(10);

            uint64_t turn_duration = time_us_64() - start_time;
            uint64_t target_duration = 1900000; // 1900ms in microseconds (time to move 15cm), might be unnecessary if the Coordinator is synchronizing time steps
            if (turn_duration < target_duration) {
                uint64_t wait_time = target_duration - turn_duration;
                sleep_ms((uint32_t)(wait_time / 1000));
            }
            
            return true;
        }
        sleep_ms(1);
    }
}

bool execute_command(char command) {
    if (command == 'M') {
        return execute_move(15);
    }
    else if (command == 'R') {
        return execute_turn('R', 90);
    }
    else if (command == 'L') {
        return execute_turn('L', 90);
    }
    else if (command == 'W') {
        motors_set_speeds(0, 0);
        sleep_ms(1900);
        return true;
    }
}


static char command_queue[100];
static int command_queue_size = 0;
static int current_command_index = 0;

bool handle_command(const char* command) {
    char response[128];
    
    if (strncmp(command, "MOVE+", 4) == 0) {
        execute_move(atoi(command + 4));
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "RIGHT+", 6) == 0) {
        execute_turn('R', atoi(command + 6));
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "LEFT+", 5) == 0) {
        execute_turn('L', atoi(command + 5));
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "WAIT", 4) == 0) {
        motors_set_speeds(0, 0);
        sleep_ms(1900);
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "CMDS+", 5) == 0) {
        const char* cmds = command + 5;
        while (*cmds && command_queue_size < sizeof(command_queue)) {
            command_queue[command_queue_size++] = *cmds++;
        }
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "EXEC", 4) == 0) {
        if (current_command_index < command_queue_size) {
            execute_command(command_queue[current_command_index++]);
        } else {
            memset(command_queue, 0, sizeof(command_queue));
            command_queue_size = 0;
            current_command_index = 0;
        }
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "GET_QUEUE", 8) == 0) {
        char buf[128];
        snprintf(buf, sizeof(buf), "QUEUE,%s\n", command_queue);
        uart_puts(UART_ID, buf);
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "CLEAR", 5) == 0) {
        memset(command_queue, 0, sizeof(command_queue));
        command_queue_size = 0;
        current_command_index = 0;
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "STATUS", 6) == 0) {
        update_robot_state();
        char buf[64];
        snprintf(buf, sizeof(buf), "ENC,%ld,%ld,ANG,%.2f\n",
                (long)robot_state.last_left_encoder,
                (long)robot_state.last_right_encoder,
                robot_state.current_angle);
        uart_puts(UART_ID, buf);
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "CAL_LINE_WHITE", 14) == 0) {
        calibrate_line_white();
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "CAL_LINE_BLACK", 14) == 0) {
        calibrate_line_black();
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "LINE_STATUS", 11) == 0) {
        line_sensors_read_calibrated();
        char buf[128];
        snprintf(buf, sizeof(buf), "IR_STATUS,%u,%u,%u\n",
                line_sensors_calibrated[1], 
                line_sensors_calibrated[2], line_sensors_calibrated[3]);
        uart_puts(UART_ID, buf);
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    else if (strncmp(command, "RESET_LINE_CAL", 14) == 0) {
        // Reset line sensor calibration
        line_sensors_reset_calibration();
        uart_puts(UART_ID, "Line sensor calibration reset\n");
        uart_puts(UART_ID, "COMPLETED\n");
        return true;
    }

    // else if (strncmp(command, "CALIBRATE_GYRO", 14) == 0) {
    //     gyro_bias = calibrate_bias();
    //     char gyro_buf[32];
    //     snprintf(gyro_buf, sizeof(gyro_buf), "GYRO,%.2f\n", gyro_bias);
    //     uart_puts(UART_ID, gyro_buf);
    //     uart_puts(UART_ID, "COMPLETED\n");
    //     return true;
    // }

    uart_puts(UART_ID, "FAILED\n");
    return false;
}



int main() {
    stdio_init_all();
    uart_init(UART_ID, BAUD_RATE);

    uart_set_format(UART_ID, 8, 1, 0);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    PIO pio = PIO_ID;
    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, RIGHT_SM, RIGHT_ENCODER_AB, 0);
    quadrature_encoder_program_init(pio, LEFT_SM, LEFT_ENCODER_AB, 0);

    motors_init();
    imu_init(&robot_state.imu);
    sleep_ms(1000);

    line_sensors_reset_calibration();
    line_sensors_start_read();  // Initialize the PIO state machine
    sleep_ms(100);  // Give time for initialization

    robot_state.last_gyro_read_time = time_us_64();
    
    update_robot_state();
    
    char command_buffer[128];
    int buffer_pos = 0;
    while (1) {
        while (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            if (c == '\r' || c == '\n') {
                if (buffer_pos > 0) {
                    command_buffer[buffer_pos] = '\0';
                    handle_command(command_buffer);
                    memset(command_buffer, 0, sizeof(command_buffer));
                    buffer_pos = 0;
                }
            } else if (buffer_pos < sizeof(command_buffer) - 1) {
                command_buffer[buffer_pos++] = c;
            }
        }
        sleep_ms(1); 
    }
    return 0;
}