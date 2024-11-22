#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "../tasks.h"

static TaskAssignment current_task;
static char receive_buffer[1024];
static int buffer_index = 0;

InstructionType char_to_instruction(char c) {
    switch(c) {
        case 'F': return MOVE_FORWARD;
        case 'L': return TURN_LEFT;
        case 'R': return TURN_RIGHT;
        case 'W': return MARK_WAYPOINT;
        case 'E': return END_TASK;
        default: return END_TASK;
    }
}

void send_status(const char* status) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "STATUS:%s\r\n", status);
    uart_puts(uart0, buffer);
}

// Process byte from UART Bluetooth module
bool process_incoming_byte(char byte) {
    if (byte == '<') {
        buffer_index = 0;
        memset(receive_buffer, 0, 1024);
        return false;
    }
    
    if (byte == '>') {
        receive_buffer[buffer_index] = '\0';
        return parse_instructions();
    }
    
    if (buffer_index < 1023) {
        receive_buffer[buffer_index++] = byte;
    }
    
    return false;
}

// 'F(DISTANCE)' move forward DISTANCE mm
// 'L(ANGLE)' turn left
// 'R(ANGLE)' turn right
// 'W' mark waypoint (do 360 degree spin)
// 'S' emergency stop
// '<' start of task assignment
// '>' end of task assignment

// <F(10)R(90)W>

// Parse task message format: "TASK_ID:INSTRUCTIONS"
bool parse_instructions(void) {
    int index = 0;
    int buffer_len = strlen(receive_buffer);
    int instr_count = 0;
    
    while (index < buffer_len && instr_count < 100) {
        char current = receive_buffer[index];
        
        switch (current) {
            case '<':
                current_task.task_id = atoi(receive_buffer + index + 1);
                instr_count = 0;
                break;
            case 'F': 
                // Extract distance from parentheses
                if (index + 1 >= buffer_len || receive_buffer[index + 1] != '(') {
                    return false;
                }
                
                char distance_str[10] = 0;
                int dist_index = 0;
                index += 2;  // Skip 'F('
                
                while (index < buffer_len && receive_buffer[index] != ')' && dist_index < 9) {
                    distance_str[dist_index++] = receive_buffer[index++];
                }
                
                if (index >= buffer_len || receive_buffer[index] != ')') {
                    return false;
                }
                
                current_task.instructions[instr_count].type = MOVE_FORWARD;
                current_task.instructions[instr_count].value = atoi(distance_str);
                instr_count++;
                break;
            
            
            case 'L':
                if (index + 1 >= buffer_len || receive_buffer[index + 1] != '(') {
                    return false;
                }
                
                char distance_str[10] = 0;
                int dist_index = 0;
                index += 2;  // Skip 'L('
                
                while (index < buffer_len && receive_buffer[index] != ')' && dist_index < 9) {
                    distance_str[dist_index++] = receive_buffer[index++];
                }
                
                if (index >= buffer_len || receive_buffer[index] != ')') {
                    return false;
                }
                current_task.instructions[instr_count].type = TURN_LEFT;
                current_task.instructions[instr_count].value = atoi(distance_str);
                instr_count++;
                break;
                
            case 'R':
                if (index + 1 >= buffer_len || receive_buffer[index + 1] != '(') {
                    return false;
                }
                
                char distance_str[10] = 0;
                int dist_index = 0;
                index += 2;  // Skip 'R('
                
                while (index < buffer_len && receive_buffer[index] != ')' && dist_index < 9) {
                    distance_str[dist_index++] = receive_buffer[index++];
                }
                
                if (index >= buffer_len || receive_buffer[index] != ')') {
                    return false;
                }
                current_task.instructions[instr_count].type = TURN_RIGHT;
                current_task.instructions[instr_count].value = atoi(distance_str);
                instr_count++;
                break;
                
            case 'W':
                current_task.instructions[instr_count].type = MARK_WAYPOINT;
                current_task.instructions[instr_count].value = 0;
                instr_count++;
                break;
            case '>':
                current_task.instructions[instr_count].type = END_TASK;
                current_task.instructions[instr_count].value = 0;
                instr_count++;
                break;
            case 'S':
                current_task.instructions[instr_count].type = EMERGENCY_STOP;
                current_task.instructions[instr_count].value = 0;
                instr_count++;
                break;
                
            case ' ':
            case '\r':
            case '\n':
                break;
                
            default:
                return false;
        }
        
        index++;
    }
    
    current_task.num_instructions = instr_count;
    current_task.is_loaded = true;
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Loaded %d instructions successfully", instr_count);
    uart_puts(uart0, buffer);
    
    return true;
}
