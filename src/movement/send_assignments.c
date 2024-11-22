
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "../tasks.h"

void send_assignment(TaskAssignment* assignment) {
    char buffer[1024];
    int buffer_index = 0;
    
    buffer_index += snprintf(buffer + buffer_index, sizeof(buffer) - buffer_index, "<TASK:%d>", assignment->task_id);
    
    for (int i = 0; i < assignment->num_instructions; i++) {
        Instruction* instr = &assignment->instructions[i];
        
        switch (instr->type) {
            case MOVE_FORWARD:
                buffer_index += snprintf(buffer + buffer_index, sizeof(buffer) - buffer_index, "F(%d)", instr->value);
                break;
                
            case TURN_LEFT:
                buffer_index += snprintf(buffer + buffer_index, sizeof(buffer) - buffer_index, "L(%d)", instr->value);
                break;
                
            case TURN_RIGHT:
                buffer_index += snprintf(buffer + buffer_index, sizeof(buffer) - buffer_index, "R(%d)", instr->value);
                break;
                
            case MARK_WAYPOINT:
                buffer_index += snprintf(buffer + buffer_index, sizeof(buffer) - buffer_index, "W");
                break;
                
            case EMERGENCY_STOP:
                buffer_index += snprintf(buffer + buffer_index, sizeof(buffer) - buffer_index, "S");
                break;
                
            case START_TASK:
                buffer_index += snprintf(buffer + buffer_index, sizeof(buffer) - buffer_index, "T");
                break;
                
            case END_TASK:
                buffer_index += snprintf(buffer + buffer_index, sizeof(buffer) - buffer_index, "E");
                break;
        }
    }
    
    buffer_index += snprintf(buffer + buffer_index, sizeof(buffer) - buffer_index, ">");
    uart_puts(uart0, buffer);
}