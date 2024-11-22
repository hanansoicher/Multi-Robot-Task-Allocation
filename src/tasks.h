typedef enum {
    MOVE_FORWARD,
    TURN_LEFT, 
    TURN_RIGHT,
    MARK_WAYPOINT,  // Perform 360 spin at waypoint
    EMERGENCY_STOP,
    START_TASK,
    END_TASK
} InstructionType;

typedef struct {
    InstructionType type;
    int sequence_num;
    int value; // mm for MOVE_FORWARD, angle for TURN_LEFT/RIGHT
} Instruction;

typedef struct {
    int task_id;
    int num_instructions;
    Instruction instructions[100];
    bool is_loaded; // for latency between receiving assignment over bluetooth and beginning execution
} TaskAssignment;