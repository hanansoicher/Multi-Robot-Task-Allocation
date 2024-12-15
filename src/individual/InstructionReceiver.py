from MotorController import MotorController

class InstructionReceiver:
    def __init__(self):
        self.motor_controller = MotorController()
        
    def parse_instruction(self, instruction_str):
        try:
            instructions = instruction_str.split('>')
            commands = list(tuple)
            
            for instr in instructions:
                if ':' in instr:
                    cmd, duration = instr.split(':')
                    duration = int(duration)
                else:
                    cmd = instr
                    duration = 1000  # Default spin duration
                commands.append((cmd, duration))

            for command in commands:
                self.execute_command(command[0], command[1])                
                # Send acknowledgment
            self.motor_controller.uart.write("COMPLETE PATH\n")
                
        except Exception as e:
            print(f"Error executing instruction: {e}")
            self.motor_controller.stop()  # Emergency stop
            
    def execute_command(self, command, duration):
        """Execute a single movement command."""
        if command == 'F':
            self.motor_controller.move_forward(duration)
        elif command == 'L':
            self.motor_controller.turn_left(duration)
        elif command == 'R':
            self.motor_controller.turn_right(duration)
        elif command == 'P' or command == 'D':
            self.motor_controller.spin()
        else:
            print(f"Unable to execute command: {command}")