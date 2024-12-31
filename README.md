This project implements a dynamic task allocation system for a team of robots given a set of tasks (Picking up and dropping off objects with deadlines) in a maze environment, utilizing an SMT-based Multi Robot Task Allocation library developed by UC Berkeley post-graduate researchers. 

Key features:
- Real-time environment mapping and object detection using overhead camera, converted to grid coordinate system for pathfinding between task locations
- Robot calibration for determining accurate travel times along optimal paths provided as input to SMT solver
- SMT-based task allocation considering temporal and spatial constraints
- Conversion of SMrTa solution (Schedule of pickup and dropoff locations assigned to each robot) into physical point-to-point movement commands
- Collision pre-emption and avoidance between robots, requiring path re-planning for overlapping paths while ensuring adherence to task deadline satisfiability
- Support for dynamically-introduced obstacles and modular environments
- Bluetooth communication for sending movement commands from a central coordinator node to the robots using HM-10 Bluetooth Low Energy Modules
- Visual monitoring and debugging interface
