This project implements a dynamic task allocation system for a team of Pololu 3pi+ robots given a set of tasks (Picking up and dropping off objects with deadlines) in a constrained environment, utilizing an SMT-based Multi Robot Task Allocation library (SMrTa) developed by UC Berkeley post-graduate researchers. 

Setup:        
git clone --recurse-submodules    
git submodule update --init --recursive    
Add files in lib/submodule_edits    

To build robot_loop:        
mkdir build && cd build    
cmake -G "Unix Makefiles" ..    
make    

For Coordinator:        
python -m venv venv    
source venv/bin/activate   (venv\Scripts\activate.ps1 on Windows)    
pip install -r requirements.txt    

Flash robot_loop.ulf2 to robots, place in environment, then run Coordinator.py from venv    


Features:
- Real-time environment mapping and object detection using overhead camera, converted to grid coordinate system for pathfinding between task locations
- Robot calibration for determining accurate travel times along optimal paths provided as input to SMT solver
- SMT-based task allocation considering temporal and spatial constraints
- Conversion of SMrTa solution (Schedule of pickup and dropoff locations assigned to each robot) into physical point-to-point movement commands
- Collision pre-emption and avoidance between robots, requiring path re-planning for overlapping paths while ensuring adherence to task deadline satisfiability
- Support for dynamically-introduced obstacles and modular environments
- Bluetooth communication for sending movement commands from a central coordinator node to the robots using HM-10 Bluetooth Low Energy Modules
- Visual monitoring and debugging interface
