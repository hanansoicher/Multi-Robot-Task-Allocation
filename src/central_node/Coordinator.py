import sys  
import os 
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from SMrTa.MRTASolver import MRTASolver, Robot
from SMrTa.MRTASolver.objects import Task
import Vision


class Coordinator:
    def __init__(self):
        video = "img/mazewmarkers.png"
        self.vg = Vision(video)
        self.vg.run()

    def run_solver(self):
        agents = [
            Robot(id=0, start=(2,2)),
            Robot(id=1, start=(2,8)),
            Robot(id=2, start=(8,8)),
        ]
        tasks = [
            Task(id=0, start=(8,2), end=(2,8)),
            Task(id=1, start=(2,5), end=(5,2)),
            Task(id=2, start=(5,8), end=(2,2)),
        ]
        solver_graph_locs = [[agent.start for agent in agents] + [task.start for task in tasks] + [task.end for task in tasks]].unique()
        solver_graph = self.vg.create_travel_time_matrix(solver_graph_locs)
        tasks_stream = [[tasks, 0]]

        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFBV', 
            agents=agents,
            tasks_stream=tasks_stream,
            room_graph=solver_graph,
            capacity=1,
            num_aps=len(solver_graph_locs),
            aps_list=[len(solver_graph_locs)],
            fidelity=1,
        )

        if solver.sol is None:
            print("No solution found!")
            return None

        return solver.sol
    
def main():
    c = Coordinator()
    c.run_solver()

if __name__ == '__main__':
    main()