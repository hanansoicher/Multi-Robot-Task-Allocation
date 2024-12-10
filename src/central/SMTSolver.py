import json
from typing import List
from SMrTa import MRTASolver
from central.GraphConstructor import GraphConstructor, QRLocation
import numpy as np
from structs import *


class SMTSolver:
    def __init__(self):
        self.assignments = None
      
    def solve(self, room_graph: List[List[int]], num_aps: int, agents: List[Agent], tasks: List[Task]):
        # Solution output in SMT_solution.smt2
        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFBV',
            agents=agents,
            tasks_stream=((tasks[i], 0) for i in range(len(tasks))),
            room_graph=room_graph,
            capacity=2,
            num_aps=num_aps, # number of intersections in maze
            fidelity=1,
            basename='SMT_solution'
        )
        # MRTASolver output: sol = {'agt': [{'t': [], 'c': [], 'id': []} for i in range(num_agents)]}
        # t: Start time of action, c: Capacity of agent, id: Action/Task ID
        return solver.extract_model('z3')