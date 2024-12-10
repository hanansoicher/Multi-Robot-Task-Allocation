class QRLocation:
    def __init__(self, id: int, x: float, y: float, type: str):
        self.id = id
        self.x = x
        self.y = y
        self.type = type # robot, obstacle, corner, intersection (aka action point)

class Agent:
    def __init__(self, id: int, start: int):
        self.id = id
        self.start = start #  QRLocation of robot's starting location

class Task:
    def __init__(self, id: int, start: QRLocation, end: QRLocation, deadline: int = None):
        self.id = id
        self.start = start # pickup QRLocation id
        self.end = end # dropoff QRLocation id
        self.deadline = deadline
