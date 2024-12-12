class DecisionPoint:
    def __init__(self, id: int, x: float, y: float, type: str):
        self.id = id
        self.x = x
        self.y = y
        self.type = type # robot, obstacle, corner, intersection (aka action point)