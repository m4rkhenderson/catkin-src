class Vertex:
    def __init__(self, id, pose, pid, cost):
        self.id = id
        self.pose = pose
        self.pid = pid
        self.cost = cost


class Obstacle:
    def __init__(self, points, radius):
        self.points = points
        self.radius = radius
