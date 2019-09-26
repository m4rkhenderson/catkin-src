class Vertex:
    def __init__(self, id, pose, pid):
        self.id = id
        self.pose = pose
        self.pid = pid


class Obstacle:
    def __init__(self, points, radius):
        self.points = points
        self.radius = radius

