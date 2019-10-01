class Vertex:
    def __init__(self, id, pose, pid, vel):
        self.id = id
        self.pose = pose
        self.pid = pid
        self.vel = vel


class Obstacle:
    def __init__(self, points, radius):
        self.points = points
        self.radius = radius
