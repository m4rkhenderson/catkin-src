class Vertex:
    def __init__(self, id, pose, edge, pid):
        self.id = id
        self.pose = pose
        self.edge = edge
        self.pid = pid


class Obstacle:
    def __init__(self, shape):
        self.shape = shape

