import numpy as np
from numpy import linalg as la


def nearVertices(q, tree, distance):
    distance = distance + 1
    v = []

    for i in range(len(tree)):
        d = la.norm(np.subtract(q.pose, tree[i].pose))
        if d <= distance:
            v.append(tree[i].id)
    return v
