import numpy as np
from numpy import linalg as la


def nearVertices(q, tree):
    dist = 11
    v = []

    for i in range(len(tree)):
        d = la.norm(np.subtract(q.pose, tree[i].pose))
        if d <= dist:
            v.append(tree[i].id)
    return v
