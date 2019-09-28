import numpy as np
from numpy import linalg as la


def nearestNeighbour(q, tree):
    minD = 10000000000
    minV = 0

    for i in range(len(tree)):
        d = la.norm(np.subtract(q.pose, tree[i].pose))
        if d < minD:
            minV = i
            minD = d
    v = tree[minV]
    return v
