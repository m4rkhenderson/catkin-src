import numpy as np
from numpy import linalg as la


def nearVertices(q, tree, distance):
    v = []

    for i in range(len(tree)):
        d = la.norm(np.subtract(q.pose[0:2], tree[i].pose[0:2]))
        if d <= distance:
            v.append(tree[i].id)
    return v
