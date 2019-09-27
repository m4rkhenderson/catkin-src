import numpy as np
from numpy import linalg as la


def checkCollision(q, obs, rRadius):
    flag = -1
    for i in range(len(obs.points)):
        dist = la.norm(np.subtract(q.pose, obs.points[i]))
        inflatedDist = dist + -1 * (obs.radius + rRadius)
        if inflatedDist > 0:
            flag = -1
        else:
            flag = 1
            break
    return flag
