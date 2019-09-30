import PRMClasses as pc
import numpy as np
from numpy import linalg as la


def steering(q, qNear, cntId, distance):
    d = la.norm(np.subtract(q.pose, qNear.pose))
    if d > distance:
        dX = q.pose[0] - qNear.pose[0]
        dY = q.pose[1] - qNear.pose[1]
        ang = np.arctan2(dY, dX)
        qNew = pc.Vertex(cntId + 1, [qNear.pose[0] + distance * np.cos(ang),
                                     qNear.pose[1] + distance * np.sin(ang)], [])
    else:
        qNew = pc.Vertex(cntId + 1, q.pose, [])
    return qNew
