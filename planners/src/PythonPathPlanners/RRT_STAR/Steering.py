import RRTClasses as rc
import numpy as np
import random as rdm
from numpy import linalg as la


def steering(q, qNear, cntId, dMax , aMax):
    dth = np.absolute(q.pose[2] - qNear.pose[2])
    #  d = la.norm(np.subtract(q.pose[0:2], qNear.pose[0:2]))

    if dth > (aMax):
        angMax = int((qNear.pose[2] + aMax)*100)
        angMin = int((qNear.pose[2] - aMax)*100)
        ang = float(rdm.randint(angMin, angMax))/100
    else:
        ang = q.pose[2]

    qNew = rc.Vertex(cntId + 1, [qNear.pose[0] + dMax * np.cos(ang), qNear.pose[1] + dMax * np.sin(ang), ang],
                     [], qNear.cost + dMax)

    return qNew
