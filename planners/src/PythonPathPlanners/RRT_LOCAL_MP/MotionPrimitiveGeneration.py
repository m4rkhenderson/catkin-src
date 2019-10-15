import numpy as np
import RRTClasses as rc
import CollisionTest as ct


def generateMP(q, cntId, nSteps, tStep, mpArray, obs, rRadius):
    branch = []
    cntId = cntId + 1
    vertices = 0

    for i in range(len(mpArray)):
        x = q.pose[0]
        y = q.pose[1]
        th = q.pose[2]

        v = mpArray[i][0]  # mpArray is in form [[v0,vth0],[v1,vth1],...,[vi,vthi]]
        vth = mpArray[i][1]

        parent = q.id

        for i in range(nSteps):
            dth = vth*tStep
            th = th + dth

            dd = v*tStep
            dx = dd*np.cos(th)
            dy = dd*np.sin(th)
            x = x + dx
            y = y + dy

            if ct.checkCollision(rc.Vertex([], [x, y, th], [], []), obs, rRadius) > 0:
                break

            branch.append(rc.Vertex(cntId, [x, y, th], [parent], [v, vth]))
            parent = branch[vertices].id
            cntId = cntId + 1
            vertices = vertices + 1

    return branch
