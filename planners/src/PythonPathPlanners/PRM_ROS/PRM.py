#!/usr/bin/env python
"""
PRM Algorithm
description: randomly samples a given configuration space,
checks for all nearest neighbours in a given radius or k-nearest
neighbours, from this a graph G(V,E) is generated
"""

import CollisionTest as ct
import ExtractPath as ep
import NearVertices as nv
import NearestNeighbour as nn
import PRMClasses as pc
import Sampling as sm
import Steering as st
import matplotlib.pyplot as plt
import numpy as np
from numpy import linalg as la

tree = []

rRadius = 5
gRadius = 2.5
minDist = 3
qInit = [70, 70]
qGoal = [30, 30]
xmax = 100
ymax = 100

obs = pc.Obstacle([[0, 0], [20, 0], [40, 0], [60, 0], [80, 0], [100, 0], [100, 20], [100, 40], [100, 60], [100, 80],
                   [100, 100], [80, 100], [60, 100], [40, 100], [20, 100], [0, 100], [0, 80], [0, 60], [0, 40],
                   [0, 20], [50, 50]], 20)
nObs = len(obs.points)

fig, ax = plt.subplots()
ax.set_xlim((0, 100))
ax.set_ylim((0, 100))
for i in range(nObs):
    ax.add_artist(plt.Circle(obs.points[i], obs.radius, color='b'))
ax.add_artist(plt.Circle((qInit[0], qInit[1]), rRadius, color='c'))
ax.add_artist(plt.Circle((qGoal[0], qGoal[1]), gRadius, color='g'))

addedVert = 1
collision = 0
goalFound = 0
cntId = 0

tree.append(pc.Vertex(cntId, qInit, []))

plt.ion()

while goalFound < 1:
    qRand = pc.Vertex([], sm.sampling(xmax, ymax), [])

    if ct.checkCollision(qRand, obs, rRadius) > 0:
        print("INVALID QRAND")
        collision = collision + 1
        continue
    else:
        print("VALID QRAND")
        ax.plot(qRand.pose[0], qRand.pose[1], '+y')

    qNear = pc.Vertex([], nn.nearestNeighbour(qRand, tree), [])
    qNew = st.steering(qRand, qNear, cntId, minDist)

    if ct.checkCollision(qNew, obs, rRadius) > 0:
        print("INVALID QNEW")
        collision = collision + 1
        continue
    else:
        print("VALID QNEW")
        ax.plot(qNew.pose[0], qNew.pose[1], 'ok')
        cntId = cntId + 1
        addedVert = addedVert + 1

    vNear = nv.nearVertices(qNew, tree, minDist)

    qNew.pid = vNear
    tree.append(qNew)
    for i in range(len(vNear)):
        ax.plot([tree[vNear[i]].pose[0], qNew.pose[0]], [tree[vNear[i]].pose[1], qNew.pose[1]], '-k')

    if la.norm(np.subtract(qNew.pose, qGoal)) < gRadius:
        print("GOAL FOUND !!!")
        goalFound = 1
        p = ep.extractPath(qNew, tree, qInit)
        if p != 0:
            plt.plot(p[0], p[1], 'r*-', linewidth=2, markersize=10)
            print(tree)
        else:
            print("Couldn't Find Path, Insufficient Iterations or Goal Non-Existent")
        break
    plt.draw()
    plt.pause(0.00001)

plt.ioff()
plt.show()
