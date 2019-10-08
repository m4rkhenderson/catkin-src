import NearVertices as nv
import numpy as np
from numpy import linalg as la


def rewire(q, tree, volume, step_size):
    c_min = q.cost
    gamma = 2*pow((1+1/2), 1/2)*pow((volume/3.14159), 1/2)
    card = len(tree)
    r = min((gamma*pow((np.log(card)/card), 1/2)), step_size)

    v = nv.nearVertices(q, tree, r)
    if q.pid[0] not in v:
        v.append(q.pid[0])

    for i in range(len(v)):
        v_cost = tree[v[i]].cost + la.norm(np.subtract(q.pose[0:2], tree[v[i]].pose[0:2]))
        if v_cost <= c_min:
            v_nearest = v[i]
            q.pid = [v_nearest]
            c_min = v_cost
            q.cost = c_min

    for i in range(len(v)):
        v_cost = q.cost + la.norm(np.subtract(q.pose[0:2], tree[v[i]].pose[0:2]))
        if v_cost < tree[v[i]].cost:
            tree[v[i]].cost = v_cost
            tree[v[i]].pid = [q.id]

    tree.append(q)

    return tree
