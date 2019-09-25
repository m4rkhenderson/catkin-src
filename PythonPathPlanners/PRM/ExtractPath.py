def extractPath(q, tree, qInit):
    Q = [q]
    discovered = [[q], [0]]
    while Q:
        v = Q.pop(0)

        if v.pose == qInit:
            p = [[v.pose[0]], [v.pose[1]]]
            vertex = [discovered[0][discovered[0].index(v)], discovered[1][discovered[0].index(v)]]
            while vertex[0].id != q.id:
                v = tree[vertex[1]]
                vertex = [discovered[0][discovered[0].index(v)], discovered[1][discovered[0].index(v)]]
                p[0].append(v.pose[0])
                p[1].append(v.pose[1])
            return p

        for i in range(len(v.pid)):
            parent = [tree[v.pid[i]], v.id]
            if parent[0] not in discovered[0]:
                Q.extend([parent[0]])
                discovered[0].extend([parent[0]])  # mark as discovered
                discovered[1].extend([parent[1]])
    return 0
