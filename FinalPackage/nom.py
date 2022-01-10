import numpy as np
import time
import matrixDijkstra

def getNom(State, pos, p):

    goalPoint = np.array([-62.1, -133.33])
    closeEnough = .15

    # Find the 3 closest points that do not intersect
    dist2p = []
    starting = time.time()
    for i in range(np.size(State.nodes, 0)):
        dist2p.append(np.sqrt((goalPoint[0] - State.nodes[i, 0]) ** 2 + (goalPoint[1] - State.nodes[i, 1]) ** 2))

    k = 12
    try:
        idx = np.argpartition(dist2p, k)
        idx = idx[:k]
    except:
        k = 2
        idx = np.argpartition(dist2p, k)
        idx = idx[:k]

    newRowGoal = np.zeros((1, np.size(State.nodeGraph, 1)))
    newRowGoal = newRowGoal[0]
    map = State.map
    # need to focus on a smaller area of the map if its big
    mapGoal = np.multiply((map[:, 0:2] - goalPoint), (map[:, 0:2] - goalPoint))
    mapAdd = np.sqrt(mapGoal[:, 0] + mapGoal[:, 1])
    walls2check = 80
    idxG = np.argpartition(mapAdd, walls2check)
    idxG = idxG[:walls2check]
    map2ConsiderGoal = map[idxG]
    for i in range(np.size(idx)):
        isect = []
        for k in range(np.size(map2ConsiderGoal, 0)):
            isecttemp = intersectPoint(goalPoint[0], goalPoint[1], State.nodes[idx[i], 0], State.nodes[idx[i], 1],
                                            map2ConsiderGoal[k, 0], map2ConsiderGoal[k, 1], map2ConsiderGoal[k, 2],
                                            map2ConsiderGoal[k, 3])
            if isecttemp == 1:
                isect.append(isecttemp)

        if len(isect) == 0:
            newRowGoal[idx[i]] = dist2p[idx[i]]

    dist2p2 = []
    for i in range(np.size(State.nodes, 0)):
        dist2p2.append(np.sqrt((pos[0] - State.nodes[i, 0]) ** 2 + (pos[1] - State.nodes[i, 1]) ** 2))

    k = 100
    try:
        idx = np.argpartition(dist2p2, k)
        idx = idx[:k]
    except:
        k = 2
        idx = np.argpartition(dist2p2, k)
        idx = idx[:k]

    newRowStart = np.zeros((1, np.size(State.nodeGraph, 1)))
    newRowStart = newRowStart[0]

    # shrink map
    mapStart = np.multiply((map[:, 0:2] - pos[0:2]), (map[:, 0:2] - pos[0:2]))
    mapAdd = np.sqrt(mapStart[:, 0] + mapStart[:, 1])

    idxS = np.argpartition(mapAdd, walls2check)
    idxS = idxS[:walls2check]
    map2ConsiderStart = map[idxS]
    for i in range(np.size(idx)):
        isect = []
        for k in range(np.size(map2ConsiderStart, 0)):
            isecttemp = intersectPoint(pos[0], pos[1], State.nodes[idx[i], 0], State.nodes[idx[i], 1],
                                            map2ConsiderStart[k, 0], map2ConsiderStart[k, 1], map2ConsiderStart[k, 2],
                                            map2ConsiderStart[k, 3])
            if isecttemp == 0:
                pt1 = pos[0:2]
                pt2 = State.nodes[idx[i], 0:2]
                ptOfI1 = map2ConsiderStart[k, 0:2]
                ptOfI2 = map2ConsiderStart[k, 2:4]
                closest2P, dist2closest1 = distWall(pt1, pt2, ptOfI1)
                closest2P, dist2closest2 = distWall(pt1, pt2, ptOfI2)
                if dist2closest1 < .25 or dist2closest2 < .25:
                    isecttemp = 1

            if isecttemp == 1:
                isect.append(isecttemp)

        if len(isect) == 0:
            newRowStart[idx[i]] = dist2p2[idx[i]]

    nodeGraph = np.vstack((State.nodeGraph, newRowStart))
    nodes = np.vstack((State.nodes, pos))
    StartNode = np.size(nodes, 0) - 1

    nodeGraph = np.vstack((nodeGraph, newRowGoal))
    nodes = np.vstack((nodes, goalPoint))
    goalNode = np.size(nodes, 0) - 1

    keepSquareStart = newRowStart.reshape(-1, 1)
    keepSquareGoal = newRowGoal.reshape(-1, 1)
    keepSquareStart = np.vstack((keepSquareStart, np.zeros((2, 1))))
    keepSquareGoal = np.vstack((keepSquareGoal, np.zeros((2, 1))))

    # Determine if you can reach the goal from where you are. Last two nodes
    isect = []
    # shrink map
    mapConnect = np.multiply((map[:, 0:2] - goalPoint), (map[:, 0:2] - goalPoint))
    mapAdd = np.sqrt(mapConnect[:, 0] + mapConnect[:, 1])
    idxC = np.argpartition(mapAdd, 20)
    idxC = idxC[:40]
    map2ConsiderConnect = map[idxC]

    for k in range(np.size(map2ConsiderConnect, 0)):
        isecttemp = intersectPoint(nodes[StartNode, 0], nodes[StartNode, 1], nodes[goalNode, 0],
                                        nodes[goalNode, 1],
                                        map2ConsiderConnect[k, 0], map2ConsiderConnect[k, 1], map2ConsiderConnect[k, 2],
                                        map2ConsiderConnect[k, 3])

        if isecttemp == 0:
            pt1 = nodes[StartNode]
            pt2 = nodes[goalNode]
            ptOfI1 = map2ConsiderConnect[k, 0:2]
            ptOfI2 = map2ConsiderConnect[k, 2:4]
            closest2P, dist2closest1 = distWall(pt1, pt2, ptOfI1)
            closest2P, dist2closest2 = distWall(pt1, pt2, ptOfI2)
            if dist2closest1 < .1 or dist2closest2 < .1:
                isecttemp = 1

        if isecttemp == 1:
            isect.append(isecttemp)

    if len(isect) == 0:
        reachGoal = np.sqrt(
            (nodes[StartNode, 0] - nodes[goalNode, 0]) ** 2 + (nodes[StartNode, 1] - nodes[goalNode, 1]) ** 2)
        keepSquareStart[goalNode] = reachGoal
        keepSquareGoal[StartNode] = reachGoal

    nodeGraph = np.hstack((nodeGraph, keepSquareStart))
    nodeGraph = np.hstack((nodeGraph, keepSquareGoal))

    # Only need to run dijkstra if you cant reach your point from where you are
    if nodeGraph[goalNode, StartNode] == 0:
        (cost, rute) = matrixDijkstra.dijkstra(nodeGraph, StartNode, goalNode)
        rute = np.flip(rute)
    else:
        rute = np.array([StartNode, goalNode])
        cost = nodeGraph[goalNode, StartNode]

    elapsedT = time.time() - starting
    # print(str(elapsedT) + ' to run dijkstra')

    wayPoint = nodes[int(rute[1])]
    distx = 0
    disty = 0
    distTotal = 0
    for k in range(np.size(rute) - 1):
        distxTemp = np.abs(nodes[int(rute[k]), 0] - nodes[int(rute[k + 1]), 0])
        distyTemp = np.abs(nodes[int(rute[k]), 1] - nodes[int(rute[k + 1]), 1])
        distx = distx + distxTemp
        disty = disty + distyTemp
        distTotal = distTotal + np.sqrt(distxTemp ** 2 + distyTemp ** 2)
    distTotal = distTotal - p

    # Check to see if this waypoint is too close. If it is go to the next one
    dist2NextPoint = np.sqrt((wayPoint[0] - pos[0]) ** 2 + (wayPoint[1] - pos[1]) ** 2)
    if dist2NextPoint <= closeEnough:
        try:
            wayPoint = nodes[int(rute[2])]
        except:
            wayPoint = nodes[int(rute[-1])]
            dist2NextPoint = np.sqrt((wayPoint[0] - pos[0]) ** 2 + (wayPoint[1] - pos[1]) ** 2)
    nom = wayPoint - pos
    if cost > 20000:
        nom = np.array([[0, 0]])
        print('no path')

    return nom, dist2NextPoint, distx, disty, distTotal, nodes[int(rute[-1])]
def intersectPoint( x1, y1, x2, y2, x3, y3, x4, y4):
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if denom == 0:
        isect = 0
    else:
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom

        if ua >= 0 and ub >= 0 and ua <= 1 and ub <= 1:
            isect = 1
        else:
            isect = 0

    return isect
def distWall( p1, p2, pt):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    t = ((pt[0] - p1[0]) * dx + (pt[1] - p1[1]) * dy) / (dx ** 2 + dy ** 2)

    if dx == 0 and dy == 0:
        closestP = p1
        dx = pt[0] - p1[0]
        dy = pt[1] - p1[1]
        dist = np.sqrt(dx ** 2 + dy ** 2)
    elif t < 0:
        closestP = p1
        dx = pt[0] - p1[0]
        dy = pt[1] - p1[1]
        dist = np.sqrt(dx ** 2 + dy ** 2)
    elif t > 1:
        closestP = p2
        dx = pt[0] - p2[0]
        dy = pt[1] - p2[1]
        dist = np.sqrt(dx ** 2 + dy ** 2)
    else:
        closestP = np.array([p1[0] + t * dx, p1[1] + t * dy])
        dx = pt[0] - closestP[0]
        dy = pt[1] - closestP[1]
        dist = np.sqrt(dx ** 2 + dy ** 2)

    return closestP, dist