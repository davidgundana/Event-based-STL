import socket
import time
from prepForCommands import getCMD
from runEvBasedSTL import formData
import pickle
import numpy as np
import os
import datetime
import logging
import nom
from activateProp import activateProp

tpath = os.getcwd()
my_dir = os.path.dirname(os.path.abspath(__file__))

pklNum = input("Enter specification you want to run (p1 for passive route 1, a1 for active route 1, p2 for passive route 2, or a2 for active route 2) and press enter: ")
if pklNum == 'p1':
    pckFile = 'NRIPassiveRoute1.pkl'
elif pklNum == 'a1':
    pckFile = 'NRIActiveRoute1.pkl'
elif pklNum == 'p2':
    pckFile = 'NRIPassiveRoute2.pkl'
elif pklNum == 'a2':
    pckFile = 'NRIActiveRoute2.pkl'
print("Running " + pckFile)
pickle_file_path = os.path.join(my_dir, 'PickleFiles',pckFile)
my_dir = os.path.dirname(os.path.abspath(__file__))
with open(pickle_file_path, 'rb') as input:
    f = pickle.load(input)

# f.M = 1
n_inputs = 2
n_people = 1
posX = np.zeros((1, f.M))
posY = np.zeros((1, f.M))
posTheta = np.zeros((1, f.M))
posxinit = np.zeros((1, f.M))
posyinit = np.zeros((1, f.M))
posthetainit = np.zeros((1, f.M))
posXPerson = np.zeros((1, n_people))
posYPerson = np.zeros((1, n_people))
posThetaPerson = np.zeros((1, n_people))

Headersize = 75
sendSize = 500
conn = 0
nomsg = 1
input = np.zeros((1, n_inputs * 2), dtype=float)
try:
    s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address2 = ('localhost', 8120)
    s2.bind(server_address2)
    s2.listen(5)

except:
    print("Python client opened")
    print("Python is listening")

try:
    while True:
        full_msg = ''
        new_msg = True

        while True:
            while nomsg == 1:
                try:
                    print("Attempting to establish connection to Unity")
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    server_address = ('localhost', 8052)
                    s.settimeout(2)
                    s.connect(server_address)
                    msg = s.recv(1024)
                    nomsg = 0
                    print("Connection Established!")
                except:
                    print("Connection Failed. Retrying")
                    time.sleep(1)
            try:
                msg = s.recv(1024)
            except:
                nomsg = 1

            try:
                msglen = int(msg[:Headersize])
                full_msg += msg.decode("utf-8")
            except:
                full_msg = ''
                msglen = 0
            new_msg = False

            if len(full_msg) > 350:
                full_msg = ''

            if len(full_msg)-Headersize == msglen:
                try:
                    new_msg = True

                    full_msg = full_msg[Headersize:]
                    parsedMes = full_msg.split()

                    for i in range(f.M):
                        posX[0,i] = float(parsedMes[3*i])
                        posY[0,i] = float(parsedMes[3*i+1])
                        posTheta[0,i] = float(parsedMes[3*i+2])

                    firstMessage = 2*(f.M*3) + 3*n_people + 3 + 2*n_inputs

                    for i in range(f.M):
                        posxinit[0,i] = float(parsedMes[3*i + 3*f.M])
                        posyinit[0,i] = float(parsedMes[3*i + 1 + 3*f.M])
                        posthetainit[0,i] = float(parsedMes[3*i+ 2 + 3*f.M])
                    for i in range(n_people):
                        posXPerson[0,i] = float(parsedMes[3*i + 6*f.M])
                        posYPerson[0,i] = float(parsedMes[3*i + 1 + 6*f.M])
                        posThetaPerson[0,i] = float(parsedMes[3*i + 2 + 6*f.M])




                    currTime = float(parsedMes[6*f.M + 3*n_people])
                    startTime = float(parsedMes[6*f.M + 1 + 3*n_people])
                    currState = int(parsedMes[6*f.M + 2 + 3*n_people])

                    for i in range(n_inputs*2):
                        try:
                            input[0, i] = float(parsedMes[6 * f.M + 3 + i + 3 * n_people])
                        except:
                            pass
                    #print(input)

                    msg = ""
                    if firstMessage*2 != len(full_msg):
                        print("DEBUG MESSAGE: " + full_msg)

                        #msg = "1 "
                        unt = 1
                        a = time.time()
                        if pklNum == 'a1' or pklNum == 'a2':
                            input = np.zeros((1, 2), dtype=float)

                            lineOfSight = 0
                            dist = np.sqrt((posX[0, 0] - posXPerson[0, 0]) ** 2 + (posY[0, 0] - posYPerson[0, 0]) ** 2)
                            isect = activateProp.intersectPoint([],posX[0,0], posY[0,0], posXPerson[0,0], posYPerson[0,0],
                                                                f.map[:, 0], f.map[:, 1], f.map[:, 2], f.map[:, 3])

                            if not np.any(isect):
                                lineOfSight = 1


                            if lineOfSight == 0 or dist > 40:
                                input[0,0] = 1
                                input[0,1] = currTime

                            # closeEnough = np.sqrt((posX[0][0] - posXPerson[0][0]) ** 2 + (posY[0][0] - posYPerson[0][0]) ** 2)
                            # nomTemp, dist2NextPoint, distX, distY, distTotal, lastPoint = nom.getNom(f.State, np.array(
                            #     [posX[0,0], posY[0,0]]), 2)
                            # timeRem = distTotal / f.maxV[0]
                            # if timeRem < currTime - 20:
                            #     enoughTime = 0
                            # else:
                            #     enoughTime = 1

                            # if closeEnough > 3 and input[0,0] == 0:
                            #     input[0,0] = 1
                            #     input[0,1] = currTime
                            # if closeEnough < 3 and enoughTime and input[0,2] == 0:
                            #     input[0,2] = 1
                            #     input[0,3] = currTime
                            # if not enoughTime and input[0,2] == 0:
                            #     input[0,2] = 1
                            #     print("..............................................................................")
                            #     input[0,3] = currTime
                            # if input[0,2] == 1:
                            #     print('............................................................................')
                        try:
                            vx, vy, vtheta, currState, distTotal, newinput, unt = getCMD(f, posX[0], posY[0], posTheta[0],
                                                                                         posxinit[0], posyinit[0],
                                                                                         posthetainit[0], posXPerson[0],
                                                                                         posYPerson[0], posThetaPerson[0],
                                                                                         currTime, startTime, currState,
                                                                                         input[0], unt)
                            # print(posX[0], posY[0], posTheta[0],posxinit[0], posyinit[0],
                            #                                                              posthetainit[0], posXPerson[0],
                            #                                                              posYPerson[0], posThetaPerson[0],
                            #                                                              currTime, startTime, currState,
                            #                                                              input[0], unt)
                        except Exception as exception:
                            logging.basicConfig(level=logging.DEBUG, filename='errorLog.txt')
                            theDate = datetime.datetime.now()
                            logging.exception(theDate)
                            #file1 = open("errorLog.txt", "a")
                            #file1.write(str(theDate) + '\n')
                            #file1.write(exception.GetType().FullName + '\n\n\n')
                            print(exception)
                            print('An exception occurred. Logging and stopping robot')
                            vx= np.array([[0]])
                            vy= np.array([[0]])
                            vtheta= np.array([[0]])

                        b = time.time() - a
                        # print(str(b) + ' seconds since last update')

                        if f.M == 1:
                            msg = msg + str(vx[0][0]) + " "
                            msg = msg + str(vy[0][0]) + " "
                            msg = msg + str(vtheta[0][0]) + " "
                        else:
                            for i in range(f.M):
                                msg = msg + str(vx[0, i]) + " "
                                msg = msg + str(vy[0, i]) + " "
                                msg = msg + str(vtheta[0, i]) + " "

                        msg = msg + str(currState) + " " + "0"

                        #for i in range(f.M):
                            #msg = msg + str(distTotal[i]) + " "

                        #for i in range(np.size(newinput)):
                            #msg = msg + str(newinput[i]) + " "
                        print(msg)
                    else:
                        msg = "0 "
                        for i in range(firstMessage):
                            msg = msg + '0 '
                    full_msg = ''
                    try:
                        msglen = len(msg)
                        padL = ' '.ljust(sendSize-msglen-1)
                    except:
                        padL = ' '.ljust(sendSize)

                    msg = f"{msg} {padL}"

                    s.sendall(bytes(msg, "utf-8"))
                    full_msg = ''
                except:
                    full_msg = ''
                    print('error')

except Exception as exception:
    assert type(exception).__name__ == 'NameError'
    assert exception.__class__.__name__ == 'NameError'
    assert exception.__class__.__qualname__ == 'NameError'
    s.close()
    print(str(exception))
    print("Process Terminated")

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


def intersectPoint(x1, y1, x2, y2, x3, y3, x4, y4):
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)

    ua = np.divide(((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)), denom)
    ub = np.divide(((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)), denom)

    isect = (ua >= 0) * (ub >= 0) * (ua <= 1) * (ub <= 1)

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