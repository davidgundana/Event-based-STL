from getAllCommands import getAllCommands
import re
import numpy as np

def getCMD(State,posX,posY,posTheta,posxinit,posyinit,posthetainit,posxperson,posyperson,posthetaperson,currTime,startTime,currState,input,until):
    # Change the inputs to a simpler, more usable format.
    pos = np.zeros((1,3*np.size(posX)))[0]
    posStart = np.zeros((1, 3 * np.size(posX)))[0]
    posPerson = np.zeros((1,3*np.size(posxperson)))[0]

    for i in range(np.size(posX)):
        pos[3*i] = posX[i]
        pos[3*i+1] = posY[i]
        pos[3*i+2] = posTheta[i]
        posStart[3*i] = posxinit[i]
        posStart[3*i+1] = posyinit[i]
        posStart[3*i+2] = posthetainit[i]
    for i in range(np.size(posxperson)):
        posPerson[3*i] = posxperson[i]
        posPerson[3*i+1] = posyperson[i]
        posPerson[3*i+2] = posthetaperson[i]

    # Check to see if you care about walls. If you do, find the closest point
    isWall = [re.findall('wall', elem) for elem in State.State.parameters]
    flatWall = [item for sublist in isWall for item in sublist]
    if flatWall != []:
        for i in range(int(np.size(pos)/3)):
            if i == 0:
                wall = findPoint(State,pos[3*i:3*i+2])
            else:
                wall = np.append(wall,findPoint(State,pos[3*i:3*i+2]))

        State.State.wall = wall
    else:
        State.State.wall = []

    # Get the commands given the input
    a = getAllCommands(State,currState, pos, posStart, posPerson, currTime, startTime, input,until)

    # Parse the output and package it to send back
    distTotal = a.distTotal
    cmd = a.nom[0]

    vx = np.zeros((1,np.size(posX)))
    vy = np.zeros((1, np.size(posX)))
    vtheta = np.zeros((1, np.size(posX)))
    for i in range(np.size(posX)):
        vx[0,i] = cmd[3*i]
        vy[0,i] = cmd[3*i + 1]
        vtheta[0,i] = cmd[3*i + 2]

    currState = a.currState
    newinput = a.State.input
    newUntil = a.State.until
    return vx, vy, vtheta, currState, distTotal,newinput,newUntil

def findPoint(State,pos):
    map = State.State.map
    wallPoints = np.empty((1,2))
    wallDistances = np.empty((1,1))
    for k in range(np.size(map,0)):
        p1 = map[k,0:2]
        p2 = map[k,2:4]
        wall, dist2wall = distWall(p1,p2,pos)
        wallPoints = np.vstack((wallPoints,wall))
        wallDistances = np.vstack((wallDistances,dist2wall))

    wallPoints = wallPoints[1:,:]
    wallDistances = wallDistances[1:,:]

    minDist = np.argmin(wallDistances)

    wall = wallPoints[minDist]

    return wall

def distWall(p1,p2,pt):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    t = ((pt[0]-p1[0])*dx + (pt[1]-p1[1])*dy)/(dx**2 + dy**2)

    if dx == 0 and dy == 0:
        closestP = p1
        dx = pt[0] - p1[0]
        dy = pt[1] - p1[1]
        dist = np.sqrt(dx**2 + dy**2)
    elif t < 0:
        closestP = p1
        dx = pt[0] - p1[0]
        dy = pt[1] - p1[1]
        dist = np.sqrt(dx**2 + dy**2)
    elif t > 1:
        closestP = p2
        dx = pt[0] - p2[0]
        dy = pt[1] - p2[1]
        dist = np.sqrt(dx**2 + dy**2)
    else:
        closestP = np.array([p1[0] + t*dx, p1[1] + t*dy])
        dx = pt[0] - closestP[0]
        dy = pt[1] - closestP[1]
        dist = np.sqrt(dx ** 2 + dy ** 2)

    return closestP, dist
