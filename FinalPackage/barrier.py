import numpy as np
import re
import matrixDijkstra
import copy

def barrier(pi, x,xR, t, wall, roadmap, preF,linear):
    # Determine whether until, eventually, or always
    if pi.t_e == []:
        inputTime = 0
    else:
        inputTime = pi.t_e
    # if pi.id == 7:
    #     print('here')

    if pi.type == 'ev':
        # If the interval has passed, then the barrier function doesnt matter
        if t >= pi.a + inputTime and t <= pi.b + inputTime and not pi.currentTruth:
            pi = evBarrier(pi, t, x, xR ,wall,roadmap, preF,linear)
        else:
            pi.bxt_i = None
    if pi.type == 'alw':
        # if phi[i].until == 1:
        #     # If phiUntil is True then we should set the time so that the time period has passed.
        #     isUntTrue = eval(phi[i].phiUntil)
        #     if isUntTrue == 1 and t >= a:
        #         b = t - .1

        if t >= pi.a + inputTime and t <= pi.b + inputTime:
            pi = alBarrier(pi, t, x, xR ,wall, roadmap,preF)
    return pi

def totalBarrier(specattr, ind, indOfActive):
    bxt_i = []
    piRobot = []
    for i in range(np.size(indOfActive,0)):
        if np.any(specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].robotsInvolved== ind):
            try:
                if (specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].type == 'ev' and not specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].currentTruth):
                    if specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].bxt_i is not None:
                        if hasattr(specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]], 'satisfied'):
                            if not specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].satisfied:
                                bxt_i.append(specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].bxt_i)
                                piRobot.append(specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]])
                        else:
                            bxt_i.append(specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].bxt_i)
                            piRobot.append(specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]])
                elif specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].type == 'alw':
                    if specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].bxt_i is not None:
                        bxt_i.append(specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]].bxt_i)
                        piRobot.append(specattr[indOfActive[i][0]].Pi_mu[indOfActive[i][1]])
            except:
                print('fail')
                pass
        else:
            print('not active')
    bxt_iNew = []
    if np.size(bxt_i) > 1 and np.where(np.array(bxt_i)==0)[0].size != 0:
        for k in range(0, np.size(bxt_i)):
            if bxt_i[k] != 0:
                bxt_iNew.append(bxt_i[k])

        bxt_i = bxt_iNew

    bxt_i = np.unique(bxt_i)
    if np.where(bxt_i!=0)[0].size != 0:
        bxt = 0
        for k in range(0, np.size(bxt_i)):
            try:
                bxt = bxt + np.exp(-bxt_i[k])
            except:
                print('error finding exponential')
        bxt = -np.log(bxt)
    else:
        bxt = 0

    if bxt < 0:
        print('Barrier less than 0')
        print(bxt_i,bxt)
        for i in range(np.size(piRobot)):
            print(piRobot[i].hxt)
    return bxt, piRobot

def partialTotalBarrier(piDis):
    bxt_i = []
    for i in range(np.size(piDis,0)):
        if piDis[i].bxt_i is not None:
            bxt_i.append(piDis[i].bxt_i)

    bxt_iNew = []
    if np.size(bxt_i) > 1 and np.where(np.array(bxt_i)==0)[0].size != 0:
        for k in range(0, np.size(bxt_i)):
            if bxt_i[k] != 0 and bxt_i[k] is not None:
                bxt_iNew.append(bxt_i[k])

        bxt_i = bxt_iNew

    bxt_i = np.unique(bxt_i)

    if np.where(bxt_i!=0)[0].size != 0:
        bxt = 0
        for k in range(0, np.size(bxt_i)):
            bxt = bxt + np.exp(-bxt_i[k])
        bxt = -np.log(bxt)
    else:
        bxt = 0

    return bxt

def evBarrier(pi, t, x, xR ,wall,roadmap, preF,linear):
    buffer = 10
    if not linear and len(pi.dir) > 1:
        x[0] += .1 * np.cos(x[2])
        x[1] += .1 * np.sin(x[2])
    hxt = eval(pi.hxt)
    if pi.hxte == []:
        pi.hxte = hxt + buffer
    pi.hxtR = hxt
    pi.xCalc = x
    # print('hxt: ', hxt, 'hxte: ', pi.hxte, 'time: ', t-pi.t_e-pi.a)
    # if theres only one bound for the safe-set

    if np.size(np.where(pi.signFS[0] != 0)) == 1:
        try:
            bxt_i = eval(pi.cbf)
        except:
            print('here')
        bxt_i = bxt_i - hxt

        if 'x[2]' in pi.hxt:
            bxt_i = bxt_i * 2

    else:
        p = np.asarray([float(p[0]), float(p[-1])])
        valP = [1.1 * p[0], (p[0] + p[1]) / 2, .9 * p[1]]
        coeff = np.polyfit(valP, [0, 1, 0], 2)
        bxt_i = (t - b) + coeff[0] * eval(funcOf)**2 + coeff[1] * eval(funcOf) + coeff[2]

    if not linear and len(pi.dir) > 1:
        x[0] -= .1 * np.cos(x[2])
        x[1] -= .1 * np.sin(x[2])

    pi.bxt_i = bxt_i

    return pi

def alBarrier(pi, t, x, xR ,wall, roadmap, preF):
    p = pi.p
    # Need to recalculate wall position because it may change for partials
    if 'wall' in pi.hxt:
        tempPos = x[3*(pi.robotsInvolved[0]-1):3 * (pi.robotsInvolved[0] - 1) + 2]
        wallTemp = lineseg_dists(tempPos, roadmap.map[:,0:2], roadmap.map[:,2:])
        wall[2*(pi.robotsInvolved[0]-1):2 * (pi.robotsInvolved[0] - 1) + 2] = wallTemp
    # if theres only one bound for the safe-set
    signF = pi.signFS[0]
    # compute the barrier function
    valOfFunc = eval(pi.hxt)
    bxt_i = 1 * signF * (-p + eval(pi.hxt))
    bxt_i = 1 * signF * (-p + eval(pi.hxt))**3

    # This is optional. For speed, if the robot is "far enough" from violation, we will ignore this. Can
    # be commented out
    if signF == -1:
        if valOfFunc < (signF*.7+p):
            bxt_i = None
    else:
        if p >= 1:
            if valOfFunc > (signF*1+p):
                bxt_i = None
        else:
            if valOfFunc > (signF * 3*p):
                bxt_i = None
    if bxt_i is not None:
        bxt_i = 1*bxt_i
    pi.bxt_i = bxt_i


    return pi

def partials(piRobot, x, xR, t, wall, roadmap, preF,bxtx,linear):
    delt = .01
    piRobotRef = copy.deepcopy(piRobot)
    xRef = copy.deepcopy(x)
    newx = copy.deepcopy(x)
    bPartialX = []
    # piRobotRef = copy.deepcopy(piRobot)
    for i in range(0,np.size(x)):
        piDis = []
        newx[i] = newx[i] + delt
        for k in range(np.size(piRobot)):
            pTemp = barrier(piRobot[k], newx, xR, t, wall, roadmap, 0,linear)
            piDis.append(pTemp)
        bTemp = partialTotalBarrier(piDis)
        bPartialX.append((bTemp - bxtx)/delt)
        piRobot = copy.deepcopy(piRobotRef)
        newx = copy.deepcopy(xRef)

    newt = t
    bPartialT = []
    for i in range(0,np.size(t)):
        newt = newt + delt
        piDis = []
        for k in range(np.size(piRobot)):
            piDis.append(barrier(piRobot[k], newx, xR, newt, wall, roadmap, 0,linear))
        bTemp = partialTotalBarrier(piDis)
        bPartialT.append((bTemp - bxtx)/delt)
        newt = t

    return bPartialX, bPartialT

def feedbackLin(vx, vy, theta, epsilon):
    R = np.array([[np.asscalar(np.cos(theta)), np.asscalar(np.sin(theta))], \
                      [np.asscalar(-np.sin(theta)), np.asscalar(np.cos(theta))]])
    dirVelocities = np.array([[vx], [vy]])
    rot = np.array([[1.0, 0.0], [0.0, 1/epsilon]])

    fVelAngVel = np.dot(np.dot(rot, R),dirVelocities).T[0]

    return fVelAngVel

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

def lineseg_dists(p, a, b):
    """Cartesian distance from point to line segment

    https://stackoverflow.com/questions/27161533
    /find-the-shortest-distance-between-a-point-and-line-segments-not-line

    Edited to support arguments as series, from:
    https://stackoverflow.com/a/54442561/11208892

    Args:
        - p: np.array of single point, shape (2,) or 2D array, shape (x, 2)
        - a: np.array of shape (x, 2)
        - b: np.array of shape (x, 2)
    """
    # normalized tangent vectors
    d_ba = b - a
    d = np.divide(d_ba, (np.hypot(d_ba[:, 0], d_ba[:, 1])
                           .reshape(-1, 1)))

    # signed parallel distance components
    # rowwise dot products of 2D vectors
    s = np.multiply(a - p, d).sum(axis=1)
    t = np.multiply(p - b, d).sum(axis=1)

    # clamped parallel distance
    h = np.maximum.reduce([s, t, np.zeros(len(s))])

    # perpendicular distance component
    # rowwise cross products of 2D vectors
    d_pa = p - a
    c = d_pa[:, 0] * d[:, 1] - d_pa[:, 1] * d[:, 0]

    allDists = np.hypot(h, c)
    closestWall = np.argmin(allDists)
    closestP, dist = distWall(a[closestWall],b[closestWall],p)
    return closestP