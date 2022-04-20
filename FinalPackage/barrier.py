import numpy as np
import re
import matrixDijkstra

def barrier(State,pos, posStart,posRef, t, Ts, phi, tmax, hz, wall, preF):
    # Initialize the candidate barrier functions
    bxt_i = []
    bxt_eventually = []
    activatedProps = []
    props = State.props
    # Loop through the STL Formulas
    for i in range(0, np.size(phi)):
        # for the stl formula find the bounds of the interval.Default is[0-inf]
        a = phi[i].interval[0]
        b = phi[i].interval[1]

        # Determine whether until, eventually, or always
        type = phi[i].type

        # if implication exists make sure timing bounds is correct
        if phi[i].implies == 1:
            inputTime = phi[i].inputTime
        else:
            inputTime = 0

        if type == 'ev':
            # If the interval has passed, then the barrier function doesnt matter
            if t >= a + inputTime and t <= b + inputTime:
                bxtTemp, phi[i] = evBarrier(State, phi[i], t, Ts, a, b, inputTime, pos, posRef, posStart, hz,wall,preF)
                bxt_i.append(bxtTemp)
                activatedProps.append(phi[i].prop_label)
                bxt_eventually.append(bxtTemp)
            else:
                phi[i].bxt_i = 0

        if type == 'alw':
            if phi[i].until == 1:
                # If phiUntil is True then we should set the time so that the time period has passed.
                isUntTrue = eval(phi[i].phiUntil)
                if isUntTrue == 1 and t >= a:
                    b = t - .1

            if t >= a + inputTime and t <= b + inputTime:
                bxt_i.append(alBarrier(State, phi[i], t, Ts, a, b, pos, posRef, tmax, None,wall,preF))
                activatedProps.append(phi[i].prop_label)



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
            bxt = bxt + np.exp(-bxt_i[k])
        bxt = -np.log(bxt)
    else:
        bxt = 0

    #want to return a matrix of all of the barrier functions for "eventually"
    return bxt, phi, activatedProps, bxt_eventually


def evBarrier(State, phi, t, Ts, a, b, inputTime, pos, posRef, posStart, hz,wall,preF):
    p = phi.p
    funcOf = phi.funcOf
    numPos = np.size(re.findall('pos', funcOf))

    # if theres only one bound for the safe-set
    if np.size(np.where(phi.signFS[0] != 0)) == 1:
        signF = phi.signFS[0]
        if numPos >= 2:
            signF = -1 * signF

        # This only occurs once during initialization to evaluate the initial state
        initBuffer = 10
        if t == Ts:
            if numPos == 1:
                initDist = abs(eval(funcOf) - p)
            else:
                initDist = eval(funcOf)
            bInit = signF * (initDist-p)
            aInit = (signF*p - signF*bInit)/b
            gamInit = (t-(1/hz)-a) * aInit + bInit
            phi.minVel = gamInit/b
        else:
            tempFunc = re.sub('pos\[', 'posStart[', funcOf)
            if numPos == 1:
                initDist = abs(eval(tempFunc) - p) + initBuffer
            else:
                initDist = eval(tempFunc) + initBuffer
            bInit = signF * (initDist - p)
            aInit = (signF * p - signF * bInit) / b
            # gamInit = (t - (1 / hz) - a) * aInit + bInit
            gamInit = (t - a) * aInit + bInit
            phi.minVel = gamInit / b

        # Make the initial barrier function safe set larger to account for any disturbances that can occur or
        # if the robot needs to go backwards
        addBuffer = .5
        b1 = signF * (initDist + addBuffer)
        a1 = (signF * p - signF * b1) / b
        # gam = (t - (1 / hz) - a) * a1 + b1
        gam = (t - a) * a1 + b1
        bxt_i = gam - signF * eval(funcOf)

        bxt_i = ((t - inputTime - a) * (p-initDist))/(b - a) - (p - initDist) + (p - eval(funcOf))


    else:
        p = np.asarray([float(p[0]), float(p[-1])])
        valP = [1.1 * p[0], (p[0] + p[1]) / 2, .9 * p[1]]
        coeff = np.polyfit(valP, [0, 1, 0], 2)
        bxt_i = (t - b) + coeff[0] * eval(funcOf)**2 + coeff[1] * eval(funcOf) + coeff[2]

    phi.bxt_i = bxt_i
    return bxt_i, phi


def alBarrier(State, phi, t, Ts, a, b, pos, posRef, tmax, unt,wall,preF):
    p = phi.p
    funcOf = phi.funcOf
    if 'wall' in phi.funcOf:
        tempPos = pos[3*(phi.robotsInvolved[0]-1):3 * (phi.robotsInvolved[0] - 1) + 2]
        wallTemp = lineseg_dists(tempPos, State.map[:,0:2], State.map[:,2:])
        wall[2*(phi.robotsInvolved[0]-1):2 * (phi.robotsInvolved[0] - 1) + 2] = wallTemp
    # if theres only one bound for the safe-set
    if np.size(np.where(phi.signFS[0] != 0)) == 1:
        signF = phi.signFS[0]

        # compute the barrier function
        bxt_i = 1.3 * signF * (-p + eval(funcOf))

        # This is optional. For speed, if the robot is "far enough" from violation, we will ignore this. Can
        # be commented out
        if signF == -1:
            if eval(funcOf) < (signF*.7+p):
                bxt_i = 0
        else:
            if p >= 1:
                if eval(funcOf) > (signF*1+p):
                    bxt_i = 0
            else:
                if eval(funcOf) > (signF * 8*p):

                    bxt_i = 0
                else:
                    bxt_i = bxt_i

        bxt_i = 15 * bxt_i
        phi.bxt_i = bxt_i

    else:
        valP = [1.4 * p[0], (p[0] + p[1]) / 2, .6 * p[1]]
        coeff = np.polyfit(valP,[0,1,0],2)
        bxt_i = coeff[0]*eval(funcOf)**2 + coeff[1]*eval(funcOf) + coeff[2]

    return bxt_i

def partials(State,pos, posStart, posRef, t, Ts, phi, tmax, hz,bxt_i,wall):
    delt = .01
    newx = list(pos)
    bPartialX = []
    bxt_i = np.asarray(bxt_i)
    phiRef = phi
    for i in range(0,np.size(pos)):
        newx[i] = newx[i] + delt
        btemp, phiTemp, actProp, beven = barrier(State,newx,posStart, posRef, t, Ts, phi, tmax, hz,wall,1)
        bPartialX.append((btemp - bxt_i)/delt)
        newx = list(pos)

    newt = t
    bPartialT = []
    for i in range(0,np.size(t)):
        newt = newt + delt
        btemp, phiTemp,actProp, beven = barrier(State,pos, posStart, posRef, newt, Ts, phi, tmax, hz,wall,1)
        bPartialT.append((btemp - bxt_i)/delt)
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