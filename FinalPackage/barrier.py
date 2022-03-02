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
        initBuffer = 100
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

    dir = re.findall('(?=\().+?(?=\*)',funcOf)
    phi.bxt_i = bxt_i
    if not preF:
        try:
            if numPos == 1:
                phi.dist = eval(funcOf) - p
                totaldist = phi.dist
                iabs = np.size(re.findall('abs',funcOf))
                inde = re.findall('(?<=\[).+?(?=\])',funcOf)
                for i in range(3):
                    if str(i) in inde:
                        if iabs < 1:
                            phi.nom[1, i] = -signF * totaldist
                        else:
                            phi.nom[1, i] = np.abs(-1 * signF * totaldist)
                    else:
                        phi.nom[1,i] = 0
            else:
                dir[0] = re.findall('(?<=\().+(?<=\))',dir[0])[0]
                dir[1] = re.findall('(?=\().+(?<=\))', dir[1])[0]

                # directionSplit = np.empty([1, np.size(dir)], dtype=object)
                # directionSplit[0, 0] = re.split('[\+,\-]', dir[0])[1]
                # directionSplit[0, 0] = '(' + directionSplit[0, 0]
                # directionSplit[0, 1] = re.split('[\+,\-]', dir[1])[1]
                # directionSplit[0, 1] = '(' + directionSplit[0, 1]

                vals = [-eval(elem, {'__builtins__': None}, {'pos': pos, 'np': np, 'posRef': posRef}) for elem in dir]
                phi.nom.astype('float')
                phi.nom[1, 0:2] = vals
                phi.dist = eval(funcOf)

        except:
            print('Couldnt find nominal controller. potential error')

    phi.bxt_i = bxt_i
    return bxt_i, phi


def alBarrier(State, phi, t, Ts, a, b, pos, posRef, tmax, unt,wall,preF):
    p = phi.p
    funcOf = phi.funcOf
    # If theres a wall we should find the closest position
    # if 'wall' in funcOf:
    #     for i in range(int(np.size(pos) / 3)):
    #         if i == 0:
    #             wall = findPoint(State.map, pos[3 * i:3 * i + 2])
    #         else:
    #             wall = np.append(wall, findPoint(State.map, pos[3 * i:3 * i + 2]))

    # if theres only one bound for the safe-set
    if np.size(np.where(phi.signFS[0] != 0)) == 1:
        signF = phi.signFS[0]

        # compute the barrier function
        bxt_i = signF * (-p + eval(funcOf))

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
                if eval(funcOf) > (signF * 3*p):
                    bxt_i = 0
                else:
                    bxt_i = bxt_i

        bxt_i = 5 * bxt_i
        phi.bxt_i = bxt_i

    else:
        valP = [1.4 * p[0], (p[0] + p[1]) / 2, .6 * p[1]]
        coeff = np.polyfit(valP,[0,1,0],2)
        bxt_i = coeff[0]*eval(funcOf)**2 + coeff[1]*eval(funcOf) + coeff[2]
    # dir = re.findall('(?=\().+?(?=\*)', funcOf)

    if bxt_i != 0 and not preF:
        try:
            # dir[0] = re.findall('(?<=\().+(?<=\))',dir[0])[0]
            # dir[1] = re.findall('(?=\().+(?<=\))', dir[1])[0]

            # if 'wall' in funcOf:
            #     vals = [-eval(elem, {'__builtins__': None}, {'pos': pos, 'np': np, 'posRef': posRef, 'wall': wall}) for elem in dir]
            # else:
            #     vals = [-eval(elem, {'__builtins__': None}, {'pos': pos, 'np': np, 'posRef': posRef}) for elem in dir]

            phi.nom.astype('float')
            phi.nom[1, 0:2] = [0,0]

        except:
            print('Couldnt find nominal controller. potential error')

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

def findPoint(map,pos):
    wallPoints = np.empty((1,2))
    wallDistances = np.empty((1,1))
    mapNew = np.multiply((map[:, 0:2] - pos), (map[:, 0:2] - pos))
    mapAdd = np.sqrt(mapNew[:, 0] + mapNew[:, 1])
    maxInd = np.size(mapAdd)-1
    if maxInd > 20:
        maxInd = 20
    idx = np.argpartition(mapAdd, maxInd)
    idx = idx[:maxInd]
    map2Consider = map[idx]

    for k in range(np.size(map2Consider, 0)):
        p1 = map2Consider[k, 0:2]
        p2 = map2Consider[k, 2:4]
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