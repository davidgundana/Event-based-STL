import numpy as np
import re
import helperFuncs
import activateProp as act
from getNom import getNom
import barrier
from customQP import quadprog
import runEvBasedSTL
import time

def synthesis(specattr,potS, roadmap,x,xR, t,maxV,sizeState,sizeU,preFailure,text1, master):
    # Check to see if you care about walls. If you do, find the closest point
    specattr = checkWall(specattr,roadmap,x)

    # Go through all specifications and, if there is implication, log the time that the input became activated.
    # also, if a specification has been satisfied, the input should be reset.
    specattr = trackInputs(specattr, x, xR, t)

    # Evaluate the current state of the environment
    specattr = evalProps(specattr, roadmap, x, xR, t, maxV,sizeU,sizeState)

    # Find Propositions to activate based on the current state and transitiosn to an accepting state
    specattr, props2Activate, potS, _,_ = act.activate(specattr,potS,roadmap, 0,[],x, xR,t, maxV, sizeU)
    print(props2Activate)

    if preFailure:
        # this is for a pre-failure warning. We want to see what happens if things change with the inputs
        conditions = findConditions(specattr,potS)

        potentialConflicts(conditions, specattr, potS, roadmap, maxV, x, xR, t,text1, master,sizeU)
    # Organize abstracted propositions by the activated propositions
    indOfActive = []
    linear = sizeState == sizeU
    for i in range(np.size(specattr)):
        for j in range(np.size(specattr[i].Pi_mu)):
            labelOfI = re.split('\.',specattr[i].Pi_mu[j].prop_label)[-1]
            if labelOfI in props2Activate:
                specattr[i].Pi_mu[j] = barrier.barrier(specattr[i].Pi_mu[j], x, xR, t, specattr[0].wall, roadmap, 0,linear)
                indOfActive.append([i,j])

    nom = np.zeros((1, sizeU))
    for i in range(1, int(np.size(x)/sizeState) + 1):
        bxtx, piRobot = barrier.totalBarrier(specattr, i, indOfActive)
        [bPartialX, bPartialT] = barrier.partials(piRobot, x, xR, t, specattr[0].wall, roadmap, 0,bxtx,linear)
        error = 0
        if np.any(bPartialX):
            if sizeState != sizeU:
                xReference = x[sizeState*(i-1):sizeState*i]
                dyn = np.identity(sizeState)
                dyn = dyn[:,1:]
                dyn[0:3,0:2] = np.array([[np.cos(xReference[2]),0],[np.sin(xReference[2]),0],[0,1]])
                A = -1 * np.dot(np.array(bPartialX), dyn)
            else:
                A = -1 * np.dot(np.array(bPartialX), np.identity(3 * int(np.size(x) / 3)))

            if abs(bPartialT[0]) > 50:
                bPartialT[0] = 0

            alpha = 1
            b = alpha * (bxtx) + bPartialT[0]

            nominals = getAllNoms(piRobot,maxV, sizeU)
            print(b, bxtx, bPartialT)
            nom,error = getControl(nom,nominals,A,b,maxV,i,bPartialX,sizeU,x,sizeState)
    return nom,specattr, error

def getAllNoms(piRobot, maxV,sizeU):
    sumNom = np.zeros((1, sizeU), dtype=float)
    nominals = piRobot[0].nom[0,:]
    for j in range(np.size(piRobot)):
        if np.sum(abs(piRobot[j].nom[1, :])) != 0:
            # velBound = maxV[piRobot[j].nom[0, 0:].astype(int)]
            sumNom += piRobot[j].nom[1,:]

    nominals = np.vstack((nominals, sumNom))
    return nominals

def getControl(nom,nominals,A,b,maxV,i,bPartialX,sizeU,x,sizeState):
    error = 0
    Anew = A
    lbI = -maxV
    ubI = maxV
    if np.any(nominals):
        x0 = nominals[1, :]
    else:
        x0 = np.zeros((np.size(lbI)))
        nominals = np.zeros((2, np.size(lbI)))

    H = 2*np.identity(np.size(Anew))
    # f = np.array([-2 * nominals[1, 0], -2 * nominals[1, 1], -2 * nominals[1, 2], -2 * nominals[1, 3], -2 * nominals[1, 4]]).T
    f = -2*nominals[1,:]

    qp = quadprog(H, f, Anew, b, x0, lbI, ubI,bPartialX,x,sizeState,sizeU)
    nomInd = qp.result.x
    if sizeU == sizeState:
        # convert to v omega
        newNom = helperFuncs.feedbackLin(nomInd[0], nomInd[1], x[2], .1, maxV[0])
        if newNom[1] != 0 and np.abs(nomInd[2]) > 0.00001:
            print('confict')
        nomRet = np.zeros((1,5))
        nomRet[0,1:] = nomInd[-4:]
        if newNom[1] == 0:
            nomRet[0,0] = newNom[0]
        else:
            nomRet[0,0:2] = newNom
        nomInd = nomRet[0]
    # if nomInd[2] != 0:
    #     print('here')
    if not qp.result.success:
        print('Specification violated')
        error = 1
        # nom[0][3 * i - 3] = 0
        # nom[0][3 * i - 2] = 0
        # nom[0][3 * i - 1] = 0
    else:
        if sizeU != sizeState:
            nom[0][sizeU*(i-1):sizeU*i] = nomInd
        else:
            nom[0][(sizeU-1)*(i-1):(sizeU-1)*i] = nomInd

    return nom, error
def checkWall(specattr,roadmap,x):
    found = 0
    for i in range(np.size(specattr)):
        if not found:
            isWall = [re.findall('wall', elem) for elem in specattr[i].parameters]
            flatWall = [item for sublist in isWall for item in sublist]
            if flatWall != []:
                for j in range(int(np.size(x)/3)):
                    if j == 0:
                        wall = findPoint(roadmap,x[3*j:3*j+2])
                    else:
                        wall = np.append(wall,findPoint(roadmap,x[3*j:3*j+2]))

                wall = wall
                found = 1
            else:
                wall = []
        else:
            specattr = wall
        specattr[i].wall = wall
    return specattr

def findPoint(roadmap,x):
    map = roadmap.map
    wallPoints = np.empty((1,2))
    wallDistances = np.empty((1,1))
    for k in range(np.size(map,0)):
        p1 = map[k,0:2]
        p2 = map[k,2:4]
        dist2wall, wall = helperFuncs.distWall(p1,p2,x)
        wallPoints = np.vstack((wallPoints,wall))
        wallDistances = np.vstack((wallDistances,dist2wall))

    wallPoints = wallPoints[1:,:]
    wallDistances = wallDistances[1:,:]

    minDist = np.argmin(wallDistances)

    wall = wallPoints[minDist]

    return wall

def evalProps(specattr, roadmap, x,xR,t,maxV,sizeU,sizeState):
    for i in range(np.size(specattr)):
        props = specattr[i].props
        # Find the values of all parameters
        if len(specattr[i].wall) != 0:
            wall = specattr[i].wall
        else:
            wall = []
        valuesOfControl = eval(','.join(specattr[i].parameters), {'__builtins__': None},
                               {'x': x, 'np': np, 'xR': xR, 'wall': wall})
        # change from true/false to 1/0
        valuesOfControl = np.multiply(valuesOfControl,1)
        if np.size(valuesOfControl) == 1:
            valuesOfControl = np.asarray([valuesOfControl])
        # Assign all for the values to the propositions
        for j in range(np.size(specattr[i].controllableProp)):
            # propName = specattr[i].controllableProp[specattr[i].controllablePropOrder[j]]
            exec('props.pred' + str(j) + ' = ' + str(valuesOfControl[j]))

        specattr[i].props = props
        for j in range(len(specattr[i].Pi_mu)):
            specattr[i].Pi_mu[j].currentTruth = eval(specattr[i].Pi_mu[j].prop_label, {'__builtins__': None},
                                 {'props': props})
            if specattr[i].Pi_mu[j].type == 'ev':
                if not specattr[i].Pi_mu[j].t_e == []:
                    inputTime = specattr[i].Pi_mu[j].t_e
                    if t >= specattr[i].Pi_mu[j].a + inputTime and t <= specattr[i].Pi_mu[j].b + inputTime and specattr[i].Pi_mu[j].currentTruth:
                        specattr[i].Pi_mu[j].satisfied = 1
                try:
                    if specattr[i].Pi_mu[j].satisfied:
                        exec(specattr[i].Pi_mu[j].prop_label + '=1')
                except:
                    pass

            if specattr[i].Pi_mu[j].currentTruth:
                nomR = np.zeros((1, 3), dtype=float)[0]
                cost = eval(specattr[i].Pi_mu[j].hxt)
                signF = -specattr[i].Pi_mu[j].signFS[0]
                specattr[i].Pi_mu[j].distFromSafe = cost + signF * specattr[i].Pi_mu[j].p
            else:
                rob =  specattr[i].Pi_mu[j].robotsInvolved[0]
                nomR, cost = getNom(specattr[i].Pi_mu[j], roadmap, x, xR,maxV[sizeU * rob - sizeU:sizeU*rob],sizeU,sizeState)
                signF = specattr[i].Pi_mu[j].signFS[0]
                specattr[i].Pi_mu[j].distFromSafe = cost + signF *  specattr[i].Pi_mu[j].p
                specattr[i].Pi_mu[j].time2Finish = specattr[i].Pi_mu[j].distFromSafe / maxV[sizeU * rob - sizeU]
            specattr[i].Pi_mu[j].nom[1, 0:np.size(nomR)] = nomR
    return specattr

def trackInputs(specattr,x,xR,t):
    for i in range(np.size(specattr)):
        props = specattr[i].props
        specattr[i].inputVal = specattr[i].input[::2].astype(int)

        for j in range(np.size(specattr[i].uncontrollableProp)):
            exec('props.' + str(specattr[i].uncontrollableProp[j]) + ' = ' + str(specattr[i].inputVal[j]))

        for j in range(np.size(specattr[i].Pi_mu)):
            if specattr[i].Pi_mu[j].implies == 1:
                #evaluate the implication message
                messTrue = eval(specattr[i].Pi_mu[j].alpha[0])
                if messTrue:
                    inpNum = int(re.split('t',specattr[i].Pi_mu[j].alpha[0])[1])
                    inpNums = []
                    for k in range(np.size(specattr[i].uncontrollableProp)):
                        inpNums.append(int(re.split('t',specattr[i].uncontrollableProp[k])[1]))
                    minInpNum = min(inpNums)
                    specattr[i].Pi_mu[j].t_e = specattr[i].input[2*(inpNum-minInpNum)+1]
        specattr[i].props = props
    return specattr

def findConditions(specattr,potS):
    allConditions = []
    for a in range(np.size(specattr)):
        #this is for a pre-failure warning. We want to see what happens if things change with the inputs
        numUncon = np.size(specattr[a].uncontrollableProp)
        conditions = np.empty((1,numUncon))
        # From your current state, evaluate all possible transitions and find all of the combinations of
        # inputs that are possible
        simpleInput = np.zeros((1, int(np.size(specattr[a].input) / 2)))
        for i in range(int(np.size(specattr[a].input) / 2)):
            simpleInput[0, i] = specattr[a].input[2 * i]
        simpleInput = simpleInput[0][:np.size(specattr[a].uncontrollableProp)]
        for i in range(specattr[a].buchiStates[potS[a]].condCNF.__len__()):
            condCNF = specattr[a].buchiStates[potS[a]].condCNF
            for j in range(condCNF[i].__len__()):
                #specVal = np.array([int(s) for line in condCNF[i][j] for s in line.split()])
                specVal = np.array(condCNF[i][j])
                # We are only interested in the uncontrollable propositions (inputs)
                specVal = specVal[0:numUncon]
                canAppend = 0
                for j in range(np.size(np.where(simpleInput == 1)[0])):
                    if np.size(np.where(simpleInput == 1)[0]) <= np.size(np.where(specVal == 1)[0]):
                        locOfInp = np.where(specVal == 1)[0]
                        if np.size(locOfInp) != 0:
                            ind = simpleInput[np.where(specVal == 1)[0][j]]
                            if int(ind) == 1:
                                canAppend += 1
                if canAppend == np.size(np.where(simpleInput == 1)[0]):
                    conditions = np.vstack((conditions, specVal))

        # The first row is zero because of how we appended. Also, we only want the unique values to increase speed
        conditions = conditions.astype(int)
        conditions = conditions[1:,:]
        conditions = np.unique(conditions, axis = 0)
        # try:
        #     indToDel = np.where((conditions == specattr[a].inputVal).all(axis=1))[0][0]
        #     conditions = np.delete(conditions, indToDel, 0)
        # except:
        #     pass
        allConditions.append(conditions)

    return allConditions

def potentialConflicts(conditions,specattr, potS,roadmap, maxV,x,xR,t,text1, master,sizeU):
    # Lets check things that can be activated for each robot
    # These are the indexes of the inputs for the robot
    for i in range(np.size(specattr)):
        badConditions = []
        robustRef = []
        while np.size(conditions[i],0) > 0:
            try:
                _, _, _, robustness,ids = act.activate([specattr[i]], [potS[i]], roadmap, 1, conditions[i][0], x, xR, t,maxV,sizeU)
                if len(robustness) > 0:
                    if any(x < 0 for x in robustness):
                        badConditions.append(conditions[i][0])
                        robustRef.append(robustness)
                        actInd = np.where(conditions[i][0]==1)[0]
                        indToDel = []
                        for i in range(np.size(actInd)):
                            indToDel.extend(np.where(conditions[i][:, actInd[i]] == 1)[0].tolist())
                        indToDel = np.unique(indToDel)
                        if np.size(conditions[i], 0) > 0:
                            conditions[i] = np.delete(conditions[i], indToDel, 0)
                if np.size(conditions[i], 0) > 0:
                    conditions[i] = np.delete(conditions[i],0,0)
            except Exception as e:
                if np.size(conditions[i],0) > 0:
                    conditions[i] = np.delete(conditions[i],0,0)
                pass

        # the badConditions variable shows which combinations of inputs result in a negative robustness score
        # for a proposition

        currInput = specattr[i].inputVal
        currInput = currInput[:np.size(specattr[i].uncontrollableProp)]
        locOfCurrent = np.where(currInput == 1)[0]
        if badConditions == []:
            text1.configure(state='normal')
            text1.delete("end-1l", "end")
            text1.configure(state='disabled')
            runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData, text1, master, '\n')
        for j in range(np.size(badConditions, 0)):
            locOfTrue = np.where(badConditions[j] == 1)[0]
            msg = '\nIf '
            if not np.array_equal(locOfCurrent, locOfTrue):
                for k in range(np.size(locOfTrue)):
                    if k > np.size(locOfTrue) - 1:
                        msg += 'and ' + specattr[i].uncontrollableProp[locOfTrue[k]] + ' '
                    else:
                        msg += specattr[i].uncontrollableProp[locOfTrue[k]] + ', '
                if np.size(locOfTrue) == 1:
                    msg += 'is sensed now, the specification may be violated'
                else:
                    msg += 'are sensed now, the specification may be violated'

            indOfNeg = np.where(np.asarray(robustRef[i]) < 0)[0]
            for j in range(np.size(indOfNeg)):
                phiId = ids[indOfNeg[j]]
                timeNeeded = round(-1*robustRef[i][indOfNeg[j]]/0.75,2)
                msg += '. To make robustness positive the task ' + specattr[i].Pi_mu[phiId].hxt + ' needs ' + str(timeNeeded) + ' more seconds to be completed'
            text1.configure(state='normal')
            text1.delete("end-1l", "end")
            text1.configure(state='disabled')
            runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData, text1, master, msg)
