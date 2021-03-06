import numpy as np
import matrixDijkstra
import re
import networkx as nx
from itertools import islice
from itertools import permutations
import copy
import time

class activateProp:
    def __init__(self, State, preF, conditions,input_,getNom):
        self.State = State.State #State of buchi
        self.M = State.M #Number of robots
        self.props = State.props # Proposition values
        self.accepting_states = State.accepting_states # set of accepting states
        self.graph = State.graph # transitions from node to node
        self.controllableProp = State.controllableProp # set of controllable propositions
        self.uncontrollableProp = State.uncontrollableProp # Set of uncontrollable propositions
        self.propositions = State.propositions # propositions to evaluate
        self.props2Activate = [] # preallocate activated propositions
        self.currState = [] # Pre-allocate current state
        self.locOfUncontrollable = [list(self.propositions).index(s) for s in list(self.uncontrollableProp)]
        self.input = []
        self.maxV = State.maxV
        self.weights = [5,.75] #What is already true, what is false right now
        self.preF = preF
        self.conditions = conditions
        self.robustRef = []
        self.distFromSafeRef = []
        self.input = input_
        self.getNom = getNom
        np.seterr(divide='ignore')

    def activate(self,currState,conditions,pos,posRef,t,preF):
        props = self.props
        wall = self.State.wall
        self.conditions = conditions
        self.preF = preF

        if self.preF == 1:
            for i in range(np.size(self.uncontrollableProp)):
                propName = self.uncontrollableProp[i]
                exec('props.' + propName + ' = ' + str(self.conditions[i]))

        #find the transition that is being made based on the conditions
        conditions = self.State.State[currState].cond
        evalCond = eval(','.join(conditions), {'__builtins__': None}, {'props': props})
        evalCond = np.multiply(evalCond,1)

        result = np.array(self.State.State[currState].result)
        potentialNextStates = result[np.where(evalCond == 1)[0]]

        # First find all accepting states that can be reached given uncontrollable propositions
        reachableAcceptCycle = self.findAcceptingCycle()

        paths2Consider = []
        for i in range(np.size(potentialNextStates)):
            potState = potentialNextStates[i]
            for j in range(np.size(reachableAcceptCycle)):
                allPaths = self.State.nRoutes[potState][reachableAcceptCycle[j]]
                paths2Consider.append(allPaths)

        paths2Consider = [item for sublist in paths2Consider for item in sublist]
        # We only care about the first transition for now
        row2Del = []
        for s in range(np.size(paths2Consider, 0)):
            if np.size(paths2Consider[s]) > 2:
                row2Del.append(s)
            if np.size(paths2Consider[s]) < 2:
                paths2Consider[s] = [paths2Consider[s][0], paths2Consider[s][0]]
            else:
                paths2Consider[s] = paths2Consider[s][0:2]
        paths2Consider = np.asarray(paths2Consider)
        paths2Consider = np.delete(paths2Consider,row2Del,0)
        paths2Consider = np.unique(paths2Consider, axis=0)

        # Compute robustness for each path
        # first find all activated propositions for all transitions
        allTransitionsWithState = np.empty((1, len(self.controllableProp) + 1), dtype=int)
        allTransitionsWithNextState = np.empty((1, len(self.controllableProp) + 1), dtype=int)

        propRef = []
        for i in range(np.size(paths2Consider, 0)):
            results = np.array(self.State.State[paths2Consider[i][0]].result)
            try:
                indCondToTran = np.where(results == paths2Consider[i][1])[0]
            except:
                indCondToTran = np.where(results == paths2Consider[i][0])[0]

            transitionOptions = self.State.State[paths2Consider[i][0]].condCNF[indCondToTran[0]]
            possibleTransitions = np.asarray(transitionOptions)[:, self.locOfUncontrollable]
            acceptable = np.where((possibleTransitions == self.input).all(axis = 1))[0]
            try:
                transitionOptions = np.unique(np.asarray(transitionOptions)[acceptable, len(self.uncontrollableProp):],
                                          axis=0)
            except:
                pass

            if np.size(acceptable != 0):
                if np.size(transitionOptions,0) != 0:
                    stateRef = paths2Consider[i][0] * np.ones((np.size(transitionOptions, 0), 1), dtype=int)
                    nextRef = paths2Consider[i][1] * np.ones((np.size(transitionOptions, 0), 1), dtype=int)
                    transitionPot = np.hstack((transitionOptions,nextRef))
                    transitionOptions = np.hstack((transitionOptions, stateRef))
                    allTransitionsWithState = np.vstack((allTransitionsWithState, transitionOptions))
                    allTransitionsWithNextState = np.vstack((allTransitionsWithNextState,transitionPot))
                    for j in range(np.size(transitionOptions, 0)):
                        propRef.extend([i for i, x in enumerate(transitionOptions[j, :-1]) if x == 1])

        allTransitions = np.unique(allTransitionsWithState[1:, :-1], axis=0)
        allTransitionsWithState = allTransitionsWithState[1:, :]
        # check if any propositions have until tag
        allTransitions = self.checkUntil(allTransitions)

        # first we choose the transition with the most infinities. this is the highest robustness
        numMax = np.count_nonzero(np.isinf(allTransitions), axis=1)
        maxLoc = np.argwhere(numMax == np.amax(numMax)).ravel()
        if np.size(maxLoc) == 1:
            # only one maximum robustness
            trans2Make = allTransitions[maxLoc[0]]
            if np.all(trans2Make == np.inf):
                self.robustness = []
            else:
                self.pickTransition(np.array([trans2Make]), pos, props, t, wall, posRef)
        else:
            # Need to break the tie by computing robustness for those that are tied
            allTransMax = allTransitions[maxLoc, :]
            trans2Make = self.pickTransition(allTransMax, pos, props, t, wall, posRef)

        stateLoc = np.where((allTransitions == trans2Make).all(axis=1))[0][0]

        self.currState = int(allTransitionsWithState[stateLoc, -1])

        propsLoc = (np.where(trans2Make == 1)[0]).tolist()
        self.props2Activate = [self.controllableProp[element] for element in propsLoc]

        return self

    def findAcceptingCycle(self):
        reachableAccepting = []
        for j in range(np.size(self.State.acceptingWithCycle)):
            transToAccept = self.State.acceptingWithCycle[j]
            comeFrom = np.where(self.graph[:,transToAccept] == 1)[0]
            for k in range(np.size(comeFrom)):
                stateFrom = comeFrom[k]
                idOfResult = np.where(self.State.State[stateFrom].result == transToAccept)[0][0]
                condOfI = np.asarray(self.State.State[stateFrom].condCNF[idOfResult])

                # New stuff
                condOfI2 = condOfI[:,self.locOfUncontrollable]
                condOfI2 = np.unique(condOfI2, axis=0)
                acceptable = np.where((condOfI2 == self.input).all(axis=1))[0]
                if np.size(acceptable != 0):
                    reachableAccepting.append(transToAccept)
                    break

            # allConds = allConds[:,self.locOfUncontrollable]
            # allConds = np.unique(allConds, axis = 0)
            # acceptable = np.where((allConds == self.input).all(axis=1))[0]
            # if np.size(acceptable != 0):
            #     reachableAccepting.append(transToAccept)

        return reachableAccepting

    def pickTransition(self,allTransMax,pos,props,t,wall,posRef):
        allRobust = []
        allRobustId = []
        if np.size(self.robustRef) == 0:
            robustRef = [None] * np.size(self.State.phi)
            distFromSafeRef = [None] * np.size(self.State.phi)
        else:
            robustRef = self.robustRef
            distFromSafeRef = self.distFromSafeRef

        for i in range(np.size(allTransMax,0)):
            indOfActive = np.where(allTransMax[i,:]==1)[0]
            phiIndex = [self.State.controllablePropOrder[s] for s in indOfActive]
            phi = [self.State.phi[s] for s in phiIndex]
            robustness = []
            ids = []

            satisfiedPhi = [phi[s] for s in range(np.size(phi)) if phi[s].currentTruth]
            unsatisfiedPhi = [phi[s] for s in range(np.size(phi)) if not phi[s].currentTruth]
            for j in range(np.size(satisfiedPhi)):
                if robustRef[satisfiedPhi[j].id] is None:
                    robtemp = self.weights[0] * satisfiedPhi[j].distFromSafe
                    robustRef[satisfiedPhi[j].id] = robtemp
                    robustness.append(robtemp)
                    ids.append(satisfiedPhi[j].id)
                else:
                    robustness.append(robustRef[satisfiedPhi[j].id])
                    ids.append(satisfiedPhi[j].id)
            for ii in range(1,self.M+1):
                phiRobot = []
                for j in range(np.size(unsatisfiedPhi)):
                    if ii in unsatisfiedPhi[j].robotsInvolved:
                        phiRobot.append(unsatisfiedPhi[j])
                        ids.append(unsatisfiedPhi[j].id)
                if len(phiRobot) == 1:
                    if robustRef[phiRobot[0].id] is None:
                        totalDist = phiRobot[0].distFromSafe
                        time2Finish = phiRobot[0].time2Finish
                        if self.preF == 1 and not isinstance(phiRobot[0].inputTime, int):
                            timeRemaining = phiRobot[0].interval[1]
                        else:
                            if isinstance(phiRobot[0].inputTime, float):
                                timeRemaining = (phiRobot[0].interval[1] + phiRobot[0].inputTime) - t
                            else:
                                timeRemaining = phiRobot[0].interval[1] - t
                        timeBuffer = timeRemaining - time2Finish
                        robtemp = self.weights[1] * timeBuffer
                        robustRef[phiRobot[0].id] = robtemp
                        distFromSafeRef[phiRobot[0].id] = totalDist
                        robustness.append(robtemp)
                        ids.append(phiRobot[0].id)
                    else:
                        robustness.append(robustRef[phiRobot[0].id])
                        ids.append(phiRobot[0].id)
                elif len(phiRobot) > 1:
                    #Need to do prioritization
                    possOrders = list(permutations(range(np.size(phiRobot))))
                    robustnessTEMP = []
                    idsTEMP = []
                    for j in range(np.size(possOrders,0)):
                        time2FinishPrev = 0
                        robustRow = []
                        idsRow = []
                        for k in range(np.size(possOrders[j],0)):
                            if k == 0:
                                if robustRef[phiRobot[possOrders[j][k]].id] is None:
                                    totalDist = phiRobot[possOrders[j][k]].distFromSafe
                                    time2Finish = phiRobot[possOrders[j][k]].time2Finish

                                    if self.preF == 1 and t > phiRobot[possOrders[j][k]].interval[1]+phiRobot[possOrders[j][k]].inputTime:
                                        # time passed. full time bound remains
                                        timeRemaining = phiRobot[possOrders[j][k]].interval[1]
                                    elif self.preF == 1:
                                        timeRemaining = (phiRobot[possOrders[j][k]].interval[1] + phiRobot[possOrders[j][k]].inputTime) - t
                                    else:
                                        if isinstance(phiRobot[possOrders[j][k]].inputTime, float):
                                                timeRemaining = (phiRobot[possOrders[j][k]].interval[1] + phiRobot[possOrders[j][k]].inputTime) - t
                                        else:
                                            timeRemaining = phiRobot[possOrders[j][k]].interval[1] - t

                                    timeBuffer = timeRemaining - time2Finish
                                    time2FinishPrev += time2Finish
                                    robtemp = self.weights[1] * timeBuffer
                                    robustRef[phiRobot[possOrders[j][k]].id] = robtemp
                                    distFromSafeRef[phiRobot[possOrders[j][k]].id] = totalDist
                                    robustRow.append(robtemp)
                                    idsRow.append(phiRobot[possOrders[j][k]].id)
                                else:
                                    robustRow.append(robustRef[phiRobot[possOrders[j][k]].id])
                                    idsRow.append(phiRobot[possOrders[j][k]].id)
                            else:
                                posTemp = copy.deepcopy(pos)
                                posTemp[phiRobot[possOrders[j][k]].nom[0, 0:2].astype('int')] = phiRobot[
                                    possOrders[j][k-1]].point
                                nom, costTemp = self.getNom(phiRobot[possOrders[j][k]], posTemp, posRef)
                                distFromSafe2 = phiRobot[possOrders[j][k-1]].distFromSafe + costTemp
                                signF = phiRobot[possOrders[j][k]].signFS[0]
                                totalDist = distFromSafe2 + signF * phiRobot[j].p
                                robot = phiRobot[possOrders[j][k]].robotsInvolved[0]
                                maxV = self.maxV[3 * robot - 3]
                                time2Finish = totalDist / maxV
                                if self.preF == 1 and not isinstance(phiRobot[possOrders[j][k]].inputTime, float):
                                    timeRemaining = phiRobot[possOrders[j][k]].interval[1]
                                else:
                                    timeRemaining = (phiRobot[possOrders[j][k]].interval[1] + phiRobot[possOrders[j][k]].inputTime) - t
                                timeBuffer = timeRemaining - (time2Finish + time2FinishPrev)
                                time2FinishPrev += time2Finish
                                idsRow.append(phiRobot[possOrders[j][k]].id)
                                robustRow.append(self.weights[1] * timeBuffer)
                        robustnessTEMP.append(robustRow)
                        idsTEMP.append(idsRow)
                    minFound = 0
                    robustCopy = copy.deepcopy(robustnessTEMP)
                    while minFound == 0:
                        minInd = []
                        for j in range(np.size(robustCopy, 0)):
                            minInd.append(np.argmin(robustCopy[j]))
                        # create a vector of the minimum values
                        minResponse = np.empty((1, np.size(minInd)))
                        for j in range(np.size(minInd)):
                            minResponse[0, j] = robustCopy[j][minInd[j]]
                        if len(minResponse[0]) != 0:
                            maxVal = np.amax(minResponse[0])
                            maxInd = np.where(minResponse[0] == maxVal)[0]
                            if np.size(maxInd) == 1:
                                minFound = 1
                                indOfTrans = maxInd[0]
                            else:
                                robustCopy = [robustCopy[j] for j in maxInd]
                                robustCopy = robustCopy[maxInd, :]
                                [robustCopy[j].remove(maxVal) for j in range(np.size(robustCopy, 0))]
                                robustCopy = [j for j in robustCopy if j]
                        else:
                            minFound = 1
                            indOfTrans = 0
                    robustness.extend(robustnessTEMP[indOfTrans])
                    ids.extend(idsTEMP[indOfTrans])

            allRobust.append(robustness)
            allRobustId.append(ids)

        #Choose transition that has the maximum minimum
        minFound = 0
        allRobustCopy = copy.deepcopy(allRobust)
        allTransMaxCopy = copy.deepcopy(allTransMax)
        if np.size(allTransMax, 0) == 1:
            minFound = 1
            indOfTrans = 0
        while minFound ==0:
            minInd = []
            for i in range(np.size(allRobustCopy, 0)):
                minInd.append(np.argmin(allRobustCopy[i]))
            #create a vector of the minimum values
            minResponse = np.empty((1,np.size(minInd)))
            for i in range(np.size(minInd)):
                minResponse[0,i] = allRobustCopy[i][minInd[i]]
            if len(minResponse[0]) != 0:
                maxVal = np.amax(minResponse[0])
                maxInd = np.where(minResponse[0] == maxVal)[0]
                if np.size(maxInd) == 1:
                    minFound = 1
                    indOfTrans = maxInd[0]
                else:
                    allRobustCopy = [allRobustCopy[i] for i in maxInd]
                    allTransMaxCopy = allTransMaxCopy[maxInd, :]
                    [allRobustCopy[i].remove(maxVal) for i in range(np.size(allRobustCopy, 0))]
                    allRobustCopy = [i for i in allRobustCopy if i]
            else:
                minFound = 1
                indOfTrans = 0

        trans2Make = allTransMaxCopy[indOfTrans, :]
        self.robustness = allRobustCopy[indOfTrans]
        self.robustness = allRobust[np.where(np.all(allTransMax == trans2Make,axis=1))[0][0]]
        # if any(x < 0 for x in self.robustness) and t > 15:
        #     print('here')
        self.ids = allRobustId[np.where(np.all(allTransMax == trans2Make,axis=1))[0][0]]
        self.robustRef = robustRef
        self.distFromSafeRef = distFromSafeRef
        return trans2Make

    def checkUntil(self,allTransitions):
        allUnt = []
        for i in range(np.size(self.State.phi)):
            if self.State.phi[i].phiUntil != "":
                propOfI = self.State.phi[i].prop_label
                propOfI2 = self.State.phi[i].phiUntil
                ia = self.State.controllablePropOrder[list(self.State.controllableProp).index(propOfI)]
                ia2 = []
                splitPhiUntil = re.split('(\||\&)', propOfI2)
                for j in range(np.size(splitPhiUntil)):
                    if '&' not in splitPhiUntil[j] and '|' not in splitPhiUntil[j]:
                        predOfI = re.search('(?<=props\.).+?(?=(\)|\s))', splitPhiUntil[j])[0]
                        locOfPred = list(self.State.controllableProp).index(predOfI)
                        #ia2.append(self.State.controllablePropOrder[locOfPred])
                        ia2.append(locOfPred)

                for j in range(np.size(allTransitions, 0)):
                    actEv = 0
                    for jj in range(np.size(ia2)):
                        if allTransitions[j, ia2[jj]] == 1:
                            actEv = 1
                            break
                    if actEv == 1 and allTransitions[j, ia] == np.inf:
                        allTransitions[j, ia] = 1

        return allTransitions

    def intersectPoint(self, x1, y1, x2, y2, x3, y3, x4, y4):
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        with np.errstate(divide='ignore', invalid='ignore'):
            ua = np.divide(((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)), denom)
            ub = np.divide(((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)), denom)

            isect = (ua>=0)*(ub>=0)*(ua<=1)*(ub<=1)

        return isect

    def distWall(self, p1, p2, pt):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        t = ((pt[:,0] - p1[0]) * dx + (pt[:,1] - p1[1]) * dy) / (dx ** 2 + dy ** 2)

        if dx == 0 and dy == 0:
            closestP = p1
            dx = pt[0] - p1[0]
            dy = pt[1] - p1[1]
            dist = np.sqrt(dx ** 2 + dy ** 2) * np.ones((1,np.size(pt,0)))[0]
        else:
            dist = np.zeros((1,np.size(pt,0)))[0]

        try:
            indL = np.where(t<0)[0]
            dx = pt[indL,0] - p1[0]
            dy = pt[indL,1] - p1[1]
            dist[indL] = np.sqrt(dx ** 2 + dy ** 2)
        except:
            pass

        try:
            indG = np.where(t>0)[0]
            dx = pt[indG,0] - p1[0]
            dy = pt[indG,1] - p1[1]
            dist[indG] = np.sqrt(dx ** 2 + dy ** 2)
        except:
            pass

        try:
            indM = np.where((t >= 0) & (t <= 1))[0]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            closestP = np.array([p1[0] + t[indM] * dx, p1[1] + t[indM] * dy])
            dx = pt[indM,0] - closestP[0]
            dy = pt[indM,1] - closestP[1]
            dist[indM] = np.sqrt(dx ** 2 + dy ** 2)
        except:
            pass

        return dist
