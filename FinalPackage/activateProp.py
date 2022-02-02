import numpy as np
import matrixDijkstra
import re
import networkx as nx
from itertools import islice
import copy
import time

class activateProp:
    def __init__(self, State, currState, pos, posRef, t, maxV, M, preF, conditions):
        self.State = State #State of buchi
        self.M = M #Number of robots
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
        self.maxV = maxV
        self.weights = [.25,.75] #What is already true, what is false right now
        self.preF = preF
        self.conditions = conditions
        np.seterr(divide='ignore')
        # self.activate(currState,pos,posRef,t)

    def activate(self,currState,props,wall,input,conditions,pos,posRef,t,preF):
        # t2 = time.time()
        self.input = input
        self.conditions = conditions
        self.preF = preF

        if self.preF == 0:
            # If we are not checking for pre-failure warnings we need the real value of the uncontrollable props
            for i in range(np.size(self.uncontrollableProp)):
                propName = self.uncontrollableProp[i]
                exec('props.' + propName + ' = ' + str(self.input[i]))
        else:
            for i in range(np.size(self.uncontrollableProp)):
                propName = self.uncontrollableProp[i]
                exec('props.' + propName + ' = ' + str(self.conditions[i]))


        #find the transition that is being made based on the conditions
        conditions = self.State.State[currState].cond
        evalCond = eval(','.join(conditions), {'__builtins__': None}, {'props': props})
        evalCond = np.multiply(evalCond,1)

        result = np.array(self.State.State[currState].result)
        if np.sum(evalCond) != 0:
            potentialNextStates = result[np.where(evalCond == 1)[0]]
        else:
            for i in range(len(self.State.phi)):
                if self.State.phi[i].type == 'alw':
                    if t not in range(self.State.phi[i].interval[0], self.State.phi[i].interval[1]):
                        propName = self.State.phi[i].prop_label[0]
                        exec('props.' + propName + ' = ' '1')
            evalCond = [eval(s, {'__builtins__': None}, {'props': props}) for s in conditions]
            evalCond = np.multiply(evalCond, 1)

            result = np.array(self.State.State[currState].result)
            potentialNextStates = result[np.where(evalCond == 1)[0]]
        # print(time.time()-t2)
        # t3 = time.time()
        # First find all accepting states that can be reached given uncontrollable propositions
        reachableAcceptCycle = self.findAcceptingCycle()
        # print(time.time()-t3)
        # Find all routes from all next states
        # t4 = time.time()
        paths2Consider = []
        for i in range(np.size(potentialNextStates)):
            potState = potentialNextStates[i]
            for j in range(np.size(reachableAcceptCycle)):
                allPaths = self.State.nRoutes[potState][reachableAcceptCycle[j]]
                try:
                    paths2Consider.append(allPaths)
                except:
                    pass


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
        # paths2Consider = np.asarray([paths2Consider[0,:]])
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
        # print(time.time() - t4)

        return self
    def findAcceptingCycle(self):
        reachableAccepting = []
        for j in range(np.size(self.State.acceptingWithCycle)):
            transToAccept = self.State.acceptingWithCycle[j]
            comeFrom = np.where(self.graph[:,transToAccept] == 1)[0]
            allConds = np.zeros((1,np.size(self.State.State[0].condCNF[0][0])),dtype = int)
            for k in range(np.size(comeFrom)):
                stateFrom = comeFrom[k]
                idOfResult = np.where(self.State.State[stateFrom].result == transToAccept)[0][0]
                condOfI = np.asarray(self.State.State[stateFrom].condCNF[idOfResult])
                # condOfI2 = np.zeros((1, np.size(condOfI[0])),dtype = int)
                #
                # for ll in range(np.size(condOfI,0)):
                #     tempStr = condOfI[ll,:]
                #     condOfI2 = np.append(condOfI2, [tempStr], axis=0)
                # condOfI2 = condOfI2[1:,:]
                #
                # allConds = np.append(allConds,condOfI2, axis = 0)
                allConds = np.append(allConds,condOfI, axis = 0)


            allConds = allConds[:,self.locOfUncontrollable]
            allConds = np.unique(allConds, axis = 0)
            acceptable = np.where((allConds == self.input).all(axis=1))[0]
            if np.size(acceptable != 0):
                reachableAccepting.append(transToAccept)

        return reachableAccepting

    def computeRobustness(self,propositions,props,pos,t,wall):
        robustness = []
        for i in range(np.size(propositions)):
            propOfI = self.controllableProp[propositions[i]]
            ia = list(self.State.controllableProp).index(propOfI)
            phiIndex = self.State.controllablePropOrder[ia]
            phi = self.State.phi[phiIndex]

            #compute truth value of predicate
            isTrue = eval('props.'+propOfI)

            #if isTrue = 0 the predicate is false and we compute the estimated time to completion
            if isTrue == 1:
                #Compute how far from leving the safe set
                try:
                    distFromSafe = eval(phi.funcOf)
                except:
                    distFromSafe = eval(phi.funcOf[0][0])
                signF = -phi.signFS[0][0]

                totalDist = distFromSafe + signF*phi.p[0][0]
                robustness.append(self.weights[0] * totalDist)
            else:
                distFromSafe = self.getDistance(phi,pos, posRef)
                signF = phi.signFS[0][0]
                totalDist = distFromSafe + signF*phi.p[0][0]
                robot = phi.robotsInvolved[0]
                maxV = np.sqrt(self.maxV[3*robot-3]**2 +  self.maxV[3*robot-2]**2)
                time2Finish = totalDist/maxV
                timeRemaining = (phi.interval[1] + phi.inputTime) - t
                timeBuffer = timeRemaining - time2Finish
                robustness.append(self.weights[1] * timeBuffer)

        return np.asarray(robustness)

    def getDistance(self,phi,pos,posRef):
        startPos = pos[3*(phi.robotsInvolved[0]-1):3 * (phi.robotsInvolved[0] - 1) + 2]
        goalPoint = copy.deepcopy(startPos)
        if np.size(phi.point) == 0:
            numPos = np.size(re.findall('pos', phi.funcOf))
            if numPos == 1:
                indSame = int(re.search('(?<=\[)\d+(?=\])', phi.funcOf)[0])
                indSame = indSame % 3
                goalPoint[indSame] = phi.p
            else:
                dir = re.findall('(?=\().+?(?=\*)', phi.funcOf)

                dir[0] = re.findall('(?<=\().+(?<=\))', dir[0])[0]
                dir[1] = re.findall('(?=\().+(?<=\))', dir[1])[0]
                directionSplit = np.empty([1, np.size(dir)], dtype=object)
                directionSplit[0, 0] = re.split('[\+,\-]', dir[0])[1]
                directionSplit[0, 0] = '(' + directionSplit[0, 0]
                directionSplit[0, 1] = re.split('[\+,\-]', dir[1])[1]
                directionSplit[0, 1] = '(' + directionSplit[0, 1]

                vals = [-eval(elem, {'__builtins__': None}, {'pos': pos, 'np': np, 'posRef': posRef}) for elem in dir]
                goalPoint = pos[0:2] + vals
        else:
            goalPoint = phi.point

        map = self.State.map
        canReach = 0

        isect = self.intersectPoint(goalPoint[0], goalPoint[1], startPos[0], startPos[1],
                                        map[:, 0], map[:, 1], map[:, 2], map[:, 3])


        if not np.any(isect):
            canReach = 1
            cost = np.sqrt((goalPoint[0] - startPos[0]) ** 2 + (goalPoint[1] - startPos[1]) ** 2)

        if canReach == 0:
            # Find the closest nodes to the goal
            dist2p = []
            for i in range(np.size(self.State.nodes, 0)):
                dist2p.append(np.sqrt((goalPoint[0] - self.State.nodes[i, 0]) ** 2 +
                                      (goalPoint[1] - self.State.nodes[i, 1]) ** 2))
            idx = np.argsort(dist2p)

            # Connect goal to first node it can. starting with closest
            for i in range(np.size(idx)):
                isect = self.intersectPoint(goalPoint[0], goalPoint[1], self.State.nodes[idx[i], 0],
                                                    self.State.nodes[idx[i], 1], map[:, 0], map[:, 1], map[:, 2],map[:, 3])

                if not np.any(isect):
                    closestGoalInd = idx[i]
                    closestGoal = self.State.nodes[closestGoalInd]
                    break

            # Find the closest nodes to the start
            dist2p2 = []
            for i in range(np.size(self.State.nodes, 0)):
                dist2p2.append(
                    np.sqrt((startPos[0] - self.State.nodes[i, 0]) ** 2 + (startPos[1] - self.State.nodes[i, 1]) ** 2))

            idx = np.argsort(dist2p2)

            for i in range(np.size(idx)):
                isect = self.intersectPoint(startPos[0], startPos[1], self.State.nodes[idx[i], 0],
                                                    self.State.nodes[idx[i], 1], map[:, 0], map[:, 1], map[:, 2],
                                                    map[:, 3])
                if not np.any(isect):
                    #Check to see how far the segments are. A buffer may be needed
                    pt1 = startPos
                    pt2 = self.State.nodes[idx[i], 0:2]
                    if np.all(pt1 != pt2):
                        ptOfI1 = map[:, 0:2]
                        ptOfI2 = map[:, 2:4]
                        dist2closest1 = self.distWall(pt1, pt2, ptOfI1)
                        dist2closest2 = self.distWall(pt1, pt2, ptOfI2)
                        if min(dist2closest1) > .5 or min(dist2closest2) > .5:
                            closestStartInd = idx[i]
                            closestStart = self.State.nodes[closestStartInd]
                            break

            # If no closest start or goal can be found the point can not be reached given map and nodes
            if 'closestStart' in locals() or 'closestGoal' in locals():
                # compute total cost
                # Cost to get to start
                costToStart = np.sqrt((closestStart[0] - startPos[0]) ** 2 + (closestStart[1] - startPos[1]) ** 2)
                costToGoal = np.sqrt((goalPoint[0] - closestGoal[0]) ** 2 + (goalPoint[1] - closestGoal[1]) ** 2)
                pathCost = self.State.nodeConnections[closestStartInd][closestGoalInd][-1]
                cost = costToStart + costToGoal + pathCost
            else:
                print('No path to goal given map and nodes')
                cost = 1000000000

        return cost

    def pickTransition(self,allTransMax,pos,props,t,wall,posRef):
        numProp = np.size(np.where(allTransMax[0,:]==1))
        allRobust = []
        for i in range(np.size(allTransMax,0)):
            indOfActive = np.where(allTransMax[i,:]==1)[0]
            phiIndex = [self.State.controllablePropOrder[s] for s in indOfActive]
            phi = [self.State.phi[s] for s in phiIndex]
            robustness = []

            satisfiedPhi = [phi[s] for s in range(np.size(phi)) if
                            eval('props.' + phi[s].prop_label, {'__builtins__': None},
                                 {'props': props})]
            unsatisfiedPhi = [phi[s] for s in range(np.size(phi)) if
                              not eval('props.' + phi[s].prop_label, {'__builtins__': None},
                                    {'props': props})]


            for j in range(np.size(satisfiedPhi)):
                distFromSafe = eval(satisfiedPhi[j].funcOf)
                signF = -satisfiedPhi[j].signFS[0]
                totalDist = distFromSafe + signF * satisfiedPhi[j].p
                robustness.append(self.weights[0] * totalDist)

            for ii in range(1,self.M+1):
                phiRobot = []
                ids = []
                for j in range(np.size(unsatisfiedPhi)):
                    if np.where(unsatisfiedPhi[j].robotsInvolved==ii)[0].size != 0:
                        phiRobot.append(unsatisfiedPhi[j])
                        ids.append(unsatisfiedPhi[j].id)

                if len(phiRobot) == 1:
                    totalDist = self.getDistance(phiRobot[0], pos, posRef)
                    robot = phiRobot[0].robotsInvolved[0]
                    maxV = np.sqrt(self.maxV[3 * robot - 3] ** 2 + self.maxV[3 * robot - 2] ** 2)
                    time2Finish = totalDist / maxV
                    if self.preF == 1 and not isinstance(phiRobot[0].inputTime, int):
                        timeRemaining = phiRobot[0].interval[1]
                    else:
                        if isinstance(phiRobot[0].inputTime, int):
                            timeRemaining = (phiRobot[0].interval[1] + phiRobot[0].inputTime) - t
                        else:
                            timeRemaining = phiRobot[0].interval[1] - t
                    timeBuffer = timeRemaining - time2Finish
                    robustness.append(self.weights[1] * timeBuffer)
                elif len(phiRobot) > 1:
                    #Need to do prioritization
                    times = []
                    for j in range(np.size(phiRobot)):
                        if isinstance(phiRobot[0].inputTime, int):
                            timeLeft = phiRobot[j].interval[1] + phiRobot[j].inputTime
                        else:
                            timeLeft = phiRobot[j].interval[1] + t
                        times.append(timeLeft)

                    isTie = np.all(times == times[0])

                    #If everything is a tie we need to go to second priorization. Else order by time
                    if isTie:
                        #Location Priorization
                        dist = []
                        for j in range(np.size(phiRobot)):
                            distFromSafe = self.getDistance(phiRobot[j], pos, posRef)
                            signF = phiRobot[j].signFS[0]
                            totalDist = distFromSafe + signF * phiRobot[j].p
                            dist.append(totalDist)
                        orderToComplete = np.argpartition(dist,np.size(dist)-1)
                    else:
                        orderToComplete = np.argpartition(times,np.size(times)-1)

                    for j in range(np.size(orderToComplete)):
                        if j == 0:
                            distFromSafe = self.getDistance(phiRobot[orderToComplete[j]], pos, posRef)
                            signF = phiRobot[0].signFS[0]
                            totalDist = distFromSafe + signF * phiRobot[0].p
                            robot = phiRobot[0].robotsInvolved[0]
                            maxV = np.sqrt(self.maxV[3 * robot - 3] ** 2 + self.maxV[3 * robot - 2] ** 2)
                            time2Finish = totalDist / maxV
                            if self.preF == 1 and not isinstance(phiRobot[0].inputTime, int):
                                timeRemaining = phiRobot[0].interval[1]
                            else:
                                if isinstance(phiRobot[0].inputTime, int):
                                    timeRemaining = (phiRobot[0].interval[1] + phiRobot[0].inputTime) - t
                                else:
                                    timeRemaining = phiRobot[0].interval[1] -t

                            timeBuffer = timeRemaining - time2Finish
                            robustness.append(self.weights[1] * timeBuffer)
                        else:
                            posTemp = copy.deepcopy(pos)
                            if len(phiRobot[j - 1].point) != 0:
                                posTemp[phiRobot[j].nom[0, 0:2].astype('int')] = phiRobot[j - 1].point
                            else:
                                posTemp = np.array([posTemp[0],posTemp[1]])
                            distFromSafe2 = distFromSafe + self.getDistance(phiRobot[orderToComplete[j]], posTemp, posRef)
                            signF = phiRobot[j].signFS[0]
                            totalDist = distFromSafe2 + signF * phiRobot[j].p
                            robot = phiRobot[j].robotsInvolved[0]
                            maxV = np.sqrt(self.maxV[3 * robot - 3] ** 2 + self.maxV[3 * robot - 2] ** 2)
                            time2Finish = totalDist / maxV
                            if self.preF == 1 and not isinstance(phiRobot[j].inputTime, int):
                                timeRemaining = phiRobot[0].interval[1]
                            else:
                                timeRemaining = (phiRobot[j].interval[1] + phiRobot[j].inputTime) - t
                            timeBuffer = timeRemaining - time2Finish
                            robustness.append(self.weights[1] * timeBuffer)
            # robustness.sort()
            allRobust.append(robustness)

        # allRobust = allRobust[1:,:]


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
                try:
                    minInd.append(np.argmin(allRobustCopy[i]))
                except:
                    print('here')
            #create a vector of the minimum values
            minResponse = np.empty((1,np.size(minInd)))
            for i in range(np.size(minInd)):
                try:
                    minResponse[0,i] = allRobustCopy[i][minInd[i]]
                except:
                    print('here')
            if len(minResponse[0]) != 0:
                maxVal = np.amax(minResponse[0])
                maxInd = np.where(minResponse[0] == maxVal)[0]
                if np.size(maxInd) == 1:
                    minFound = 1
                    ind = [i for i in range(np.size(allRobustCopy, 0)) if maxVal in allRobustCopy[i]]
                    indOfTrans = maxInd[0]
                    # indOfTrans = ind[0]

                else:
                    allRobustCopy = [allRobustCopy[i] for i in maxInd]
                    allTransMaxCopy = allTransMaxCopy[maxInd, :]
                    indToDelete = [allRobustCopy[i].index(maxVal) for i in range(np.size(allRobustCopy, 0))]
                    [allRobustCopy[i].remove(maxVal) for i in range(np.size(allRobustCopy, 0))]
                    allRobustCopy = [i for i in allRobustCopy if i]
                    # allRobustCopy = np.delete(allRobustCopy,indToDelete,1)
            else:
                minFound = 1
                indOfTrans = 0

        trans2Make = allTransMaxCopy[indOfTrans, :]
        self.robustness = allRobustCopy[indOfTrans]

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
                        ia2.append(self.State.controllablePropOrder[locOfPred])

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

    def prepActivate(self,pos,posRef):
        props = self.props
        # Find the values of all parameters
        if self.State.wall is not None and len(self.State.wall) != 0:
            wall = self.State.wall
            valuesOfControl = eval(','.join(self.State.parameters), {'__builtins__': None},
                                    {'pos': pos, 'np': np, 'posRef': posRef, 'wall': wall})

        else:
            wall = []
            valuesOfControl = eval(','.join(self.State.parameters), {'__builtins__': None},
                                    {'pos': pos, 'np': np, 'posRef': posRef})
        # change from true/false to 1/0
        valuesOfControl = np.multiply(valuesOfControl,1)

        input = self.State.input[::2].astype(int)
        input = input[:np.size(self.uncontrollableProp)]

        # Assign all for the values to the propositions
        for i in range(np.size(self.controllableProp)):
            propName = self.controllableProp[i]
            exec('props.' + propName + ' = ' + str(valuesOfControl[i]))

        return props, wall, input