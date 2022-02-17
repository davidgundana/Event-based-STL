import numpy as np
import activateProp
import barrier
from customQP import quadprog
from itertools import combinations
import re
import time
import matrixDijkstra
import copy

class getAllCommands:
    def __init__(self,State,currState,pos,posStart,posRef,t,Ts,input,until):
        self.State = State.State
        self.Conflicts = State.Conflicts
        self.props = State.State.props
        self.accepting_states = State.State.accepting_states
        self.graph = State.State.graph
        self.controllableProp = State.State.controllableProp
        self.uncontrollableProp = State.State.uncontrollableProp
        self.propositions = State.State.propositions
        self.props2Activate = []
        self.currState = []
        self.maxV = State.maxV
        self.hz = int(State.freq)
        self.M = int(State.M)
        self.nom = []
        self.nodeGraph = State.State.nodeGraph
        self.nodes = State.State.nodes
        self.map = State.State.map
        self.State.input = input
        self.State.until = until
        self.bxt_eventually = []
        self.distTotal = np.zeros((1,self.M))[0]
        self.pos = pos.tolist()
        self.Commands(currState,pos,posStart,posRef,t,Ts)

    def trackInputs(self,pos,posStart,posRef,t):
        props = self.props
        input = self.State.input[::2].astype(int)
        for i in range(np.size(self.uncontrollableProp)):
            propName = self.uncontrollableProp[i]
            exec('props.' + propName + ' = ' + str(input[i]))

        count = 1
        for i in range(np.size(self.State.phi)):
            # Only interested if there is implication
            if self.State.phi[i].implies == 1:
                if int(np.size(self.State.input)) < np.size(self.uncontrollableProp) * 2 + 2 * count:
                    arrayToApp = np.array([0, 0])
                    self.State.input = np.hstack((self.State.input, arrayToApp))

                messTrue = eval(self.State.phi[i].impliesmessage)
                if messTrue and self.State.input[np.size(self.uncontrollableProp) * 2 + 2 * count - 2] == 0:
                    self.State.phi[i].inputTime = t
                    self.State.input[np.size(self.uncontrollableProp) * 2 + 2 * count - 2] = 1
                    self.State.input[np.size(self.uncontrollableProp) * 2 + 2 * count - 1] = t
                elif messTrue and self.State.input[np.size(self.uncontrollableProp) * 2 + 2 * count - 2] == 1:
                    if self.State.input[np.size(self.uncontrollableProp) * 2 + 2 * count - 1] + self.State.phi[i].interval[1] > t:
                        self.State.phi[i].inputTime = self.State.input[np.size(self.uncontrollableProp) * 2 + 2 * count - 1]
                    else:
                        self.State.phi[i].inputTime = t
                        self.State.input[np.size(self.uncontrollableProp) * 2 + 2 * count - 1] = t
                elif not messTrue:
                    self.State.input[np.size(self.uncontrollableProp) * 2 + 2 * count - 2] = 0

                count += 1
        # Need to reset specs

    def findConditions(self,activate):
        #this is for a pre-failure warning. We want to see what happens if things change with the inputs
        numUncon = np.size(activate.State.uncontrollableProp)
        self.conditions = np.empty((1,numUncon))
        # From your current state, evaluate all possible transitions and find all of the combinations of
        # inputs that are possible
        simpleInput = np.zeros((1, int(np.size(self.State.input) / 2)))
        for i in range(int(np.size(self.State.input) / 2)):
            simpleInput[0, i] = self.State.input[2 * i]
        simpleInput = simpleInput[0][:np.size(self.uncontrollableProp)]
        for i in range(activate.State.State[self.currState].condCNF.__len__()):
            condCNF = activate.State.State[self.currState].condCNF
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
                    self.conditions = np.vstack((self.conditions, specVal))

        # The first row is zero because of how we appended. Also, we only want the unique values to increase speed
        self.conditions = self.conditions.astype(int)
        self.conditions = self.conditions[1:,:]
        self.conditions = np.unique(self.conditions, axis = 0)
        # for i in range(np.size(simpleInput)):
        #     if simpleInput[i] == 1:
        #         indOfMatch = np.where(self.conditions[:,i] == 1)[0]
        #         self.conditions = np.delete(self.conditions,indOfMatch,axis=0)

    def potentialConflicts(self,act,pos,posRef,ub,t,props_, wall_, tempInp_):
        # Lets check things that can be activated for each robot
        # These are the indexes of the inputs for the robot
        badConditions = []
        robustRef = []
        for i in range(np.size(self.conditions, 0)):
            try:
                activate = activateProp.activateProp.activate(act, self.currState, props_, wall_, tempInp_, self.conditions[i], pos, posRef, t,1)
                if len(activate.robustness) > 0:
                    if any(x < 0 for x in activate.robustness):
                        badConditions.append(self.conditions[i])
                        robustRef.append(activate.robustness)
            except:
                pass
        # the badConditions variable shows which combinations of inputs result in a negative robustness score
        # for a proposition
        currInput = self.State.input[::2].astype(int)
        currInput = currInput[:np.size(self.uncontrollableProp)]
        locOfCurrent = np.where(currInput == 1)[0]
        for i in range(np.size(badConditions, 0)):
            locOfTrue = np.where(badConditions[i] == 1)[0]
            msg = 'If '
            if not np.array_equal(locOfCurrent, locOfTrue):
                for j in range(np.size(locOfTrue)):
                    if j == np.size(locOfTrue) - 1:
                        msg += 'and ' + self.uncontrollableProp[locOfTrue[j]] + ' '
                    else:
                        msg += self.uncontrollableProp[locOfTrue[j]] + ', '
                if np.size(locOfTrue) == 1:
                    msg += 'is sensed now, the specification may be violated'
                else:
                    msg += 'are sensed now, the specification may be violated'

            indOfNeg = np.where(np.asarray(robustRef[i]) < 0)[0]
            for j in range(np.size(indOfNeg)):
                phiId = activate.ids[indOfNeg[j]]
                timeNeeded = round(-1*robustRef[i][indOfNeg[j]]/activate.weights[1],2)
                msg += '. To make robustness positive the task ' + self.State.phi[phiId].params + ' needs ' + str(timeNeeded) + ' more seconds to be completed'
            print(msg)

    def Commands(self,currState,pos,posStart,posRef,t,Ts):
        # bounds for commands
        lb = -self.maxV
        ub = self.maxV
        wall = self.State.wall
        # Go through all specifications and, if there is implication, log the time that the input became activated.
        # also, if a specification has been satisfied, the input should be reset.
        self.trackInputs(pos, posStart, posRef,t)

        #Find Propositions to activate based on the current state and transitiosn to an accepting state
        act = activateProp.activateProp(self.State, currState, pos, posRef, t, self.maxV, self.M, 0, [])
        props_,wall_,tempInp_ = activateProp.activateProp.prepActivate(act,pos,posRef)
        activate = activateProp.activateProp.activate(act,currState,props_,wall_,tempInp_,[],pos,posRef,t,0)
        print(activate.props2Activate)
        self.props2Activate = activate.props2Activate
        self.currState = activate.currState

        # Toggle to turn on/off pre failure warnings
        preFailure = 0
        if preFailure:
            # this is for a pre-failure warning. We want to see what happens if things change with the inputs
            self.findConditions(activate)

            # Check for potential conflicts if additional inputs are sensed
            self.potentialConflicts(act,pos, posRef, ub, t,props_, wall_, tempInp_)
        # create the new STL specification with the activated propositions
        ia = [list(self.State.controllableProp).index(s) for s in list(self.props2Activate)]
        phiIndex = [self.State.controllablePropOrder[s] for s in ia]
        phi = [self.State.phi[s] for s in phiIndex]

        # loop through find the order of the stl objects for assignment later
        orderOfPhi = []
        for i in range(np.size(phi)):
            orderOfPhi.append(phi[i].id)

        nom = np.zeros((1,3 * self.M))

        for i in range(1,self.M+1):
            # Find all fo the specifications for each robot
            phiRobot = []
            ids = []
            for j in range(np.size(phi)):
                if np.where(phi[j].robotsInvolved==i)[0].size != 0:
                    phiRobot.append(phi[j])
                    ids.append(phi[j].id)

            if phiRobot != []:
                # find the candidate barrier function at the position and find partial derivatives
                bxtx, phiRobotPlace, propsActivated, bxt_eventually = barrier.barrier(self, pos, posStart, posRef, t,
                                                                                      Ts, phiRobot, 100, self.hz)

                phiRobot = []
                ids = []
                for j in range(np.size(phiRobotPlace)):
                    try:
                        if phiRobotPlace[j].bxt_i != 0:
                            phiRobot.append(phiRobotPlace[j])
                            ids.append(phiRobotPlace[j].id)
                    except:
                        pass

                self.bxt_eventually.append(bxt_eventually)

                [bPartialX, bPartialT] = barrier.partials(self, pos, posStart, posRef, t, Ts, phiRobot, 100, self.hz,
                                                          bxtx)

                A = -1 * np.dot(np.array(bPartialX), np.identity(3 * self.M))
                if abs(bPartialT[0]) > 50:
                    bPartialT[0] = 0

                alpha = 1
                b = alpha * (bxtx) + bPartialT[0]

                # check for changes.these are the robots affected by barrier functions
                if np.any(bPartialX):
                    nominals = np.empty((1, 3), dtype=float)
                    for j in range(np.size(phiRobot)):
                        posRob = pos[phiRobot[j].nom[0, :].astype('int')]
                        nomRob = phiRobot[j].nom[1, :]
                        if eval(phiRobot[j].params) and phiRobot[j].type == 'ev':
                            nomR = np.zeros((1,3),dtype=float)[0]
                        else:
                            try:
                                nomR = self.getNom(self.State, posRob, nomRob, phiRobot[j].p)
                            except:
                                nomR = np.zeros((1, 3), dtype=float)[0]
                        phiRobot[j].nom = phiRobot[j].nom.astype('float')
                        phiRobot[j].nom[1, :] = nomR
                        if np.sum(abs(phiRobot[j].nom[1, :])) != 0:
                            velBound = ub[phiRobot[j].nom[0, 0:].astype(int)]
                            thisNom = phiRobot[j].nom
                            normalize = np.sqrt(thisNom[1, 0] ** 2 + thisNom[1, 1] ** 2)
                            thisNomBounded = np.zeros((1, 3))[0]
                            if normalize != 0:
                                thisNomBounded[0:2] = np.multiply((thisNom[1, 0:2] / normalize), velBound[0:2])
                            else:
                                thisNomBounded = thisNom[1, :]

                            nominals = np.vstack((nominals, thisNom[0, :], thisNomBounded))
                            # break

                    nominals = nominals[1:, :]

                    # This is to check if there are multiple nominal controllers (2+ activated cbfs for same robot)
                    if np.size(nominals, 0) > 2:
                        for j in range(int(np.size(nominals, 0) / 2)):
                            if j > 0:
                                nominals = np.delete(nominals, 2, 0)
                        finalT = []
                        for j in range(np.size(phiRobot)):
                            if phiRobot[j].implies == 0:
                                finalT.append(phiRobot[j].interval[1])
                            else:
                                finalT.append(phiRobot[j].interval[1] + phiRobot[j].inputTime)
                        locOfSoonest = np.argmin(finalT)
                        try:
                            nominals = np.vstack((nominals[0,:],nominals[locOfSoonest+1,:]))
                        except:
                            pass
                        #print('multiple nominal controllers. Attempting to satisfy specification by satisfying in order of time bound')

                    Anew = A[3 * i - 3:3 * i]
                    lbI = lb[3 * i - 3:3 * i]
                    ubI = ub[3 * i - 3:3 * i]

                    if np.any(nominals):
                        x0 = nominals[1,:]
                    else:
                        x0 = np.array([0,0,0])
                        nominals = np.zeros((2, 3))

                    H = np.array([[2,0,0],[0,2,0],[0,0,2]])
                    f = np.array([-2*nominals[1,0],-2*nominals[1,1],-2*nominals[1,2]]).T

                    qp = quadprog(H,f,Anew, b, x0, lbI, ubI)
                    nomInd = qp.result.x
                    if not qp.result.success:
                        print('Specification violated')
                        nom[0][3 * i - 3] = 0
                        nom[0][3 * i - 2] = 0
                        nom[0][3 * i - 1] = 0
                    else:
                        nom[0][3 * i - 3] = nomInd[0]
                        nom[0][3 * i - 2] = nomInd[1]
                        nom[0][3 * i - 1] = nomInd[2]
                    # if nom[0][0] == 0 and t > 5:
                    #     print('here')
        self.nom = nom


    def getNom(self,State, pos, nom, p):
        startPos = pos
        goalPoint = pos + nom
        closeEnough = .2

        map = self.State.map
        canReach = 0

        isect = self.intersectPoint(goalPoint[0], goalPoint[1], startPos[0], startPos[1],
                                        map[:, 0], map[:, 1], map[:, 2], map[:, 3])

        if not np.any(isect):
            pt1 = startPos
            pt2 = goalPoint
            ptOfI1 = map[:, 0:2]
            ptOfI2 = map[:, 2:4]
            dist2closest1 = self.distWall(pt1, pt2, ptOfI1)
            dist2closest2 = self.distWall(pt1, pt2, ptOfI2)
            if min(dist2closest1) > .1 and min(dist2closest2) > .1:
                canReach = 1

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

            closestStartInd = []
            closestStartDist = []
            for i in range(np.size(idx)):
                isect = self.intersectPoint(startPos[0], startPos[1], self.State.nodes[idx[i], 0],
                                                    self.State.nodes[idx[i], 1], map[:, 0], map[:, 1], map[:, 2],
                                                    map[:, 3])
                if not np.any(isect):
                    #Check to see how far the segments are. A buffer may be needed
                    pt1 = startPos
                    pt2 = self.State.nodes[idx[i], 0:2]
                    ptOfI1 = map[:, 0:2]
                    ptOfI2 = map[:, 2:4]
                    dist2closest1 = self.distWall(pt1, pt2, ptOfI1)
                    dist2closest2 = self.distWall(pt1, pt2, ptOfI2)
                    if min(dist2closest1) > .03 and min(dist2closest2) > .03:
                        closestStartInd.append(idx[i])
                        closestStartDist.append(np.sqrt((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2))

            # Find Route
            nodesToGo = [self.State.nodeConnections[i][closestGoalInd][-1] for i in closestStartInd]
            distToGoals = np.asarray(nodesToGo) + np.asarray(closestStartDist)
            if np.size(distToGoals) != 0:
                indOfNext = np.argmin(distToGoals)
                #Check to see if this node wrongly itnersects
                wayPoint = self.State.nodes[closestStartInd[indOfNext],:]
                nom = wayPoint - pos[0:2]
                nom = np.hstack((nom,0))

        return nom  #, dist2NextPoint, distx, disty,distTotal, lastPoint

    def intersectPoint(self, x1, y1, x2, y2, x3, y3, x4, y4):
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)

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
            dist = np.zeros((1,np.size(pt,0)))[0]
            # dist = np.sqrt(dx ** 2 + dy ** 2) * np.ones((1,np.size(pt,0)))[0]
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
