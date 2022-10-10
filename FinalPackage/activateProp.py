import numpy as np
import matrixDijkstra
import re
import networkx as nx
from itertools import islice
from itertools import permutations
import copy
import time
from getNom import getNom
import matrixDijkstra


# class activateProp:
#     def __init__(self, State, preF, conditions,input_,getNom):
#         self.State = State.State #State of buchi
#         self.M = State.M #Number of robots
#         self.props = State.props # Proposition values
#         self.accepting_states = State.accepting_states # set of accepting states
#         self.graph = State.graph # transitions from node to node
#         self.controllableProp = State.controllableProp # set of controllable propositions
#         self.uncontrollableProp = State.uncontrollableProp # Set of uncontrollable propositions
#         self.propositions = State.propositions # propositions to evaluate
#         self.props2Activate = [] # preallocate activated propositions
#         self.currState = [] # Pre-allocate current state
#         self.locOfUncontrollable = [list(self.propositions).index(s) for s in list(self.uncontrollableProp)]
#         self.input = []
#         self.maxV = State.maxV
#         self.weights = [5,.75] #What is already true, what is false right now
#         self.preF = preF
#         self.conditions = conditions
#         self.robustRef = []
#         self.distFromSafeRef = []
#         self.input = input_
#         self.getNom = getNom
#         np.seterr(divide='ignore')
#
#     def activate(self,currState,conditions,x,xR,t,preF):
#         props = self.props
#         wall = self.State.wall
#         self.conditions = conditions
#         self.preF = preF
#
#         if self.preF == 1:
#             for i in range(np.size(self.uncontrollableProp)):
#                 propName = self.uncontrollableProp[i]
#                 exec('props.' + propName + ' = ' + str(self.conditions[i]))
#
#         #find the transition that is being made based on the conditions
#         conditions = self.State.State[currState].cond
#         evalCond = eval(','.join(conditions), {'__builtins__': None}, {'props': props})
#         evalCond = np.multiply(evalCond,1)
#
#         result = np.array(self.State.State[currState].result)
#         potentialNextStates = result[np.where(evalCond == 1)[0]]
#
#         # First find all accepting states that can be reached given uncontrollable propositions
#         reachableAcceptCycle = self.findAcceptingCycle()
#
#         paths2Consider = []
#         for i in range(np.size(potentialNextStates)):
#             potState = potentialNextStates[i]
#             for j in range(np.size(reachableAcceptCycle)):
#                 allPaths = self.State.nRoutes[potState][reachableAcceptCycle[j]]
#                 paths2Consider.append(allPaths)
#
#         paths2Consider = [item for sublist in paths2Consider for item in sublist]
#         # We only care about the first transition for now
#         row2Del = []
#         for s in range(np.size(paths2Consider, 0)):
#             if np.size(paths2Consider[s]) > 2:
#                 row2Del.append(s)
#             if np.size(paths2Consider[s]) < 2:
#                 paths2Consider[s] = [paths2Consider[s][0], paths2Consider[s][0]]
#             else:
#                 paths2Consider[s] = paths2Consider[s][0:2]
#         paths2Consider = np.asarray(paths2Consider)
#         paths2Consider = np.delete(paths2Consider,row2Del,0)
#         paths2Consider = np.unique(paths2Consider, axis=0)
#
#         # Compute robustness for each path
#         # first find all activated propositions for all transitions
#         allTransitionsWithState = np.empty((1, len(self.controllableProp) + 1), dtype=int)
#         allTransitionsWithNextState = np.empty((1, len(self.controllableProp) + 1), dtype=int)
#
#         propRef = []
#         for i in range(np.size(paths2Consider, 0)):
#             results = np.array(self.State.State[paths2Consider[i][0]].result)
#             try:
#                 indCondToTran = np.where(results == paths2Consider[i][1])[0]
#             except:
#                 indCondToTran = np.where(results == paths2Consider[i][0])[0]
#
#             transitionOptions = self.State.State[paths2Consider[i][0]].condCNF[indCondToTran[0]]
#             possibleTransitions = np.asarray(transitionOptions)[:, self.locOfUncontrollable]
#             acceptable = np.where((possibleTransitions == self.input).all(axis = 1))[0]
#             try:
#                 transitionOptions = np.unique(np.asarray(transitionOptions)[acceptable, len(self.uncontrollableProp):],
#                                           axis=0)
#             except:
#                 pass
#
#             if np.size(acceptable != 0):
#                 if np.size(transitionOptions,0) != 0:
#                     stateRef = paths2Consider[i][0] * np.ones((np.size(transitionOptions, 0), 1), dtype=int)
#                     nextRef = paths2Consider[i][1] * np.ones((np.size(transitionOptions, 0), 1), dtype=int)
#                     transitionPot = np.hstack((transitionOptions,nextRef))
#                     transitionOptions = np.hstack((transitionOptions, stateRef))
#                     allTransitionsWithState = np.vstack((allTransitionsWithState, transitionOptions))
#                     allTransitionsWithNextState = np.vstack((allTransitionsWithNextState,transitionPot))
#                     for j in range(np.size(transitionOptions, 0)):
#                         propRef.extend([i for i, x in enumerate(transitionOptions[j, :-1]) if x == 1])
#
#         allTransitions = np.unique(allTransitionsWithState[1:, :-1], axis=0)
#         allTransitionsWithState = allTransitionsWithState[1:, :]
#         # check if any propositions have until tag
#         allTransitions = self.checkUntil(allTransitions)
#
#         # first we choose the transition with the most infinities. this is the highest robustness
#         numMax = np.count_nonzero(np.isinf(allTransitions), axis=1)
#         try:
#             maxLoc = np.argwhere(numMax == np.amax(numMax)).ravel()
#         except:
#             pass
#         if np.size(maxLoc) == 1:
#             # only one maximum robustness
#             trans2Make = allTransitions[maxLoc[0]]
#             if np.all(trans2Make == np.inf):
#                 self.robustness = []
#             else:
#                 self.pickTransition(np.array([trans2Make]), x, props, t, wall, xR)
#         else:
#             # Need to break the tie by computing robustness for those that are tied
#             allTransMax = allTransitions[maxLoc, :]
#             trans2Make = self.pickTransition(allTransMax, x, props, t, wall, xR)
#
#         stateLoc = np.where((allTransitions == trans2Make).all(axis=1))[0][0]
#
#         self.currState = int(allTransitionsWithState[stateLoc, -1])
#
#         propsLoc = (np.where(trans2Make == 1)[0]).tolist()
#         self.props2Activate = [self.controllableProp[element] for element in propsLoc]
#
#         return self
#
#     def findAcceptingCycle(self):
#         reachableAccepting = []
#         for j in range(np.size(self.State.acceptingWithCycle)):
#             transToAccept = self.State.acceptingWithCycle[j]
#             comeFrom = np.where(self.graph[:,transToAccept] == 1)[0]
#             for k in range(np.size(comeFrom)):
#                 stateFrom = comeFrom[k]
#                 idOfResult = np.where(self.State.State[stateFrom].result == transToAccept)[0][0]
#                 condOfI = np.asarray(self.State.State[stateFrom].condCNF[idOfResult])
#
#                 # New stuff
#                 condOfI2 = condOfI[:,self.locOfUncontrollable]
#                 condOfI2 = np.unique(condOfI2, axis=0)
#                 acceptable = np.where((condOfI2 == self.input).all(axis=1))[0]
#                 if np.size(acceptable != 0):
#                     reachableAccepting.append(transToAccept)
#                     break
#
#         return reachableAccepting
#
#     def pickTransition(self,allTransMax,x,props,t,wall,xR):
#         allRobust = []
#         allRobustId = []
#         if np.size(self.robustRef) == 0:
#             robustRef = [None] * np.size(self.State.phi)
#             distFromSafeRef = [None] * np.size(self.State.phi)
#         else:
#             robustRef = self.robustRef
#             distFromSafeRef = self.distFromSafeRef
#
#         for i in range(np.size(allTransMax,0)):
#             indOfActive = np.where(allTransMax[i,:]==1)[0]
#             phiIndex = [self.State.controllablePropOrder[s] for s in indOfActive]
#             phi = [self.State.phi[s] for s in phiIndex]
#             robustness = []
#             ids = []
#
#             satisfiedPhi = [phi[s] for s in range(np.size(phi)) if phi[s].currentTruth]
#             unsatisfiedPhi = [phi[s] for s in range(np.size(phi)) if not phi[s].currentTruth]
#             for j in range(np.size(satisfiedPhi)):
#                 if robustRef[satisfiedPhi[j].id] is None:
#                     robtemp = self.weights[0] * satisfiedPhi[j].distFromSafe
#                     robustRef[satisfiedPhi[j].id] = robtemp
#                     robustness.append(robtemp)
#                     ids.append(satisfiedPhi[j].id)
#                 else:
#                     robustness.append(robustRef[satisfiedPhi[j].id])
#                     ids.append(satisfiedPhi[j].id)
#             for ii in range(1,self.M+1):
#                 phiRobot = []
#                 for j in range(np.size(unsatisfiedPhi)):
#                     if ii in unsatisfiedPhi[j].robotsInvolved:
#                         phiRobot.append(unsatisfiedPhi[j])
#                         ids.append(unsatisfiedPhi[j].id)
#                 if len(phiRobot) == 1:
#                     if robustRef[phiRobot[0].id] is None:
#                         totalDist = phiRobot[0].distFromSafe
#                         time2Finish = phiRobot[0].time2Finish
#                         if self.preF == 1 and not isinstance(phiRobot[0].inputTime, int):
#                             timeRemaining = phiRobot[0].interval[1]
#                         else:
#                             if isinstance(phiRobot[0].inputTime, float):
#                                 timeRemaining = (phiRobot[0].interval[1] + phiRobot[0].inputTime) - t
#                             else:
#                                 timeRemaining = phiRobot[0].interval[1] - t
#                         timeBuffer = timeRemaining - time2Finish
#                         robtemp = self.weights[1] * timeBuffer
#                         robustRef[phiRobot[0].id] = robtemp
#                         distFromSafeRef[phiRobot[0].id] = totalDist
#                         robustness.append(robtemp)
#                         ids.append(phiRobot[0].id)
#                     else:
#                         robustness.append(robustRef[phiRobot[0].id])
#                         ids.append(phiRobot[0].id)
#                 elif len(phiRobot) > 1:
#                     #Need to do prioritization
#                     possOrders = list(permutations(range(np.size(phiRobot))))
#                     robustnessTEMP = []
#                     idsTEMP = []
#                     for j in range(np.size(possOrders,0)):
#                         time2FinishPrev = 0
#                         robustRow = []
#                         idsRow = []
#                         for k in range(np.size(possOrders[j],0)):
#                             if k == 0:
#                                 if robustRef[phiRobot[possOrders[j][k]].id] is None:
#                                     totalDist = phiRobot[possOrders[j][k]].distFromSafe
#                                     time2Finish = phiRobot[possOrders[j][k]].time2Finish
#
#                                     if self.preF == 1 and t > phiRobot[possOrders[j][k]].interval[1]+phiRobot[possOrders[j][k]].inputTime:
#                                         # time passed. full time bound remains
#                                         timeRemaining = phiRobot[possOrders[j][k]].interval[1]
#                                     elif self.preF == 1:
#                                         timeRemaining = (phiRobot[possOrders[j][k]].interval[1] + phiRobot[possOrders[j][k]].inputTime) - t
#                                     else:
#                                         if isinstance(phiRobot[possOrders[j][k]].inputTime, float):
#                                                 timeRemaining = (phiRobot[possOrders[j][k]].interval[1] + phiRobot[possOrders[j][k]].inputTime) - t
#                                         else:
#                                             timeRemaining = phiRobot[possOrders[j][k]].interval[1] - t
#
#                                     timeBuffer = timeRemaining - time2Finish
#                                     time2FinishPrev += time2Finish
#                                     robtemp = self.weights[1] * timeBuffer
#                                     robustRef[phiRobot[possOrders[j][k]].id] = robtemp
#                                     distFromSafeRef[phiRobot[possOrders[j][k]].id] = totalDist
#                                     robustRow.append(robtemp)
#                                     idsRow.append(phiRobot[possOrders[j][k]].id)
#                                 else:
#                                     robustRow.append(robustRef[phiRobot[possOrders[j][k]].id])
#                                     idsRow.append(phiRobot[possOrders[j][k]].id)
#                             else:
#                                 xTemp = copy.deepcopy(x)
#                                 xTemp[phiRobot[possOrders[j][k]].nom[0, 0:2].astype('int')] = phiRobot[
#                                     possOrders[j][k-1]].point
#                                 nom, costTemp = self.getNom(phiRobot[possOrders[j][k]], xTemp, xR)
#                                 distFromSafe2 = phiRobot[possOrders[j][k-1]].distFromSafe + costTemp
#                                 signF = phiRobot[possOrders[j][k]].signFS[0]
#                                 totalDist = distFromSafe2 + signF * phiRobot[possOrders[j][k]].p
#                                 robot = phiRobot[possOrders[j][k]].robotsInvolved[0]
#                                 maxV = self.maxV[3 * robot - 3]
#                                 time2Finish = totalDist / maxV
#                                 if self.preF == 1 and not isinstance(phiRobot[possOrders[j][k]].inputTime, float):
#                                     timeRemaining = phiRobot[possOrders[j][k]].interval[1]
#                                 else:
#                                     timeRemaining = (phiRobot[possOrders[j][k]].interval[1] + phiRobot[possOrders[j][k]].inputTime) - t
#                                 timeBuffer = timeRemaining - (time2Finish + time2FinishPrev)
#                                 time2FinishPrev += time2Finish
#                                 idsRow.append(phiRobot[possOrders[j][k]].id)
#                                 robustRow.append(self.weights[1] * timeBuffer)
#                         robustnessTEMP.append(robustRow)
#                         idsTEMP.append(idsRow)
#                     minFound = 0
#                     robustCopy = copy.deepcopy(robustnessTEMP)
#                     while minFound == 0:
#                         minInd = []
#                         for j in range(np.size(robustCopy, 0)):
#                             minInd.append(np.argmin(robustCopy[j]))
#                         # create a vector of the minimum values
#                         minResponse = np.empty((1, np.size(minInd)))
#                         for j in range(np.size(minInd)):
#                             minResponse[0, j] = robustCopy[j][minInd[j]]
#                         if len(minResponse[0]) != 0:
#                             maxVal = np.amax(minResponse[0])
#                             maxInd = np.where(minResponse[0] == maxVal)[0]
#                             if np.size(maxInd) == 1:
#                                 minFound = 1
#                                 indOfTrans = maxInd[0]
#                             else:
#                                 robustCopy = [robustCopy[j] for j in maxInd]
#                                 robustCopy = robustCopy[maxInd, :]
#                                 [robustCopy[j].remove(maxVal) for j in range(np.size(robustCopy, 0))]
#                                 robustCopy = [j for j in robustCopy if j]
#                         else:
#                             minFound = 1
#                             indOfTrans = 0
#                     robustness.extend(robustnessTEMP[indOfTrans])
#                     ids.extend(idsTEMP[indOfTrans])
#
#             allRobust.append(robustness)
#             allRobustId.append(ids)
#
#         #Choose transition that has the maximum minimum
#         minFound = 0
#         allRobustCopy = copy.deepcopy(allRobust)
#         allTransMaxCopy = copy.deepcopy(allTransMax)
#         if np.size(allTransMax, 0) == 1:
#             minFound = 1
#             indOfTrans = 0
#         while minFound ==0:
#             minInd = []
#             for i in range(np.size(allRobustCopy, 0)):
#                 minInd.append(np.argmin(allRobustCopy[i]))
#             #create a vector of the minimum values
#             minResponse = np.empty((1,np.size(minInd)))
#             for i in range(np.size(minInd)):
#                 minResponse[0,i] = allRobustCopy[i][minInd[i]]
#             if len(minResponse[0]) != 0:
#                 maxVal = np.amax(minResponse[0])
#                 maxInd = np.where(minResponse[0] == maxVal)[0]
#                 if np.size(maxInd) == 1:
#                     minFound = 1
#                     indOfTrans = maxInd[0]
#                 else:
#                     allRobustCopy = [allRobustCopy[i] for i in maxInd]
#                     allTransMaxCopy = allTransMaxCopy[maxInd, :]
#                     [allRobustCopy[i].remove(maxVal) for i in range(np.size(allRobustCopy, 0))]
#                     allRobustCopy = [i for i in allRobustCopy if i]
#             else:
#                 minFound = 1
#                 indOfTrans = 0
#
#         trans2Make = allTransMaxCopy[indOfTrans, :]
#         self.robustness = allRobustCopy[indOfTrans]
#         self.robustness = allRobust[np.where(np.all(allTransMax == trans2Make,axis=1))[0][0]]
#         # if any(x < 0 for x in self.robustness) and t > 15:
#         #     print('here')
#         self.ids = allRobustId[np.where(np.all(allTransMax == trans2Make,axis=1))[0][0]]
#         self.robustRef = robustRef
#         self.distFromSafeRef = distFromSafeRef
#         return trans2Make
#
#     def checkUntil(self,allTransitions):
#         allUnt = []
#         for i in range(np.size(self.State.phi)):
#             if self.State.phi[i].phiUntil != "":
#                 propOfI = self.State.phi[i].prop_label
#                 propOfI2 = self.State.phi[i].phiUntil
#                 ia = self.State.controllablePropOrder[list(self.State.controllableProp).index(propOfI)]
#                 ia2 = []
#                 splitPhiUntil = re.split('(\||\&)', propOfI2)
#                 for j in range(np.size(splitPhiUntil)):
#                     if '&' not in splitPhiUntil[j] and '|' not in splitPhiUntil[j]:
#                         predOfI = re.search('(?<=props\.).+?(?=(\)|\s))', splitPhiUntil[j])[0]
#                         locOfPred = list(self.State.controllableProp).index(predOfI)
#                         #ia2.append(self.State.controllablePropOrder[locOfPred])
#                         ia2.append(locOfPred)
#
#                 for j in range(np.size(allTransitions, 0)):
#                     actEv = 0
#                     for jj in range(np.size(ia2)):
#                         if allTransitions[j, ia2[jj]] == 1:
#                             actEv = 1
#                             break
#                     if actEv == 1 and allTransitions[j, ia] == np.inf:
#                         allTransitions[j, ia] = 1
#
#         return allTransitions

# class activatePropositions:
#     def __init__(self, specattr, potS, preF):
#         self.State = specattr #State of buchis
#         self.potS = potS
#         self.M = State.M #Number of robots
#         self.props = State.props # Proposition values
#         self.accepting_states = State.accepting_states # set of accepting states
#         self.graph = State.graph # transitions from node to node
#         self.controllableProp = State.controllableProp # set of controllable propositions
#         self.uncontrollableProp = State.uncontrollableProp # Set of uncontrollable propositions
#         self.propositions = State.propositions # propositions to evaluate
#         self.props2Activate = [] # preallocate activated propositions
#         self.currState = [] # Pre-allocate current state
#         self.locOfUncontrollable = [list(self.propositions).index(s) for s in list(self.uncontrollableProp)]
#         self.input = []
#         self.maxV = State.maxV
#         self.weights = [5,.75] #What is already true, what is false right now
#         self.preF = preF
#         self.conditions = conditions
#         self.robustRef = []
#         self.distFromSafeRef = []
#         self.input = input_
#         self.getNom = getNom
#         np.seterr(divide='ignore')

def activate(specattr,potS,roadmap,preF,conditions,x,xR,t,maxV,sizeU):
    allProps2Activate = []
    allRobustness = []
    for b in range(np.size(specattr,0)):
        props = specattr[b].props
        wall = specattr[b].wall

        if preF == 1:
            for i in range(np.size(specattr[b].uncontrollableProp)):
                propName = specattr[b].uncontrollableProp[i]
                exec('props.' + propName + ' = ' + str(conditions[i]))

        #find the transition that is being made based on the conditions
        conditions = specattr[b].buchiStates[potS[b]].cond
        try:
            evalCond = eval(','.join(conditions), {'__builtins__': None}, {'props': props})
        except:
            print('here')
        evalCond = np.multiply(evalCond,1)

        result = np.array(specattr[b].buchiStates[potS[b]].result)
        potentialNextStates = result[np.where(evalCond == 1)[0]]
        # First find all accepting states that can be reached given uncontrollable propositions
        reachableAcceptCycle = findAcceptingCycle(specattr[b])
        if np.size(potentialNextStates) == 0:
            print('no state')

        firstTry = 1
        while firstTry < 2:
            paths2Consider = []
            allPaths = []
            for i in range(np.size(potentialNextStates)):
                potState = potentialNextStates[i]
                for j in range(np.size(reachableAcceptCycle)):
                    if specattr[b].nRoutes != []:
                        try:
                            allPaths = specattr[b].nRoutes[potState][reachableAcceptCycle[j]]
                        except:
                            # print(props.pred0,props.pred1,props.pred2,props.pred3,props.pred4,props.pred5,props.pred6,props.pred7,props.pred8)
                            allPaths = specattr[b].nRoutes[potState][j]
                    else:
                        if firstTry:
                            (cost, rute) = matrixDijkstra.dijkstra(specattr[b].graph, potState, reachableAcceptCycle[j])
                            allPaths = np.flip(rute).tolist()
                        else:
                            graph = specattr[b].graph
                            G = nx.DiGraph()
                            G.add_nodes_from(range(0, np.size(graph, 0)))
                            for jj in range(np.size(graph, 0)):
                                for ii in range(np.size(graph, 0)):
                                    if graph[ii, jj] == 1:
                                        G.add_edge(ii, jj)
                            if potState == reachableAcceptCycle[j]:
                                statesFrom = np.where(specattr[b].graph[:, potState] == 1)[0]
                                for jj in range(np.size(statesFrom)):
                                    for path in k_shortest_paths(G, potState, statesFrom[jj], 5):
                                        allPaths.append(path + [potState])
                            else:
                                for path in k_shortest_paths(G, potState, reachableAcceptCycle[j], 5):
                                    allPaths.append(path)
                            print('method2')
                    paths2Consider.append(allPaths)

            # if len(paths2Consider) > 1:
            paths2Consider = [item for sublist in paths2Consider for item in sublist]
            if np.size(paths2Consider) == 0:
                print('no paths!')
            # We only care about the first transition for now
            for s in range(np.size(paths2Consider, 0)):
            # for s in range(len(paths2Consider)):
                try:
                    if np.size(paths2Consider[s]) < 2:
                        paths2Consider[s] = [paths2Consider[s][0], paths2Consider[s][0]]
                    else:
                        paths2Consider[s] = paths2Consider[s][0:2]
                except:
                    pass
            paths2Consider = np.asarray(paths2Consider)
            # paths2Consider = np.delete(paths2Consider,row2Del,0)
            paths2Consider = np.atleast_2d(paths2Consider)
            paths2Consider = np.unique(paths2Consider, axis=0)
            row2Del = []
            try:
                for i in range(np.size(paths2Consider,0)):
                    if not(paths2Consider[i,1] in reachableAcceptCycle):
                        row2Del.append(i)
                if len(row2Del) != np.size(paths2Consider,0):
                    paths2Consider = np.delete(paths2Consider,row2Del,0)
            except:
                pass
            # Compute robustness for each path
            # first find all activated propositions for all transitions
            allTransitionsWithState = np.empty((1, len(specattr[b].controllableProp) + 1), dtype=int)
            allTransitionsWithNextState = np.empty((1, len(specattr[b].controllableProp) + 1), dtype=int)

            propRef = []
            for i in range(np.size(paths2Consider, 0)):
                try:
                    results = np.array(specattr[b].buchiStates[int(paths2Consider[i][0])].result)
                except:
                    print('here')
                try:
                    indCondToTran = np.where(results == int(paths2Consider[i][1]))[0]
                except:
                    indCondToTran = np.where(results == int(paths2Consider[i][0]))[0]

                try:
                    transitionOptions = specattr[b].buchiStates[int(paths2Consider[i][0])].condCNF[indCondToTran[0]]
                except:
                    continue
                possibleTransitions = np.asarray(transitionOptions)[:, specattr[b].locOfUncontrollable]
                acceptable = np.where((possibleTransitions == specattr[b].inputVal).all(axis = 1))[0]
                try:
                    transitionOptions = np.asarray(transitionOptions)[acceptable, :]
                    transitionOptions = np.unique(transitionOptions[:,specattr[b].locOfControllable],axis = 0)
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

            allTransitions, idx = np.unique(allTransitionsWithState[1:, :-1], axis=0,return_index=True)
            allTransitions = allTransitionsWithState[1:, :-1][np.sort(idx)]
            allTransitionsWithState = allTransitionsWithState[1:, :]
            # check if any propositions have until tag
            allTransitions = checkUntil(specattr[b], allTransitions)

            # first we choose the transition with the most infinities. this is the highest robustness
            numMax = np.count_nonzero(np.isinf(allTransitions), axis=1)
            try:
                maxLoc = np.argwhere(numMax == np.amax(numMax)).ravel()
                firstTry = 2
            except:
                if firstTry:
                    firstTry = 0
                else:
                    firstTry = 2
                    print('here')

        ids = []
        if np.size(maxLoc) == 1:
            # only one maximum robustness
            trans2Make = allTransitions[maxLoc[0]]
            if np.all(trans2Make == np.inf):
                robustness = []
            else:
                trans2Make,robustness, ids = pickTransition(specattr[b], np.array([trans2Make]), x, props, t, wall, xR, preF,roadmap,maxV,sizeU)
        else:
            # Need to break the tie by computing robustness for those that are tied
            allTransMax = allTransitions[maxLoc, :]
            trans2Make,robustness, ids = pickTransition(specattr[b], allTransMax, x, props, t, wall, xR, preF,roadmap,maxV,sizeU)

        stateLoc = np.where((allTransitions == trans2Make).all(axis=1))[0][0]

        if int(allTransitionsWithState[stateLoc, -1]) == 57:
            print('here')
        potS[b] = int(allTransitionsWithState[stateLoc, -1])

        propsLoc = (np.where(trans2Make == 1)[0]).tolist()
        props2Activate = [specattr[b].controllableProp[element] for element in propsLoc]

        allProps2Activate.append(props2Activate)
        allRobustness.append(robustness)
    #determine which to buchi to follow
    if len(specattr) > 1 and any(allRobustness):
        specattr, props2Activate, potS, robustness = pickSpec(specattr, allRobustness, potS, allProps2Activate)

    return specattr,props2Activate,potS,robustness, ids

def pickSpec(specattr,allRobustness, potS, allProps2Activate):
    option = 1
    # Option 1: Pick one now and dont change until you get to an accepting state
    if option == 1:
        # First check to see if a spec is already chosen
        pickNew = 0
        if specattr[0].specToPick == []:
            pickNew = 1
        elif isinstance(specattr[0].specToPick, int):
            if potS[specattr[0].specToPick] in specattr[specattr[0].specToPick].acceptingWithCycle:
                pickNew = 1

        if pickNew:
            list_len = [len(i) for i in allRobustness]
            minVal = min(list_len)
            indices = [i for i, x in enumerate(list_len) if x == minVal]
            if len(indices) > 1:
                minFound = 0
                robustCopy = copy.deepcopy(allRobustness)
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
                specToPick = indices[indOfTrans]
            else:
                specToPick = indices[0]
            for j in range(np.size(specattr)):
                specattr[j].specToPick = specToPick
        else:
            specToPick = specattr[0].specToPick
        robustness = allRobustness[specToPick]
        props2Activate = allProps2Activate[specToPick]
    return specattr, props2Activate, potS, robustness
def findAcceptingCycle(specattr):
    reachableAccepting = []
    for j in range(np.size(specattr.acceptingWithCycle)):
        transToAccept = specattr.acceptingWithCycle[j]
        comeFrom = np.where(specattr.graph[:,transToAccept] == 1)[0]
        for k in range(np.size(comeFrom)):
            stateFrom = comeFrom[k]
            idOfResult = np.where(specattr.buchiStates[stateFrom].result == transToAccept)[0][0]
            condOfI = np.asarray(specattr.buchiStates[stateFrom].condCNF[idOfResult])

            # New stuff
            condOfI2 = condOfI[:,specattr.locOfUncontrollable]
            condOfI2 = np.unique(condOfI2, axis=0)
            acceptable = np.where((condOfI2 == specattr.inputVal).all(axis=1))[0]
            if np.size(acceptable != 0):
                reachableAccepting.append(transToAccept)
                break
    if len(reachableAccepting) == 0:
        print('Can not reach accepting state')
    return reachableAccepting

def pickTransition(specattr,allTransMax,x,props,t,wall,xR,preF, roadmap,maxV,sizeU):
    allRobust = []
    weights = [5, .75]  # What is already true, what is false right now
    allRobustId = []
    robustRef = [None] * np.size(specattr.Pi_mu)
    distFromSafeRef = [None] * np.size(specattr.Pi_mu)

    for i in range(np.size(allTransMax,0)):
        indOfActive = np.where(allTransMax[i,:]==1)[0]
        phiIndex = [specattr.controllablePropOrder[s] for s in indOfActive]
        phi = [specattr.Pi_mu[s] for s in phiIndex]
        robustness = []
        ids = []

        satisfiedPhi = [phi[s] for s in range(np.size(phi)) if phi[s].currentTruth]
        # unsatisfiedPhi = [phi[s] for s in range(np.size(phi)) if not phi[s].currentTruth]
        unsatisfiedPhi = []
        for s in range(np.size(phi)):
            if phi[s].t_e == []:
                inputTime = 0
            else:
                inputTime = phi[s].t_e
            if not phi[s].currentTruth and t >= phi[s].a + inputTime and t <= phi[s].b + inputTime:
                if hasattr(phi[s], 'satisfied'):
                    if not phi[s].satisfied:
                        unsatisfiedPhi.append(phi[s])
                else:
                    unsatisfiedPhi.append(phi[s])
        for j in range(np.size(satisfiedPhi)):
            if robustRef[satisfiedPhi[j].id] is None:
                robtemp = weights[0] * satisfiedPhi[j].distFromSafe
                robustRef[satisfiedPhi[j].id] = robtemp
                robustness.append(robtemp)
                ids.append(satisfiedPhi[j].id)
            else:
                robustness.append(robustRef[satisfiedPhi[j].id])
                ids.append(satisfiedPhi[j].id)
        for ii in range(1,int(np.size(x)/3)+1):
            phiRobot = []
            for j in range(np.size(unsatisfiedPhi)):
                if ii in unsatisfiedPhi[j].robotsInvolved:
                    phiRobot.append(unsatisfiedPhi[j])
                    ids.append(unsatisfiedPhi[j].id)
            # Check to see if nominal controllers actually conflict
            conflict = 0
            if len(phiRobot) >= 1:
                sumNom = np.zeros((1,np.size(phiRobot[0].nom,1)))
                for iii in range(np.size(phiRobot)):
                    sumNom+=phiRobot[iii].nom[1,:]
                for iii in range(np.size(phiRobot)):
                    for jj in range(np.size(phiRobot[iii].nom,1)):
                        if phiRobot[iii].nom[1,jj] > 0 and not phiRobot[iii].nom[1,jj] == sumNom[0,jj]:
                            conflict = 1
                            break
            else:
                conflict = 1
            ####
            if len(phiRobot) == 1 or conflict == 0:
                for iii in range(np.size(phiRobot)):
                    totalDist = phiRobot[iii].distFromSafe
                    time2Finish = phiRobot[iii].time2Finish
                    if preF == 1 and not isinstance(phiRobot[iii].t_e, float):
                        timeRemaining = phiRobot[iii].b
                    else:
                        if isinstance(phiRobot[iii].t_e, float):
                            timeRemaining = (phiRobot[iii].b + phiRobot[iii].t_e) - t
                        else:
                            timeRemaining = phiRobot[iii].b - t
                    timeBuffer = timeRemaining - time2Finish
                    # print('robustness: {}, timeRemaining: {}, time2Finish: {}'.format(timeBuffer, timeRemaining, time2Finish))
                    robtemp = weights[1] * timeBuffer
                    robustness.append(robtemp)
                    ids.append(phiRobot[iii].id)

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

                                if preF == 1 and not isinstance(phiRobot[0].t_e, float):
                                    timeRemaining = phiRobot[0].b
                                else:
                                    if isinstance(phiRobot[0].t_e, float):
                                        timeRemaining = (phiRobot[0].b + phiRobot[0].t_e) - t
                                    else:
                                        timeRemaining = phiRobot[0].b - t
                                timeBuffer = timeRemaining - time2Finish
                                time2FinishPrev += time2Finish
                                robtemp = weights[1] * timeBuffer
                                robustRef[phiRobot[possOrders[j][k]].id] = robtemp
                                distFromSafeRef[phiRobot[possOrders[j][k]].id] = totalDist
                                robustRow.append(robtemp)
                                idsRow.append(phiRobot[possOrders[j][k]].id)
                            else:
                                robustRow.append(robustRef[phiRobot[possOrders[j][k]].id])
                                idsRow.append(phiRobot[possOrders[j][k]].id)
                        else:
                            xTemp = copy.deepcopy(x)
                            pPoint = phiRobot[possOrders[j][k-1]].point
                            pPoint[0] = eval(str(phiRobot[possOrders[j][k-1]].point[0]))
                            try:
                                pPoint[1] = eval(str(phiRobot[possOrders[j][k-1]].point[1]))
                            except:
                                print('conflict')
                            xTemp[phiRobot[possOrders[j][k]].nom[0, 0:2].astype('int')] = pPoint
                            nom, costTemp = getNom(phiRobot[possOrders[j][k]], roadmap, xTemp, xR, maxV,sizeU)
                            distFromSafe2 = phiRobot[possOrders[j][k-1]].distFromSafe + costTemp
                            signF = phiRobot[possOrders[j][k]].signFS[0]
                            totalDist = distFromSafe2 + signF * phiRobot[possOrders[j][k]].p
                            robot = phiRobot[possOrders[j][k]].robotsInvolved[0]
                            maxVT = maxV[3 * robot - 3]
                            time2Finish = totalDist / maxVT
                            if preF == 1 and not isinstance(phiRobot[possOrders[j][k]].t_e, float):
                                timeRemaining = phiRobot[possOrders[j][k]].b
                            else:
                                timeRemaining = (phiRobot[possOrders[j][k]].b + phiRobot[possOrders[j][k]].t_e) - t
                            timeBuffer = timeRemaining - (time2Finish + time2FinishPrev)
                            time2FinishPrev += time2Finish
                            idsRow.append(phiRobot[possOrders[j][k]].id)
                            robustRow.append(weights[1] * timeBuffer)
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
            try:
                minInd.append(np.argmin(allRobustCopy[i]))
            except:
                allRobustCopy[i] = [100900000]
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

    # robustness = allRobustCopy[indOfTrans]
    robustness = allRobust[np.where(np.all(allTransMax == trans2Make,axis=1))[0][0]]

    ids = allRobustId[np.where(np.all(allTransMax == trans2Make,axis=1))[0][0]]
    robustRef = robustRef
    distFromSafeRef = distFromSafeRef
    return trans2Make, robustness,ids

def checkUntil(specattr, allTransitions):
    for i in range(np.size(specattr.Pi_mu)):
        if specattr.Pi_mu[i].phi_unt != 0:
            propOfI = re.split('\.',specattr.Pi_mu[i].prop_label)[-1]
            propOfI2 = specattr.Pi_mu[i].phiUntil
            ia = specattr.controllablePropOrder[list(specattr.controllableProp).index(propOfI)]
            ia2 = []
            splitPhiUntil = re.split('(\||\&)', propOfI2)
            for j in range(np.size(splitPhiUntil)):
                if '&' not in splitPhiUntil[j] and '|' not in splitPhiUntil[j]:
                    predOfI = re.search('(?<=props\.).+?(?=(\)|\s))', splitPhiUntil[j])[0]
                    locOfPred = list(specattr.controllableProp).index(predOfI)
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

def k_shortest_paths(G, source, target, k, weight=None):
    return list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))