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
import itertools


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
        if not isinstance(specattr[b].buchiStates[potS[b]].cond, tuple):
            evalCond = eval(','.join(specattr[b].buchiStates[potS[b]].cond), {'__builtins__': None}, {'props': props})
            evalCond = np.multiply(evalCond, 1)
        else:
            evalCond = np.ones(len(specattr[b].buchiStates[potS[b]].cond[0]))
            for i in range(len(specattr[b].buchiStates[potS[b]].cond)):
                evalCond *= np.multiply(eval(','.join(specattr[b].buchiStates[potS[b]].cond[i]), {'__builtins__': None}, {'props': props}),1)

        result = np.array(specattr[b].buchiStates[potS[b]].result)
        potentialNextStates = result[np.where(evalCond == 1)[0]]

        # First find all accepting states that can be reached given uncontrollable propositions
        reachableAcceptCycle = findAcceptingCycle(specattr[b])
        if np.size(potentialNextStates) == 0:
            print('no state')
        # print(time.time()-t1)
        paths2Consider = []
        for i in range(np.size(potentialNextStates)):
            potState = potentialNextStates[i]
            for j in range(np.size(reachableAcceptCycle)):
                if specattr[b].nRoutes[potState,reachableAcceptCycle[j]] is None:
                    allPaths = []
                    if potState == reachableAcceptCycle[j]:
                        if specattr[b].graph[potState,potState] == 1:
                            allPaths.append([potState,potState])
                        else:
                            statesTo = np.where(specattr[b].graph[:,potState]==1)[0]
                            for jj in range(np.size(statesTo)):
                                for path in k_shortest_paths(specattr[b].G, potState, statesTo[jj], 5):
                                    allPaths.append(path+ [potState])
                    else:
                        for path in k_shortest_paths(specattr[b].G, potState, reachableAcceptCycle[j], 5):
                            allPaths.append(path)
                    specattr[b].nRoutes[potState,reachableAcceptCycle[j]] = allPaths
                    paths2Consider.append(allPaths)
                else:
                    paths2Consider.append(specattr[b].nRoutes[potState,reachableAcceptCycle[j]])
        paths2Consider = [item for sublist in paths2Consider for item in sublist]
        if np.size(paths2Consider) == 0:
            print('no paths!')

        # check if path is valid with uncontrollable propositions
        # paths2Consider = checkPaths(paths2Consider)

        # Consider only shortest paths
        if isinstance(paths2Consider[0], list):
            pathLength = np.asarray([len(x) for x in paths2Consider])
            pathIndices = np.where(pathLength == min(pathLength))[0].tolist()
            paths2Consider = [paths2Consider[i][0:2] for i in pathIndices]
        else:
            paths2Consider = paths2Consider[0:2]


        # We only care about the first transition for now
        paths2Consider = np.asarray(paths2Consider)
        paths2Consider = np.atleast_2d(paths2Consider)
        paths2Consider = np.unique(paths2Consider, axis=0)

        # Compute robustness for each path
        # first find all activated propositions for all transitions
        allTransitionsWithState = np.empty((1, len(specattr[b].controllableProp) + 1), dtype=int)
        allTransitionsWithNextState = np.empty((1, len(specattr[b].controllableProp) + 1), dtype=int)

        propRef = []
        for i in range(np.size(paths2Consider, 0)):
            results = np.array(specattr[b].buchiStates[int(paths2Consider[i][0])].result)
            indCondToTran = np.where(results == int(paths2Consider[i][1]))[0]
            if specattr[b].buchiStates[int(paths2Consider[i][0])].condCNF[indCondToTran[0]] == [] or isinstance(specattr[b].buchiStates[int(paths2Consider[i][0])].condCNF[indCondToTran[0]],tuple):
                specattr[b] = addCNF(specattr[b],int(paths2Consider[i][0]),indCondToTran[0])

            transitionOptions = specattr[b].buchiStates[int(paths2Consider[i][0])].condCNF[indCondToTran[0]]
            possibleTransitions = np.asarray(transitionOptions)[:, specattr[b].locOfUncontrollable]
            acceptable = np.where((possibleTransitions == specattr[b].inputVal).all(axis = 1))[0]
            transitionOptions = np.asarray(transitionOptions)[acceptable, :]
            transitionOptions = np.unique(transitionOptions[:,specattr[b].locOfControllable],axis = 0)

            if np.size(acceptable != 0) and np.size(transitionOptions,0) != 0:
                stateRef = paths2Consider[i][0] * np.ones((np.size(transitionOptions, 0), 1), dtype=int)
                nextRef = paths2Consider[i][1] * np.ones((np.size(transitionOptions, 0), 1), dtype=int)
                transitionPot = np.hstack((transitionOptions,nextRef))
                transitionOptions = np.hstack((transitionOptions, stateRef))
                allTransitionsWithState = np.vstack((allTransitionsWithState, transitionOptions))
                allTransitionsWithNextState = np.vstack((allTransitionsWithNextState,transitionPot))
                for j in range(np.size(transitionOptions, 0)):
                    propRef.extend([i for i, x in enumerate(transitionOptions[j, :-1]) if x == 1])
            else:
                # print('no transition options for this path')
                pass

        allTransitions, idx = np.unique(allTransitionsWithState[1:, :-1], axis=0,return_index=True)
        allTransitions = allTransitionsWithState[1:, :-1][np.sort(idx)]
        allTransitionsWithState = allTransitionsWithState[1:, :]
        # check if any propositions have until tag
        allTransitions = checkUntil(specattr[b], allTransitions)

        # first we choose the transition with the most infinities. this is the highest robustness
        numMax = np.count_nonzero(np.isinf(allTransitions), axis=1)
        if np.size(numMax) == 0:
            print('No transitions')
        maxLoc = np.argwhere(numMax == np.amax(numMax)).ravel()

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
        potS[b] = int(allTransitionsWithState[stateLoc, -1])

        propsLoc = (np.where(trans2Make == 1)[0]).tolist()
        props2Activate = [specattr[b].controllableProp[element] for element in propsLoc]

        allProps2Activate.append(props2Activate)
        allRobustness.append(robustness)
        # print(time.time()-t1)

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
    indOfInput = np.where((specattr.acceptableLoop[:,:-1] == specattr.inputVal).all(axis=1))[0][0]
    if specattr.acceptableLoop[indOfInput,-1] is None:
        reachableAccepting = []
        for j in range(np.size(specattr.acceptingWithCycle)):
            transToAccept = specattr.acceptingWithCycle[j]
            comeFrom = np.where(specattr.graph[:,transToAccept] == 1)[0]
            for k in range(np.size(comeFrom)):
                stateFrom = comeFrom[k]
                idOfResult = specattr.buchiStates[stateFrom].result.index(transToAccept)

                if specattr.buchiStates[stateFrom].condCNF[idOfResult] == [] or isinstance(specattr.buchiStates[stateFrom].condCNF[idOfResult],tuple):
                    specattr = addCNF(specattr,stateFrom,idOfResult)
                condOfI = np.asarray(specattr.buchiStates[stateFrom].condCNF[idOfResult])

                condOfI2 = condOfI[:,specattr.locOfUncontrollable]
                condOfI2 = np.unique(condOfI2, axis=0)

                acceptable = np.where((condOfI2 == specattr.inputVal).all(axis=1))[0]

                if np.size(acceptable != 0):
                    reachableAccepting.append(transToAccept)
                    break
        if len(reachableAccepting) == 0:
            print('Can not reach accepting state')
        specattr.acceptableLoop[indOfInput,-1] = reachableAccepting
    else:
        reachableAccepting = specattr.acceptableLoop[indOfInput,-1]

    return reachableAccepting

def addCNF(specattr, sFrom, sTo, i = False):
    allRCond = np.zeros((1,len(specattr.propositions)),dtype = int)
    if not isinstance(specattr.buchiStates[sFrom].cond,tuple):
        optionsForTrans = re.split('or', specattr.buchiStates[sFrom].cond[sTo])
    else:
        optionsForTrans = []
        option = re.split('or', specattr.buchiStates[sFrom].cond[0][sTo])
        for ii in range(1,len(specattr.buchiStates[sFrom].cond)):
            options2 = re.split('or', specattr.buchiStates[sFrom].cond[ii][sTo])
            for kk in range(np.size(options2)):
                optionsForTrans.append([ind + ' and ' + options2[kk] for ind in option])
        optionsForTrans = [item for sublist in optionsForTrans for item in sublist]
    for k in range(np.size(optionsForTrans)):
        thisOption = re.split('and', optionsForTrans[k])
        CNFrep = np.multiply(['not' not in e for e in thisOption],1)
        propsPresent = re.findall('(?<=\.)\w*', optionsForTrans[k])
        refNum = [specattr.propositions.index(x) for x in propsPresent]
        finalPropVals = np.asarray([np.inf] * np.size(specattr.propositions))
        finalPropVals[specattr.locOfUncontrollable] = np.size(specattr.uncontrollableProp) * [2]
        finalPropVals[refNum] = CNFrep
        if np.size(np.where(finalPropVals==2)) > 0:
            lst = list(map(list, itertools.product([0, 1], repeat=np.size(np.where(finalPropVals==2)))))
            allR = np.repeat(np.asarray([finalPropVals]), len(lst), axis=0)
            for ii in range(len(lst)):
                locOfChange = np.where(allR[ii, :] == 2)[0]
                allR[ii, locOfChange] = lst[ii]

            allRCond = np.vstack((allRCond, allR))
        else:
            allRCond = np.vstack((allRCond, finalPropVals))
    if np.size(allRCond,0) != 1:
        allRCond = allRCond[1:]
    allRCond = np.unique(allRCond,axis = 0)
    allRCond = allRCond.tolist()

    specattr.buchiStates[sFrom].condCNF[sTo] = allRCond


    return specattr
def pickTransition(specattr,allTransMax,x,props,t,wall,xR,preF, roadmap,maxV,sizeU):
    allRobust = []
    weights = [5, .75]  # What is already true, what is false right now
    allRobustId = []

    for i in range(np.size(allTransMax,0)):
        indOfActive = np.where(allTransMax[i,:]==1)[0]
        phiIndex = [specattr.controllablePropOrder[s] for s in indOfActive]
        phi = [specattr.Pi_mu[s] for s in phiIndex]
        robustness = []
        ids = []

        satisfiedPhi = [phi[s] for s in range(np.size(phi)) if phi[s].currentTruth]
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
            robtemp = weights[0] * satisfiedPhi[j].distFromSafe
            robustness.append(robtemp)
            ids.append(satisfiedPhi[j].id)

        for ii in range(1,int(np.size(x)/3)+1):
            phiRobot = []
            for j in range(np.size(unsatisfiedPhi)):
                if ii in unsatisfiedPhi[j].robotsInvolved:
                    phiRobot.append(unsatisfiedPhi[j])
                    ids.append(unsatisfiedPhi[j].id)
            # Check to see if nominal controllers actually conflict
            conflict = 0
            # if len(phiRobot) >= 1:
            #     sumNom = np.zeros((1,np.size(phiRobot[0].nom,1)))
            #     for iii in range(np.size(phiRobot)):
            #         sumNom+=phiRobot[iii].nom[1,:]
            #     for iii in range(np.size(phiRobot)):
            #         for jj in range(np.size(phiRobot[iii].nom,1)):
            #             if phiRobot[iii].nom[1,jj] > 0 and not phiRobot[iii].nom[1,jj] == sumNom[0,jj]:
            #                 conflict = 1
            #                 break
            # else:
            #     conflict = 1
            ####
            if len(phiRobot) == 1 or conflict == 0:
                for iii in range(np.size(phiRobot)):
                    time2Finish = phiRobot[iii].time2Finish
                    if preF == 1 and not isinstance(phiRobot[iii].t_e, float):
                        timeRemaining = phiRobot[iii].b
                    else:
                        if isinstance(phiRobot[iii].t_e, float):
                            timeRemaining = (phiRobot[iii].b + phiRobot[iii].t_e) - t
                        else:
                            timeRemaining = phiRobot[iii].b - t
                    timeBuffer = timeRemaining - time2Finish
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
                allRobustCopy[i] = [100000000]
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
    robustness = allRobust[np.where(np.all(allTransMax == trans2Make,axis=1))[0][0]]
    ids = allRobustId[np.where(np.all(allTransMax == trans2Make,axis=1))[0][0]]

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

def checkPaths(paths2Consider):
    if isinstance(paths2Consider[0], list):
        rowToDel = []
        for i in range(np.size(paths2Consider)):
            for j in range(np.size(paths2Consider[i])-1):
                sFrom = paths2Consider[i][j]
                sTo = paths2Consider[i][j+1]
                indCondToTran = specattr[b].buchiStates[sFrom].result.index(sTo)
                if isinstance(specattr[b].buchiStates[sFrom].cond, tuple):
                    evalCond = np.ones(len(specattr[b].buchiStates[sFrom].cond[0]))
                    for l in range(len(specattr[b].buchiStates[sFrom].cond)):
                        evalCond *= np.multiply(eval(','.join(specattr[b].buchiStates[sFrom].cond[l]), {'__builtins__': None},
                                 {'props': props}), 1)
                else:
                    conditions = specattr[b].buchiStates[sFrom].cond[indCondToTran]
                    evalCond = eval(','.join(conditions), {'__builtins__': None},{'props': props})
                    evalCond = np.multiply(evalCond, 1)
                potNext = np.where(evalCond == 1)[0]
                if np.size(potNext) == 0:
                    # Cant actually make this transition
                    rowToDel.append(i)
                    print('cant go from {} to {}'.format(sFrom,sTo))
                    break