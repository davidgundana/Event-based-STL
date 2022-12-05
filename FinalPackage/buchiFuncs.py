import numpy as np
import copy
import matrixDijkstra
import re
import networkx as nx
from itertools import islice
import time
import spot
import itertools


class StatesOfI:
    def __init__(self,cond=None,result=None,condCNF=None):
        if cond is None:
            self.cond = []
            self.result = []
            self.condCNF = []
        else:
            self.cond = cond # Conditions in a transition
            self.result = result # result of transitions
            self.condCNF = condCNF # Simplified conditions

    def __eq__(self, other):
        if (isinstance(other, StatesOfI)):
            return self.cond == other.cond and self.result == other.result and self.condCNF == other.condCNF
        return false

def buchiIntersect(originalBuchi, s01, newBuchi, s02):
    forLoopMethod = 0
    if forLoopMethod:
        t1 = time.time()
        specattr = originalBuchi
        specattr2 = newBuchi
        states = [(s01, s02, 0)]
        accepting1 = originalBuchi.acceptingWithCycle
        # accepting1 = originalBuchi.accepting_states
        buchi1 = originalBuchi.buchiStates
        accepting2 = newBuchi.acceptingWithCycle
        # accepting2 = newBuchi.accepting_states
        accepting3 = []
        buchi2 = newBuchi.buchiStates
        buchi3Start = []
        buchi3Result = []
        buchi3 = []
        new_list = []
        stateRef = [(0,0,0)]
        while len(states) != 0:
            t2 = time.time()
            for s in states:
                buchi3.append(StatesOfI())
                sOf1, sOf2, accPrev = s
                conditions1 = []
                conditions2 = []
                #Loop through all neighbors (results) of sOf1 in buchi1
                for i in range(np.size(buchi1[sOf1].result)):
                    for j in range(np.size(buchi2[sOf2].result)):
                        n1 = buchi1[sOf1].result[i]
                        n2 = buchi2[sOf2].result[j]

                        # formulas do not conflict so update acc
                        if n1 in accepting1 and accPrev == 0:
                            acc = 1
                        # elif n2 in accepting2 and n1 in accepting1 and accPrev == 1:
                        elif n2 in accepting2 and accPrev == 1:
                            acc = 2
                        # elif accPrev == 2 and not (n1 == sOf1 and n2 == sOf2):
                        elif accPrev == 2:
                            acc = 0
                        else:
                            acc = accPrev

                        if (n1,n2,acc) not in buchi3Result and (n1,n2,acc) not in buchi3Start:
                            new_list.append((n1,n2,acc))
                            stateRef.append((n1,n2,acc))
                        if acc == 2:
                            accepting3.append(stateRef.index((n1,n2,acc)))
                        buchi3Result.append((n1,n2,acc))
                        buchi3Start.append((sOf1,sOf2,accPrev))
                        buchi3[-1].condCNF.append(([[]], [[]]))
                        buchi3[-1].result.append(stateRef.index((n1,n2,acc)))
                        conditions1.append(buchi1[sOf1].cond[i])
                        conditions2.append(buchi2[sOf2].cond[j])

                buchi3[-1].cond = (conditions1,conditions2)
            print('loop time for {}: {}'.format(len(states),time.time()-t2))
            states = new_list
            new_list = []

        print(time.time()-t1)
        buchiRef = copy.deepcopy(buchi3)
        resultRef = copy.deepcopy(buchi3Result)
        acceptRef = copy.deepcopy(accepting3)

    t1 = time.time()
    specattr = originalBuchi
    specattr2 = newBuchi
    states = [(s01, s02, 0)]
    accepting1 = originalBuchi.acceptingWithCycle
    # accepting1 = originalBuchi.accepting_states
    buchi1 = originalBuchi.buchiStates
    accepting2 = newBuchi.acceptingWithCycle
    # accepting2 = newBuchi.accepting_states
    accepting3 = []
    buchi2 = newBuchi.buchiStates
    buchi3Start = []
    buchi3Result = []
    buchi3 = []
    graphRef = []
    new_list = []
    stateRef = [(0,0,0)]
    while len(states) != 0:
        n1Vec = [buchi1[s[0]].result for s in states]
        n2Vec = [buchi2[s[1]].result for s in states]
        sOf1Vec = [[s[0]]*len(n1Vec[0]) for s in states]
        sOf2Vec = [[s[1]]*len(n1Vec[0]) for s in states]
        accPrevVec = [s[2] for s in states]

        if len(n2Vec[0]) > 1:
            print('need to correct for larger 2nd buchi')
        vec = np.zeros((8*len(n1Vec),np.size(n1Vec,1)*np.size(n2Vec,1)))

        # result1, result2, accPrev, in accepting1, in accepting2, acc

        vec[0::8,:] = np.array(n1Vec)
        vec[1::8,:] = np.array(n2Vec)
        vec[2::8,:] = np.outer(np.array(accPrevVec), np.ones(np.size(n1Vec,1)*np.size(n2Vec,1)))

        indT = (np.in1d(vec[0::8, :], np.array(accepting1)) * 1).nonzero()[0]
        tempVec = vec[3::8,:].reshape(np.size(vec[0::8,:]))
        tempVec[indT] = 1
        vec[3::8,:] = tempVec.reshape(len(states),np.size(vec,1))

        indT2 = (np.in1d(vec[1::8, :], np.array(accepting2)) * 1).nonzero()[0]
        tempVec = vec[4::8,:].reshape(np.size(vec[0::8,:]))
        tempVec[indT2] = 1
        vec[4::8, :] = tempVec.reshape(len(states), np.size(vec, 1))

        vec[5::8,:] = vec[2::8,:]

        tempVec = vec[5::8,:].reshape(np.size(vec[0::8,:]))
        cond1T = np.where(((vec[3::8,:].reshape(np.size(vec[0::8,:])) == 1)*1) + ((vec[2::8,:].reshape(np.size(vec[0::8,:])) == 0) * 1) == 2)[0]
        tempVec[cond1T] = 1
        vec[5::8,:] = tempVec.reshape(len(states), np.size(vec, 1))

        tempVec = vec[5::8,:].reshape(np.size(vec[0::8,:]))
        cond2T = np.where((((vec[4::8,:].reshape(np.size(vec[0::8,:])) == 1)*1) + ((vec[2::8,:].reshape(np.size(vec[0::8,:])) == 1) * 1)) ==2)[0]
        cond2FinalT = np.setdiff1d(cond2T,cond1T)
        tempVec[cond2FinalT] = 2
        vec[5::8,:] = tempVec.reshape(len(states), np.size(vec, 1))

        tempVec = vec[5::8,:].reshape(np.size(vec[0::8,:]))
        cond3T = np.where(vec[2::8,:].reshape(np.size(vec[0::8,:]))==2)[0]
        cond12T = np.hstack((cond1T, cond2T))
        cond3FinalT = np.setdiff1d(cond3T,cond12T)
        tempVec[cond3FinalT] = 0
        vec[5::8,:] = tempVec.reshape(len(states), np.size(vec, 1))

        vec[6::8,:] = sOf1Vec
        vec[7::8,:] = sOf2Vec

        # flatVec = vec.reshape(8,np.size(vec,1)*len(states))
        s0 = vec[0::8,:].flatten()
        s1 = vec[1::8,:].flatten()
        s2 = vec[2::8,:].flatten()
        s5 = vec[5::8,:].flatten()
        s6 = vec[6::8,:].flatten()
        s7 = vec[7::8,:].flatten()
        stateVec = np.vstack((s0,s1,s5)).T
        # stateVec = flatVec[[0,1,5],:].T
        stateTuple = np.array([tuple(e) for e in stateVec],dtype="i,i,i")
        # startVec = vec[[6,7,2],:].T
        startVec = np.vstack((s6,s7,s2)).T
        startTuple = np.array([tuple(e) for e in startVec],dtype="i,i,i")

        stateTupleU = np.unique(stateTuple)
        startTupleU = np.unique(startTuple)
        resultArray = np.array(buchi3Result,dtype='i,i,i')
        startArray = np.array(buchi3Start,dtype='i,i,i')

        _,notAppend1,_ = np.intersect1d(stateTupleU,resultArray, return_indices=True)
        _,notAppend2,_ = np.intersect1d(stateTupleU,startArray, return_indices=True)
        notAppendTotal = list(set(list(notAppend1) + list(notAppend2)))

        # print(time.time()-tTest)

        statesToApp = np.delete(stateTupleU,notAppendTotal)
        new_list += list(statesToApp)
        stateRef += list(statesToApp)

        # newAccept = list(stateVec[np.where(stateVec[:,2]==2)[0]])
        # if newAccept != []:
        #     accepting3 += newAccept

        buchi3Result += list(map(tuple,stateTupleU))
        buchi3Start += list(map(tuple,startTupleU))
        buchi3Result = list(set(buchi3Result))
        buchi3Start = list(set(buchi3Start))

        condCNF = [([[]], [[]])] *  np.size(n1Vec,1)
        sRef = np.array(stateRef,dtype='i,i,i')
        refSorted = np.argsort(sRef)
        stateSearch = np.searchsorted(sRef[refSorted], stateTuple)
        allResult = refSorted[stateSearch]
        allResult = allResult.reshape(len(states),np.size(n1Vec,1))
        allResult = allResult.tolist()

        for i in range(len(states)):
            result = allResult[i]
            conditions1 = buchi1[sOf1Vec[i][0]].cond[:]*np.size(n2Vec[0])
            conditions2 = list(np.repeat(buchi2[sOf2Vec[i][0]].cond[:],np.size(n1Vec[0])))
            buchi3.append(StatesOfI((conditions1,conditions2),result,condCNF))
            graphRef += np.vstack(([len(buchi3)-1] * len(result), result)).T.tolist()

        states = new_list
        new_list = []

    # print(time.time()-t1)

    acceptingArray = np.array(list(map(tuple,stateRef)))
    accepting_states = list(np.where(acceptingArray[:,2] == 2)[0])
    graph = np.zeros((np.size(buchi3), np.size(buchi3)))

    G = nx.DiGraph()
    G.add_nodes_from(range(0, np.size(graph, 0)))
    G.add_edges_from(graphRef)
    graphRefArray = np.array(graphRef)
    graph[graphRefArray[:, 0], graphRefArray[:, 1]] = 1
    # for i in range(np.size(buchi3)):
    #     for j in range(np.size(buchi3[i].result)):
    #         graph[i, int(buchi3[i].result[j])] = 1
    #         G.add_edge(i, int(buchi3[i].result[j]))

    #formatt everything correctly
    specattr.Pi_mu += specattr2.Pi_mu
    specattr.acceptingWithCycle = accepting_states
    specattr.accepting_states = accepting_states
    # if specattr2.acceptableLoop[0,0] is not None:
    totalN = specattr.N + specattr2.N
    acceptableLoop = np.array(list(itertools.product([0, 1], repeat=totalN)))
    sizeCombo = np.size(acceptableLoop,0)
    specattr.acceptableLoop = np.hstack((acceptableLoop,np.atleast_2d(np.full(sizeCombo,None)).T))

    specattr.buchi = []
    specattr.buchiStates = buchi3
    specattr.controllableProp += specattr2.controllableProp
    specattr.controllablePropOrder = []
    for i in range(np.size(specattr.controllableProp)):
        propName = specattr.controllableProp[i]
        # find the phi that matches this prop
        for j in range(np.size(specattr.Pi_mu)):
            isThere = re.findall(propName, specattr.Pi_mu[j].prop_label)
            if isThere:
                break
        if isThere:
            specattr.controllablePropOrder.append(j)
    try:
        specattr.evProps.__dict__.update(specattr2.evProps.__dict__)
    except:
        pass
    specattr.graph = graph
    specattr.G = G
    specattr.inpRef += specattr2.inpRef
    specattr.inpLabels += specattr2.inpLabels
    specattr.input = np.hstack((specattr.input, specattr2.input))
    specattr.propositions += specattr2.propositions
    specattr.uncontrollableProp += specattr2.uncontrollableProp
    specattr.locOfUncontrollable = [list(specattr.propositions).index(s) for s in list(specattr.uncontrollableProp)]
    specattr.locOfControllable = np.asarray([list(specattr.propositions).index(s) for s in list(specattr.controllableProp)])
    specattr.N = np.size(specattr.uncontrollableProp)
    specattr.nRoutes = np.full((len(specattr.graph),len(specattr.graph)), None)
    specattr.parameters += specattr2.parameters
    specattr.props.__dict__.update(specattr2.props.__dict__)    # specattr.uncontrollableProp =
    currState = list(map(tuple,stateRef)).index((s01,s02,0))
    # print(time.time()-t1)
    return specattr, currState

def findNRoutes(graph,startState, endState):
    numP = 5
    G = nx.DiGraph()
    G.add_nodes_from(range(0, np.size(graph, 0)))
    for jj in range(np.size(graph, 0)):
        for ii in range(np.size(graph, 0)):
            if graph[ii, jj] == 1:
                G.add_edge(ii, jj)
    allPaths = []
    if startState != endState:
        for path in k_shortest_paths(G, startState, endState, numP):
            allPaths.append(path)
    else:
        if graph[startState,endState] == 1:
            allPaths.append([startState,endState])
        else:
            # Find states that can go to the accepting state
            statesFrom = np.where(graph[:,startState]==1)[0]
            for i in range(np.size(statesFrom)):
                for path in k_shortest_paths(G, startState, statesFrom[i], numP):
                    allPaths.append(path + [startState])

            # (cost, rute) = matrixDijkstra.dijkstra(graph, startState, statesFrom[i])
            # allPaths.append(rute.astype('int').tolist())

    return allPaths

def k_shortest_paths(G, source, target, k, weight=None):
    return list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))

def construct_buchi_intersect(buchi1, buchi2, init_buchi1, init_buchi2, accepting_states1, accepting_states2):
    ''' given two buchi's, construct buchi intersection
    while buchi_state has successor states:
        buchi_edges = find edges going out of buchi state
        for each state in caps
            for each neighbor from state:
                for each edge in buchi_edges
                    check if neighbor & edge conflict
                    if not: add nodes (state,buchi_state) and (state,buchi_state_neighbor)
                            add edge with weight of state,neighbor

    '''

    forward_prod = nx.DiGraph()

    # state_list = buchi1.nodes()
    state_list = [init_buchi1]

    new_list = set()
    count = 0

    while len(state_list) != 0:
        for state in state_list:
            buchi1_state = state[0]
            if count == 0:
                buchi1_state = init_buchi1
                buchi2_state = init_buchi2
                accept_condition_prev = 0
            else:
                # env_state_current = state[0][0]
                buchi1_state = state[0]
                buchi2_state = state[1]
                accept_condition_prev = state[2]

            for neighbor2 in list(buchi2.successors(buchi2_state)):
                for neighbor1 in list(buchi1.successors(buchi1_state)):
                    buchi_formula1 = buchi1.get_edge_data(buchi1_state, neighbor1)[0]['label'].strip('"')
                    buchi_formula2 = buchi2.get_edge_data(buchi2_state, neighbor2)[0]['label'].strip('"')

                    # ap_dict = buchi1_dict[buchi_formula1]

                    if buchi_formula2 == '1':
                        check_conflict = True
                    else:

                        buchi_formula1_upd = buchi_formula1.replace("!", "~")
                        buchi_formula2_upd = buchi_formula2.replace("!", "~")
                        # symplify expression. =0 if there are conflicting propositions
                        check_conflict = expr.expr(
                            '(' + buchi_formula1_upd + ')' + ' & ' + '(' + buchi_formula2_upd + ')').to_dnf()

                    if check_conflict != expr.expr(0):

                        # figure out if 3rd element in tuple should be 0, 1, or 2

                        if neighbor1 in accepting_states1 and accept_condition_prev == 0:
                            # if accepting state of buchi1
                            accept_condition = 1
                        elif neighbor2 in accepting_states2 and accept_condition_prev == 1:
                            # if accepting state of buchi2 and previous was accepting state of buchi1
                            accept_condition = 2
                        elif accept_condition_prev == 2:
                            # if previously both were accepting states
                            accept_condition = 0
                        else:
                            accept_condition = accept_condition_prev

                        if (neighbor1, neighbor2, accept_condition) not in forward_prod.nodes():
                            new_list.add((neighbor1, neighbor2, accept_condition))
                        # label accepting states
                        if accept_condition == 2:
                            forward_prod.add_node((neighbor1, neighbor2, accept_condition), accepting='F')
                        else:
                            forward_prod.add_node((neighbor1, neighbor2, accept_condition))

                        new_edge = '(' + buchi_formula1 + ')' + ' & ' + '(' + buchi_formula2 + ')'
                        # forward_prod.add_edge(((env_state_current, state_a), buchi_state),(neighbor_a_new, neighbor_b), weight=weight)
                        forward_prod.add_edge((buchi1_state, buchi2_state, accept_condition_prev),
                                              (neighbor1, neighbor2, accept_condition), label=new_edge)
                        # else:
                        #     print('F', buchi_formula, ap_dict)
        state_list = new_list.copy()
        new_list = set()
        count += 1
    return forward_prod

