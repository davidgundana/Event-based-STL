import numpy as np
import copy
import matrixDijkstra
import re
import networkx as nx
from itertools import islice

class StatesOfI:
    def __init__(self):
        self.cond = [] # Conditions in a transition
        self.result = [] # result of transitions
        self.condCNF = [] # Simplified conditions

def buchiIntersect(buchi1, s01, buchi2, s02):
    # specattr = copy.deepcopy(buchi1)
    # specattr2 = copy.deepcopy(buchi2)
    specattr = buchi1
    specattr2 = buchi2
    states = [(s01, s02, 0)]
    print('calculating accepting states')
    accepting1 = buchi1.acceptingWithCycle
    buchi1 = buchi1.buchiStates
    accepting2 = buchi2.acceptingWithCycle
    print('found accepting states')
    accepting3 = []
    buchi2 = buchi2.buchiStates
    buchi3Start = []
    buchi3Result = []
    buchi3Edges = []
    buchi3 = []
    state_update = []
    count = 0
    new_list = []
    stateRef = [(0,0,0)]
    while len(states) != 0:
        print(len(states), len(stateRef))
        for s in states:
            buchi3.append(StatesOfI())
            sOf1, sOf2, accPrev = s
            # if sOf1 == 0 and sOf2 == 0 and accPrev == 2:
            #     print('here')
            #Loop through all neighbors (results) of sOf1 in buchi1
            for i in range(np.size(buchi1[sOf1].result)):
                for j in range(np.size(buchi2[sOf2].result)):
                    n1 = buchi1[sOf1].result[i]
                    n2 = buchi2[sOf2].result[j]
                    try:
                        formula1 = buchi1[sOf1].condCNF[n1]
                        formula2 = buchi2[sOf2].condCNF[n2]
                    except:
                        continue
                    formula1Word = buchi1[sOf1].cond[n1]
                    formula2Word = buchi2[sOf2].cond[n2]
                    newFormula = []
                    #build new formula
                    for k in range(len(formula1)):
                        for l in range(len(formula2)):
                            newFormula.append(formula1[k] + formula2[l])
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
                    # buchi3Edges.append(newFormula)
                    buchi3Start.append((sOf1,sOf2,accPrev))
                    buchi3[-1].condCNF.append(newFormula)
                    buchi3[-1].result.append(stateRef.index((n1,n2,acc)))
                    buchi3[-1].cond.append('(('+formula1Word + ') and (' + formula2Word + '))')
        # states = copy.deepcopy(new_list)
        states = new_list
        new_list = []
    print('done')
    accepting_states = np.unique(accepting3)
    graph = np.zeros((np.size(buchi3), np.size(buchi3)))
    for i in range(np.size(buchi3)):
        for j in range(np.size(buchi3[i].result)):
            graph[i, int(buchi3[i].result[j])] = 1
    print('finding accepting states')
    # Find accepting states with a cycle
    acceptingWithCycle = []
    for j in range(len(accepting_states)):
        if graph[accepting_states[j], accepting_states[j]] != 1:
            (cost, rute) = matrixDijkstra.dijkstra(graph, accepting_states[j], accepting_states[j])
        else:
            cost = 1
            rute = np.array([accepting_states[j], accepting_states[j]])
        costRef = np.size(rute, 0) - 1
        if not cost > 10000 or costRef == cost:
            acceptingWithCycle.append(accepting_states[j])

    #formatt everything correctly
    specattr.Pi_mu += specattr2.Pi_mu
    specattr.acceptingWithCycle = acceptingWithCycle
    specattr.accepting_states = accepting_states
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
    specattr.evProps.__dict__.update(specattr2.evProps.__dict__)
    specattr.graph = graph
    specattr.inpRef += specattr2.inpRef
    # specattr.inpRef = [list(x) for x in set(tuple(x) for x in specattr.inpRef)]
    specattr.inpLabels += specattr2.inpLabels
    # specattr.inpLabels = [list(x) for x in set(tuple(x) for x in specattr.inpLabels)]
    specattr.input = np.hstack((specattr.input, specattr2.input))
    specattr.inputVal = np.hstack((specattr.inputVal, specattr2.inputVal))
    specattr.propositions += specattr2.propositions
    specattr.uncontrollableProp += specattr2.uncontrollableProp
    specattr.locOfUncontrollable = [list(specattr.propositions).index(s) for s in list(specattr.uncontrollableProp)]
    specattr.locOfControllable = np.asarray([list(specattr.propositions).index(s) for s in list(specattr.controllableProp)])
    specattr.N = np.size(specattr.uncontrollableProp)
    specattr.nRoutes = [[]] * len(specattr.graph)
    print('finding routes')
    # for i in range(np.size(specattr.graph, 0)):
    #     tempRoute = []
    #     for j in range(np.size(specattr.acceptingWithCycle, 0)):
    #         goTo = specattr.acceptingWithCycle[j]
    #         try:
    #             allPaths = findNRoutes(specattr.graph, i, goTo)
    #             shortestLength = len(min(allPaths, key=len))
    #             allPaths = [path for path in allPaths if len(path) == shortestLength]
    #
    #             #double check first transition
    #             if int(specattr.graph[allPaths[0][0],allPaths[0][1]]):
    #                 tempRoute.append(allPaths)
    #             else:
    #                 if int(specattr.graph[allPaths[0][-1],allPaths[0][-2]]):
    #                     allPaths[0].reverse()
    #                     tempRoute.append(allPaths)
    #                 else:
    #                     tempRoute.append([])
    #         except:
    #             pass
    #     specattr.nRoutes[i] = tempRoute
    specattr.nRoutes = []
    specattr.parameters += specattr2.parameters
    specattr.props.__dict__.update(specattr2.props.__dict__)    # specattr.uncontrollableProp =
    specattr.wall += specattr2.wall

    return specattr

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

