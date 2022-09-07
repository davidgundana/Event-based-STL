import copy

import numpy as np
import re
import runEvBasedSTL
import pickle
import matrixDijkstra
import networkx as nx
import os
from itertools import islice

class spec:
    def __init__(self,spec,accepting):
        self.spec = spec # specification
        self.accepting_states = accepting # pre-allocate accepting states
        self.controllableProp = [] # pre-allocate controllable propositions
        self.props = [] # value of propositions
        self.propositions = []
        self.N = []
        self.initSpec()


    def initSpec(self):
        # Replace the exclamation points with readable values for python and create an object with the
        # values of all propositions
        propo = [re.findall('(?<=\.)\w*', elem) for elem in self.spec]
        flat_propo = [item for sublist in propo for item in sublist]
        indexes = np.unique(flat_propo, return_index=True)[1]
        propositions = [flat_propo[index] for index in sorted(indexes)]
        propositions.sort()
        self.propositions = propositions
        props = propos()
        for i in range(np.size(propositions)):
            propName = propositions[i]
            exec('props.'+propName+' = 0')

        self.props = props

        controllableProp = [re.findall('pred', elem) for elem in self.propositions]
        for i in range(np.size(controllableProp)):
            if len(controllableProp[i]) != 0:
                self.controllableProp.append(self.propositions[i])
        # Find number of inputs
        self.N = np.size(self.propositions) - np.size(self.controllableProp)
class propos:
    pass

class StatesOfI:
    def __init__(self):
        self.cond = [] # Conditions in a transition
        self.result = [] # result of transitions
        self.condCNF = [] # Simplified conditions

class specInfo:
    def __init__(self,specattr,spec,props,propositions,phi,controllableProp,M,text1,master,map,nodes,nodeGraph,nodeConnections):
        self.graph = [] # Transition graph
        self.State = [] # States in the system
        self.accepting_states = specattr.accepting_states # Set of accepting states
        self.props = [] # proposition values
        self.controllableProp = controllableProp # Set of controllable propositions
        self.uncontrollableProp = [] # Set of uncontrollable Propositions
        self.spec = spec # Full Specification
        self.phi = phi # Parsed spec
        self.controllablePropOrder = [] # Order of controllable propositions used to ensure things do not get mixed
        self.parameters = [] # Parameters of interest (fucntions to evaluate)
        self.M = M # Number of robots
        self.nodeGraph = [] # Graph of nodes for map
        self.nodes = nodes # nodes in map
        self.map = map # map
        self.nodeGraph = nodeGraph
        self.nodeConnections = nodeConnections
        self.beginSpec(spec,props,propositions,text1,master)


    def beginSpec(self,spec,props,simpleProp,text1,master):
        # Find the number of States
        numStates = int(re.split(' ',spec[0])[-1])

        # Find the location of all states
        states = [re.findall('(State\:)', elem) for elem in spec]
        idx = [i for i, x in enumerate(states) if len(x) > 0]
        idx.append(len(states)-1)

        # Translate numbers to AP
        AP = [re.findall('(AP\: )', elem) for elem in spec]
        idxAP = [i for i, x in enumerate(AP) if len(x) > 0]
        allAP = spec[idxAP[0]]
        propositions = re.findall('"([^"]*)"', allAP)

        # Go through all States and find the conditions of transitions, the results of the conditions,
        # and the accepting states

        State = []
        for i in range(numStates):
            State.append(StatesOfI())
            condRef = spec[idx[i]+1:idx[i+1]]
            for j in range(len(condRef)):
                condTemp = re.split('] ',condRef[j])
                if condTemp[0] != 't':
                    for k in range(np.size(propositions)):
                        condTemp[0] = re.sub(' '+str(k)+' ',' ('+propositions[k]+') ',condTemp[0])
                        condTemp[0] = re.sub(' '+str(k)+'$',' ('+propositions[k]+') ',condTemp[0])

                    condTemp[0] = re.sub(' or ',' ) or ( ',condTemp[0])
                    condTemp[0] = '(' + condTemp[0] + ')'
                    State[i].cond.append(condTemp[0])
                else:
                    State[i].cond.append('All')
                State[i].result.append(int(condTemp[1]))
        # toCo = copy.deepcopy(State)
        # State = []
        # for i in range(numStates):
        #     State.append(StatesOfI())
        #     condRef = spec[idx[i]+1:idx[i+1]]
        #     for j in range(len(condRef)):
        #         condTemp = re.split('] ',condRef[j])
        #         condEdit = re.sub('\[','',condTemp[0])
        #         if condEdit != 't' and condEdit != ' t':
        #             condEditSplit = re.split('(?<=[.dr])',condEdit)
        #             for k in range(np.size(condEditSplit)):
        #                 try:
        #                     wholeNum = re.findall('[0-9]+', condEditSplit[k])[0]
        #                     condEditSplit[k] = re.sub(wholeNum,'('+propositions[int(wholeNum)]+')',condEditSplit[k])
        #                 except:
        #                     pass
        #             condFinal = ''.join(condEditSplit)
        #             condSplit2 = re.split('or',condFinal)
        #             for k in range(np.size(condSplit2)):
        #                 condSplit2[k] = '('+condSplit2[k]+')'
        #             condFinal2 = ' | '.join(condSplit2)
        #             condFinal2 = re.sub('&', ' and ',condFinal2)
        #             condFinal2 = re.sub('\|', 'or', condFinal2)
        #             condFinal2 = re.sub('!','not ',condFinal2)
        #             State[i].cond.append(condFinal2)
        #         else:
        #             State[i].cond.append('All')
        #         State[i].result.append(int(condTemp[1]))


        self.State = State
        self.accepting_states = np.array(self.accepting_states,dtype='int')



        # find intial value of proposiion and set that
        if np.size(State[0].cond) == 1 and np.size(State[0].result) == 1:
            initVal = re.split('&&',State[0].cond[0])
            for i in range(np.size(initVal)):
                bool = re.search('not', initVal[i])
                propName = simpleProp[i]
                if bool is not None:
                    exec('props.' + propName + ' = 0')
                else:
                    exec('props.' + propName + ' = 1')

        self.props = props

        graph = np.zeros((np.size(State),np.size(State)))
        for i in range(np.size(State)):
            for j in range(np.size(State[i].result)):
                graph[i, int(State[i].result[j])] = 1

        self.graph = graph

        # Find accepting states with a cycle
        acceptingWithCycle = []
        for j in range(len(self.accepting_states)):
            if self.graph[self.accepting_states[j], self.accepting_states[j]] != 1:
                (cost, rute) = matrixDijkstra.dijkstra(self.graph, self.accepting_states[j], self.accepting_states[j])
            else:
                cost = 1
                rute = np.array([self.accepting_states[j], self.accepting_states[j]])
            costRef = np.size(rute, 0) - 1
            if not cost > 10000 or costRef == cost:
                if cost == 1:
                    acceptingWithCycle.append(self.accepting_states[j])

        self.acceptingWithCycle = acceptingWithCycle

        # Find all routes
        percentages = list(np.linspace(0, 100, np.size(self.graph, 0) + 1))
        percentages = percentages[1:]
        nRoutes = [[]] * len(self.graph)
        for i in range(np.size(self.graph,0)):
            tempRoute = []
            for j in range(np.size(self.acceptingWithCycle,0)):
                goTo = self.acceptingWithCycle[j]
                try:
                    allPaths = self.findNRoutes(i,goTo)
                    tempRoute.append(allPaths)
                except:
                    pass
            nRoutes[i] = tempRoute

            percentage = round(percentages[i], 2)
            text1.configure(state='normal')
            text1.delete("end-1l", "end")
            text1.configure(state='disabled')
            complete_status = str(percentage) + '% complete'
            message = '\nFinding Routes through Buchi ' + complete_status
            runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData, text1, master, message)

        self.nRoutes = nRoutes
        # Because of the naming convention, the o
        for i in range(np.size(self.controllableProp)):
            propName = self.controllableProp[i]
            # find the phi that matches this prop
            for j in range(np.size(self.phi)):
                isThere = re.findall(propName, self.phi[j].prop_label)
                if isThere:
                    break
            if isThere:
                self.controllablePropOrder.append(j)

        # Create a set of the parameters in the order that they appear
        # controllablePropOrder = np.unique(self.controllablePropOrder)
        for i in range(np.size(self.controllablePropOrder)):
            self.parameters.append(self.phi[self.controllablePropOrder[i]].params)

        controllableProp = np.array(self.controllableProp)
        self.uncontrollableProp = np.setdiff1d(simpleProp, controllableProp).tolist()



        # Create a nodegraph using the nodes and map
        if len(self.nodeGraph) == 0:
            nodeGraph = np.zeros((np.size(self.nodes, 0), np.size(self.nodes, 0)))

            # Track progress
            percentages = list(np.linspace(0, 100,np.size(self.nodes, 0)+1))
            percentages = percentages[1:]



            for i in range(np.size(self.nodes, 0)):
                for j in range(np.size(self.nodes, 0)):
                    if i != j:
                        isect = self.intersectPoint(self.nodes[i, 0], self.nodes[i, 1], self.nodes[j, 0], self.nodes[j, 1],
                                                    self.map[:, 0], self.map[:, 1], self.map[:, 2],
                                                    self.map[:, 3])

                        if not np.any(isect):
                            pt1 = self.nodes[i,:]
                            pt2 = self.nodes[j,:]
                            dist2closest1 = self.distWall(pt1, pt2, np.vstack((self.map[:, 0:2], self.map[:, 2:4])))
                            if min(dist2closest1) > .07:
                                nodeGraph[i, j] = np.sqrt(
                                    (self.nodes[i, 0] - self.nodes[j, 0]) ** 2 + (self.nodes[i, 1] - self.nodes[j, 1]) ** 2)

                # Track Progress
                percentage = round(percentages[i],2)
                text1.configure(state='normal')
                text1.delete("end-1l", "end")
                text1.configure(state='disabled')
                complete_status = str(percentage) + '% complete'
                if percentage != 100:
                    message = '\nCreating Roadmap. ' + complete_status
                else:
                    message = '\nCreating Roadmap. ' + complete_status + '\n'
                runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData,text1,master,message)

            self.nodeGraph = nodeGraph
            my_dir = os.path.dirname(os.path.abspath(__file__))
            pickle_file_path = os.path.join(my_dir, 'Maps', 'NRINodeGraph.pkl')
            with open(pickle_file_path, 'wb') as output:
                pickle.dump(nodeGraph, output, pickle.DEFAULT_PROTOCOL)

        if len(self.nodeConnections) == 0:
            # Track progress
            percentages = list(np.linspace(0, 100,np.size(self.nodes, 0)+1))
            percentages = percentages[1:]
            nodeConnections = [[]] * np.size(self.nodes,0)

            for i in range(np.size(self.nodes,0)):
                tempConn = []
                for j in range(np.size(self.nodes,0)):
                    if i != j:
                        try:
                            if j != 0:
                                (cost, rute) = matrixDijkstra.dijkstra(nodeGraph, i, j)
                                rute = np.flip(rute)
                            else:
                                (cost, rute) = matrixDijkstra.dijkstra(nodeGraph, j, i)
                        except:
                            print('here')
                    else:
                        rute = np.array([i,j])
                        cost = 0
                    rute = rute.tolist()
                    rute.append(cost)
                    tempConn.append(rute)
                nodeConnections[i] = tempConn

                # Track Progress
                percentage = round(percentages[i],2)
                text1.configure(state='normal')
                text1.delete("end-1l", "end")
                text1.configure(state='disabled')
                complete_status = str(percentage) + '% complete'
                if percentage != 100:
                    message = '\nFinding routes. ' + complete_status
                else:
                    message = '\nFinding routes. ' + complete_status + '\n'
                runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData,text1,master,message)

            self.nodeConnections = nodeConnections
            my_dir = os.path.dirname(os.path.abspath(__file__))
            pickle_file_path = os.path.join(my_dir, 'Maps', 'NRINodeConnections.pkl')
            with open(pickle_file_path, 'wb') as output:
                pickle.dump(nodeConnections, output, pickle.DEFAULT_PROTOCOL)






    # def intersectPoint(self, x1, y1, x2, y2, x3, y3, x4, y4):
    #     with np.errstate(divide='ignore', invalid='ignore'):
    #         ua = np.divide(((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)), ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)))
    #         ub = np.divide(((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)), ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)))
    #
    #         isect = (ua>=0)*(ub>=0)*(ua<=1)*(ub<=1)
    #         return isect

    def distWall(self, p1, p2, pt):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        t = ((pt[:, 0] - p1[0]) * dx + (pt[:, 1] - p1[1]) * dy) / (dx ** 2 + dy ** 2)

        if dx == 0 and dy == 0:
            closestP = p1
            dx = pt[0] - p1[0]
            dy = pt[1] - p1[1]
            dist = np.zeros((1, np.size(pt, 0)))[0]
            # dist = np.sqrt(dx ** 2 + dy ** 2) * np.ones((1,np.size(pt,0)))[0]
        else:
            dist = np.zeros((1, np.size(pt, 0)))[0]

        try:
            indL = np.where(t < 0)[0]
            dx = pt[indL, 0] - p1[0]
            dy = pt[indL, 1] - p1[1]
            dist[indL] = np.sqrt(dx ** 2 + dy ** 2)
        except:
            pass

        try:
            indG = np.where(t > 0)[0]
            dx = pt[indG, 0] - p1[0]
            dy = pt[indG, 1] - p1[1]
            dist[indG] = np.sqrt(dx ** 2 + dy ** 2)
        except:
            pass

        try:
            indM = np.where((t >= 0) & (t <= 1))[0]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            closestP = np.array([p1[0] + t[indM] * dx, p1[1] + t[indM] * dy])
            dx = pt[indM, 0] - closestP[0]
            dy = pt[indM, 1] - closestP[1]
            dist[indM] = np.sqrt(dx ** 2 + dy ** 2)
        except:
            pass

        return dist

    def findNRoutes(self, startState, endState):
        numP = 5
        G = nx.DiGraph()
        G.add_nodes_from(range(0, np.size(self.graph, 0)))
        for jj in range(np.size(self.graph, 0)):
            for ii in range(np.size(self.graph, 0)):
                if self.graph[ii, jj] == 1:
                    G.add_edge(ii, jj)
        allPaths = []
        if startState != endState:
            for path in self.k_shortest_paths(G, startState, endState, numP):
                allPaths.append(path)
        else:
            if self.graph[startState,endState] == 1:
                allPaths.append([startState,endState])
            else:
                (cost, rute) = matrixDijkstra.dijkstra(self.graph, startState, endState)
                allPaths.append(rute.astype('int').tolist())

        return allPaths

    def k_shortest_paths(self,G, source, target, k, weight=None):
        return list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))