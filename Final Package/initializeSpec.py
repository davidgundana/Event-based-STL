import numpy as np
import re
import runEvBasedSTL
import pickle
import matrixDijkstra
import networkx as nx
import os
from itertools import islice

class spec:
    def __init__(self,spec):
        self.spec = spec # specification
        self.accepting_states = [] # pre-allocate accepting states
        self.controllableProp = [] # pre-allocate controllable propositions
        self.props = [] # value of propositions
        self.initSpec()


    def initSpec(self):
        # Replace the exclamation points with readable values for python and create an object with the
        # values of all propositions
        self.spec = [re.sub("!", "not ", elem) for elem in self.spec]

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


class propos:
    pass

class StatesOfI:
    def __init__(self):
        self.cond = [] # Conditions in a transition
        self.result = [] # result of transitions
        self.condCNF = [] # Simplified conditions

class specInfo:
    def __init__(self,spec,props,propositions,phi,controllableProp,M,text1,master,map,nodes,nodeGraph,nodeConnections):
        self.graph = [] # Transition graph
        self.State = [] # States in the system
        self.accepting_states = [] # Set of accepting states
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


    def beginSpec(self,spec,props,propositions,text1,master):
        # Find all states
        states = [re.findall('(\_S+\d+\:)', elem) for elem in spec]
        if 'all' in ' '.join(spec):
            # find highest state in spec
            highestState = re.findall('(\_S+\d+\:)', ' '.join(spec))
            strHighest = [re.search('[+-]?\d+\.?\d*', s)[0] for s in highestState]
            intHighest = max(list(map(int, strHighest))) + 1
            spec = [re.sub('_all', '_S' + str(intHighest), line) for line in spec]
            states = [re.findall('(\_S+\d+\:)', elem) for elem in spec]

        init = [re.findall('init\:',elem) for elem in spec]
        idxi = [i for i, x in enumerate(init) if len(x) > 0]
        idxs = [i for i, x in enumerate(states) if len(x) > 0]
        idx = idxi + idxs

        labelOfS = [spec[i] for i in idx]
        State = []

        # Track progress
        percentages = list(np.linspace(0, 100, np.size(idx)+1))
        percentages = percentages[1:]

        # Go through all States and find the conditions of transitions, the results of the conditions,
        # and the accepting states
        for i in range(np.size(idx)):
            State.append(StatesOfI())
            if i + 2 > np.size(idx):
                condition = spec[idx[i] + 2:-2]
                if np.size(condition) == 0:
                    try:
                        condition = spec[idx[i] + 2:-2][0]
                    except:
                        condition = spec[idx[i] + 2:-1][0]
            else:
                longRange = range(idx[i] + 2,idx[i + 1] - 2)
                lengthOfR = longRange.stop - longRange.start
                if lengthOfR >= 1:
                    condition = spec[idx[i] + 2:idx[i + 1] - 1]
                else:
                    condition = spec[idx[i] + 2]

            if np.size(condition) > 1:
                splitcond = [re.split('\->+', elem) for elem in condition]
            else:
                try:
                    splitcond = re.split('\->+', condition)
                except:
                    splitcond = re.split('\->+', condition[0])

            for j in range(np.size(condition)):
                if np.size(condition) > 1:
                    splitcondred = re.split(':+\s', splitcond[j][0], maxsplit=1)
                    try:
                        ref = re.search('S+\d*', splitcond[j][1])[0]
                        ref = ref + ':'
                    except:
                        ref = re.search('init', splitcond[j][1])[0]
                        ref = ref + ':'
                    splitcondred = re.sub("\&\&", "and", splitcondred[1])
                    splitcondred = re.sub("\|\|", "or", splitcondred)
                    State[i].cond.append(splitcondred)
                    location = [re.findall(ref, elem) for elem in labelOfS]

                    accepting = re.search('accept', splitcond[j][1])
                    results = [k for k, x in enumerate(location) if len(x) > 0]
                else:
                    splitcondred = re.split(':+\s', splitcond[j], maxsplit=1)
                    try:
                        ref = re.search('S+\d*', splitcond[1])[0]
                        ref = ref + ':'
                    except:
                        ref = re.search('init', splitcond[1])[0]
                        ref = ref + ':'
                    splitcondred = re.sub("\&\&", "and", splitcondred[1])
                    splitcondred = re.sub("\|\|", "or", splitcondred)
                    State[i].cond.append(splitcondred)
                    location = [re.findall(ref, elem) for elem in labelOfS]

                    accepting = re.search('accept', splitcond[1])
                    results = [k for k, x in enumerate(location) if len(x) > 0]

                State[i].result.append(results[0])
                if accepting is not None:
                    self.accepting_states.append(State[i].result[j])

            # Track Progress
            percentage = round(percentages[i],2)
            text1.configure(state='normal')
            text1.delete("end-1l", "end")
            text1.configure(state='disabled')
            complete_status = str(percentage) + '% complete'
            message = '\nPreparing specification. ' + complete_status
            runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData,text1,master,message)

        # Object for each State
        self.State = State
        self.accepting_states = np.array(self.accepting_states)
        # Set of accepting States
        self.accepting_states = np.unique(self.accepting_states)


        # find intial value of proposiion and set that
        if np.size(State[0].cond) == 1 and np.size(State[0].result) == 1:
            initVal = re.split('&&',State[0].cond[0])
            for i in range(np.size(initVal)):
                bool = re.search('not', initVal[i])
                propName = propositions[i]
                if bool is not None:
                    exec('props.' + propName + ' = 0')
                else:
                    exec('props.' + propName + ' = 1')

        self.props = props

        graph = np.zeros((np.size(State),np.size(State)))
        for i in range(np.size(State)):
            for j in range(np.size(State[i].result)):
                graph[i, State[i].result[j]] = 1

        self.graph = graph
        nRoutes = [[]] * len(self.graph)
        for i in range(np.size(self.graph,0)):
            tempRoute = []
            for j in range(np.size(self.graph,1)):
                allPaths = self.findNRoutes(i,j)
                tempRoute.append(allPaths)
            nRoutes[i] = tempRoute

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
        controllablePropOrder = np.unique(self.controllablePropOrder)
        for i in range(np.size(controllablePropOrder)):
            self.parameters.append(self.phi[controllablePropOrder[i]].params)

        controllableProp = np.array(self.controllableProp)
        self.uncontrollableProp = np.setdiff1d(propositions, controllableProp).tolist()


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
        # Create a nodegraph using the nodes and map
        if len(self.nodeGraph) == 0:
            nodeGraph = np.zeros((np.size(self.nodes, 0), np.size(self.nodes, 0)))

            # Track progress
            percentages = list(np.linspace(0, 100,np.size(self.nodes, 0)+1))
            percentages = percentages[1:]

            for i in range(np.size(self.nodes, 0)):
                for j in range(np.size(self.nodes, 0)):
                    if i != j:
                        isect = []
                        for k in range(np.size(self.map, 0)):
                            isecttemp = self.intersectPoint(self.nodes[i, 0], self.nodes[i, 1], self.nodes[j, 0], self.nodes[j, 1],
                                                            self.map[k, 0], self.map[k, 1], self.map[k, 2], self.map[k, 3])
                            if isecttemp == 1:
                                isect.append(isecttemp)

                        if len(isect) == 0:
                            pt1 = self.nodes[i,:]
                            pt2 = self.nodes[j,:]
                            ptOfI1 = self.map[:, 0:2]
                            ptOfI2 = self.map[:, 2:4]
                            dist2closest1 = self.distWall(pt1, pt2, ptOfI1)
                            dist2closest2 = self.distWall(pt1, pt2, ptOfI2)
                            if min(dist2closest1) > .5 and min(dist2closest2) > .5:
                                nodeGraph[i, j] = np.sqrt(
                                    (self.nodes[i, 0] - self.nodes[j, 0]) ** 2 + (self.nodes[i, 1] - self.nodes[j, 1]) ** 2)

                # Track Progress
                percentage = round(percentages[i],2)
                text1.configure(state='normal')
                text1.delete("end-1l", "end")
                text1.configure(state='disabled')
                complete_status = str(percentage) + '% complete'
                message = '\nCreating Roadmap. ' + complete_status
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
                            (cost, rute) = matrixDijkstra.dijkstra(nodeGraph, i, j)
                            rute = np.flip(rute)
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
                message = '\nFinding routes. ' + complete_status
                runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData,text1,master,message)

            self.nodeConnections = nodeConnections
            my_dir = os.path.dirname(os.path.abspath(__file__))
            pickle_file_path = os.path.join(my_dir, 'Maps', 'NRINodeConnections.pkl')
            with open(pickle_file_path, 'wb') as output:
                pickle.dump(nodeConnections, output, pickle.DEFAULT_PROTOCOL)






    def intersectPoint(self,x1,y1,x2,y2,x3,y3,x4,y4):
        denom = (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1)
        if denom == 0:
            isect = 0
        else:
            ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3))/denom
            ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3))/denom

            if ua >= 0 and ub >= 0 and ua <=1 and ub <=1:
                isect = 1
            else:
                isect = 0
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

    def findNRoutes(self, startState, endState):
        numP = 10
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