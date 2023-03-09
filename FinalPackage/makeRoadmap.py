import pickle
import numpy as np
import helperFuncs
import runEvBasedSTL
import os
import re
import matrixDijkstra
import time


class mapInfo():
    def __init__(self,filenames,text1,master):
        self.nodeGraph = []
        self.nodeConnections = []
        self.nodes = []
        self.map = []
        self.createMap(filenames,text1,master)

    def createMap(self,filenames,text1,master):
        avoidWallsInPath = 1
        wallDistance = .03
        map = np.loadtxt(filenames[2])
        self.map = map
        nodes = np.loadtxt(filenames[3])
        self.nodes = nodes

        try:
            with open(filenames[4], 'rb') as input:
                nodeGraph = pickle.load(input)
        except:
            nodeGraph = []

        try:
            with open(filenames[5], 'rb') as input:
                nodeConnections = pickle.load(input)
        except:
            nodeConnections = []

        # Create a nodegraph using the nodes and map
        if len(nodeGraph) == 0:
            t1 = time.time()
            nodeGraph = np.zeros((np.size(nodes, 0), np.size(nodes, 0)))

            # Track progress
            percentages = list(np.linspace(0, 100,np.size(nodes, 0)+1))
            percentages = percentages[1:]

            for i in range(np.size(nodes, 0)):
                isect = helperFuncs.intersectPointVec(nodes[i, 0], nodes[i, 1], nodes[:, 0], nodes[:, 1], map[:, 0],
                                                      map[:, 1], map[:, 2], map[:, 3])
                for j in range(np.size(isect)):
                    if avoidWallsInPath:
                        dist2walls, closestP = helperFuncs.distWall(nodes[i,:],nodes[isect[j]], np.vstack((map[:,0:2],map[:,2:4])))
                        if min(dist2walls) > wallDistance:
                            nodeGraph[i, isect[j]] = np.sqrt(
                                (nodes[i, 0] - nodes[isect[j], 0]) ** 2 + (nodes[i, 1] - nodes[isect[j], 1]) ** 2)

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
                # runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData,text1,master,message)

            print('Total time to create roadmap was {} seconds'.format(time.time()-t1))

            my_dir = os.path.dirname(os.path.abspath(__file__))
            mapName = os.path.split(filenames[2])[-1]
            mapNameSave = re.split('\.',mapName)[0] + 'Nodes.pkl'
            pickle_file_path = os.path.join(my_dir, 'Maps', mapNameSave)
            with open(pickle_file_path, 'wb') as output:
                pickle.dump(nodeGraph, output, pickle.DEFAULT_PROTOCOL)

        self.nodeGraph = nodeGraph

        if len(nodeConnections) == 0:
            t1 = time.time()
            # Track progress
            percentages = list(np.linspace(0, 100,np.size(nodes, 0)+1))
            percentages = percentages[1:]
            nodeConnections = [[]] * np.size(nodes,0)

            for i in range(np.size(nodes,0)):
                tempConn = []
                for j in range(np.size(nodes,0)):
                    if i != j:
                        if j != 0:
                            (cost, rute) = matrixDijkstra.dijkstra(nodeGraph, i, j)
                            rute = np.flip(rute)
                        else:
                            (cost, rute) = matrixDijkstra.dijkstra(nodeGraph, j, i)
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
                    message = '\nFinding routes in roadmap. ' + complete_status
                else:
                    message = '\nFinding routes in roadmap. ' + complete_status + '\n'

                # runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData,text1,master,message)

            print('Total time to create node connections in roadmap was {} seconds'.format(time.time()-t1))

            my_dir = os.path.dirname(os.path.abspath(__file__))
            mapName = os.path.split(filenames[2])[-1]
            mapNameSave = re.split('\.',mapName)[0] + 'NodeConnections.pkl'
            pickle_file_path = os.path.join(my_dir, 'Maps', mapNameSave)
            with open(pickle_file_path, 'wb') as output:
                pickle.dump(nodeConnections, output, pickle.DEFAULT_PROTOCOL)

        self.nodeConnections = nodeConnections


