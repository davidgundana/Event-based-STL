import numpy as np
from itertools import combinations
import runEvBasedSTL

class Props:
    def __init__(self):
        self.props = []
        self.id = []

class check:
    def __init__(self,State, M,text1,master):
        self.State = State.State
        self.controllableProp = State.controllableProp
        self.M = M
        self.phi = State.phi
        self.allConflicts = []
        self.isconflict = 0
        self.okConflict = 0
        self.checkConflicts(text1,master)

    def checkConflicts(self,text1,master):
        allEdge = []
        for i in range(np.size(self.State)):
            for j in range(np.size(self.State[i].condCNF,0)):
                #allEdgeTemp = np.array(self.State[i].condCNF[j]).reshape(-1,1)
                allEdgeTemp = np.array(self.State[i].condCNF[j][0])
                #allEdgeTemp = allEdgeTemp[allEdgeTemp != '0'].reshape(-1,1)
                allEdge.append(allEdgeTemp)

        allEdge = np.concatenate(allEdge)
        allEdge = np.unique(allEdge, axis=0)

        allEdgeNew = np.zeros((1,np.size(np.fromstring(allEdge[0][0], dtype=int, sep=' '))))

        for i in range(np.size(allEdge)):
            tempStr = allEdge[i][0]
            tempStr = np.fromstring(tempStr, dtype=int, sep=' ')
            allEdgeNew = np.append(allEdgeNew,[tempStr],axis=0)

        allEdgeNew = allEdgeNew[1:,:]
        lengthCont = np.size(np.fromstring(allEdge[0][0], dtype=int, sep=' '))
        lengthUnc = lengthCont - np.size(self.controllableProp)
        allEdgeNew1 = allEdgeNew[:,lengthUnc:lengthCont]
        allEdgeFinal = np.unique(allEdgeNew1, axis=0)

        locOfTrue = np.zeros(allEdgeFinal.shape)
        for i in range(np.size(allEdgeFinal, 0)):
            ind = np.where(allEdgeFinal[i,:] == 1)
            locOfTrue[i, 0:np.size(ind)] = ind[0]

        propsAffect = []
        for i in range(1,int(self.M)+1):
            propsAffect.append(Props())
            for j in range(np.size(self.phi)):
                ind = np.where(self.phi[j].robotsInvolved == i)
                if not ind[0].size == 0:
                    propsAffect[i-1].props.append([p for p, s in enumerate(self.controllableProp) if self.phi[j].prop_label in s][0])
                    propsAffect[i-1].id.append(self.phi[j].id)

        allConflicts = np.empty((1,2),dtype=str)
        allConflictsOk = np.empty((1,2),dtype=str)
        for i in range(int(self.M)):
            conflictProps = np.empty((1,2))
            for j in range(np.size(locOfTrue, 0)):
                numCon = list(set(propsAffect[i].props) & set(locOfTrue[j,:].tolist()))
                if np.size(numCon) == 2:
                    conflictProps = np.vstack((conflictProps[0], numCon))
                elif np.size(numCon) > 2:
                    possibleCom = np.array(list(combinations(numCon,2)))
                    conflictProps = np.append(conflictProps, possibleCom, axis = 0)

            conflictProps = conflictProps[1:,:]
            if conflictProps.size !=0:
                conflictProps = np.unique(conflictProps, axis = 0)
            for j in range(np.size(conflictProps, 0)):
                #first find the phi that is associated with each prop number
                intersectVal = list(set(propsAffect[i].props) & set(conflictProps[j,:].tolist()))
                ia = [propsAffect[i].props.index(s) for s in intersectVal]
                phi2Consider = [propsAffect[i].id[s] for s in ia]
                signPhi = np.append(self.phi[phi2Consider[0]].signFS[0], self.phi[phi2Consider[1]].signFS[0])
                signSum = np.sum(signPhi)
                if signSum == -2:
                    interse = np.linalg.norm(self.phi[phi2Consider[0]].point - self.phi[phi2Consider[1]].point) - self.phi[phi2Consider[0]].p - self.phi[phi2Consider[1]].p
                    if interse > 0:
                        func1 = str(self.phi[phi2Consider[0]].prop_label)
                        func2 = str(self.phi[phi2Consider[1]].prop_label)
                        conflicts = np.empty((1, 2), dtype="<U20")
                        conflicts[0,0] = func1
                        conflicts[0,1] = func2
                        allConflicts = np.append(allConflicts,conflicts,axis = 0)
                        self.isconflict = 1
                    else:
                        func1 = str(self.phi[phi2Consider[0]].prop_label)
                        func2 = str(self.phi[phi2Consider[1]].prop_label)
                        conflictsOk = np.empty((1, 2), dtype="<U20")
                        conflictsOk[0, 0] = func1
                        conflictsOk[0, 1] = func2
                        allConflictsOk = np.append(allConflictsOk, conflictsOk, axis=0)
                        self.okConflict = 1

        allConflicts = allConflicts[1:,:]
        allConflictsOk = allConflictsOk[1:, :]
        self.allConflicts = allConflicts
        if self.isconflict == 1:
            runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData, text1, master, '\nThe following propositions may conflict:')
            print('The following propositions do not intersect and may conflict: ')
            for i in range(np.size(self.allConflicts,0)):
                for j in range(np.size(self.phi)):
                    if self.allConflicts[i,0] == self.phi[j].prop_label:
                        conflict1 = str(j)
                        break
                for  j in range(np.size(self.phi)):
                    if self.allConflicts[i,1] == self.phi[j].prop_label:
                        conflict2 = str(j)
                        break
                mess1 = '\n' +  'Event-based STL formula ' + conflict1 + ' and ' + conflict2
                mess2 = 'Event-based STL formula ' + conflict1 + ' and ' + conflict2
                runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData, text1, master, mess1)
                print(mess2)

        if self.okConflict == 1:
            RunSTL.formData.updateStatus(RunSTL.formData, text1, master, '\nThe following propositions may conflict, but \nthey are intersecting so a solution can be found:')
            print('The following propositions may conflict: ')
            for i in range(np.size(allConflictsOk, 0)):
                for j in range(np.size(self.phi)):
                    if self.allConflicts[i,0] == self.phi[j].prop_label:
                        conflict1 = str(j)
                        break
                for  j in range(np.size(self.phi)):
                    if self.allConflicts[i,1] == self.phi[j].prop_label:
                        conflict2 = str(j)
                        break
                mess1 = '\n' + 'Event-based STL formula ' + conflict1 + ' and ' + conflict2
                mess2 = 'Event-based STL formula ' + conflict1 + ' and ' + conflict2
                runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData, text1, master, mess1)
                print(mess2)