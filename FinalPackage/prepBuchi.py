import numpy as np
import re
import runEvBasedSTL
import tkinter as tk
import itertools

class prep:
    def __init__(self,State,propositions,text1,master):
        self.State = State.State # buchi automaton
        self.accepting_states = State.accepting_states # set of accepting states
        self.controllableProp = State.controllableProp # set of controllable propositions
        self.controllablePropOrder = State.controllablePropOrder # Order of the controllable propositions
        self.parameters = State.parameters # parameters for all formulas
        self.graph = State.graph # Graph from nodes to nodes
        self.nRoutes = State.nRoutes
        self.acceptingWithCycle = State.acceptingWithCycle
        self.nodeConnections = State.nodeConnections
        self.phi = State.phi # Event-based STL formula
        self.props = State.props # Propositions values
        self.spec = State.spec # specification
        self.uncontrollableProp = State.uncontrollableProp # set of uncontrollable propositions
        self.propositions = propositions # Propositions to be evaluated
        self.nodeGraph = State.nodeGraph #node graph for map
        self.nodes = State.nodes # nodes for map
        self.map = State.map #map of environment
        self.prepSpecification(text1,master)

    def prepSpecification(self,text1,master):
        # Track status
        percentages = list(np.linspace(0, 100, np.size(self.State)+1))
        percentages = percentages[1:]
        reference = 0

        # Transform complex conditions for a transition to a set of  values (0,1,2). This will help with
        # evaluating at runtime
        for i in range(np.size(self.State)):
            condCNF = []
            for q in range(np.size(self.State[i].cond)):
                column = []
                for r in range(np.size(self.propositions)):
                    column.append(0)
                condCNF.append(column)
            self.State[i].condCNF = condCNF

            for j in range(np.size(self.State[i].cond)):
                allRCond = np.zeros((1,np.size(self.propositions)),dtype = int)
                optionsForTrans = re.split('or', self.State[i].cond[j])
                for k in range(np.size(optionsForTrans)):
                    thisOption = re.split('and', optionsForTrans[k])
                    valueOfProp = [re.search('not', elem) for elem in thisOption]

                    CNFrep = []
                    for p in range(np.size(valueOfProp)):
                        if valueOfProp[p] is None:
                            CNFrep.append(1)
                        else:
                            CNFrep.append(0)

                    refNum = []
                    for l in range(np.size(self.propositions)):
                        propo = self.propositions[l]
                        propo = '\(props.' + propo + '\)'
                        locOfOption = [re.findall(propo, element) for element in thisOption]
                        locOfOptionTemp = locOfOption
                        for elem in locOfOptionTemp:
                            if elem:
                                refNum.append(l)
                                break

                    finalPropVals = [np.inf] * np.size(self.propositions)
                    for l in range(np.size(self.uncontrollableProp)):
                        finalPropVals[l] = 2

                    for l in range(np.size(refNum)):
                        finalPropVals[refNum[l]] = CNFrep[l]

                    temporaryR = np.asarray([finalPropVals])
                    locOfAll = [i for i, x in enumerate(finalPropVals) if x == 2]

                    if len(locOfAll) != 0:
                        lst = list(map(list, itertools.product([0, 1], repeat=len(locOfAll))))
                        allR = np.repeat(temporaryR, len(lst), axis=0)
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

                self.State[i].condCNF[j] = allRCond


            # Update status of preparation
            percentage = round(percentages[i],2)
            text1.configure(state='normal')
            text1.delete("end-1l", "end")
            text1.configure(state='disabled')
            complete_status = str(percentage) + '% complete'
            message = '\nPreparing Buchi. ' + complete_status
            runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData,text1,master,message)


        percentage = 100
        text1.configure(state='normal')
        text1.delete("end-1l", "end")
        text1.configure(state='disabled')
        complete_status = str(percentage) + '% complete'
        message = '\nPreparing Buchi. ' + complete_status
        runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData, text1, master, message)
