import numpy as np
import re
import copy


class Parsed:
    def __init__(self, absPred, id, M, init_pos, init_posRef):
        self.interval = np.array([0, np.inf]) # Interval for formula
        self.tprime = [] # Time formula is activated
        self.implies = 0 # Toggle for implication
        self.inputTime = []
        self.impliesmessage = '' # Name of implication message defined by user
        self.type = '' # Type of Event-based STL formula
        self.initDist = [] # Initial distance
        self.minVel = [] # min velocity
        self.prop_label = absPred[1]  # Label for proposition created
        self.nom = np.zeros([2, 3], dtype='float') # Nominal controller
        self.p = [] # Size of safe set
        self.point = []  #Reference goal point if it exists
        self.funcOf = [] # Function to Evaluate
        self.signFS = [] # direction of inequality
        self.robotsInvolved = [] # Index of robots involved
        self.M = M #Number of robots in the system
        self.init_pos = init_pos # Initial Position of robots
        self.init_posRef = init_posRef #Initial Robot posisiton
        self.dir = [] #Direction used for nominal controller
        self.phiUntil = absPred[4]
        self.until = 0
        self.robustness = None
        self.STLParse(absPred, id)

    def STLParse(self, absPred, id):
        # id of abstracted predicate
        self.id = id
        self.type = re.search(r'(alw|ev|un)', absPred[2])[0]
        if self.type == 'un' and self.phiUntil == '':
            self.type = 'ev'
            self.until = 1
        elif self.type == 'un' and self.phiUntil != '':
            self.type = 'alw'
            self.until = 1
        # interval or abstracted predicate
        inte = re.findall(r"[^[]*\[([^]]*)\]", absPred[2])[0]
        self.interval = np.asarray(inte.split(','), dtype='float')
        self.tprime = self.interval[1]

        self.prop_label = re.split('\.', self.prop_label)[-1]

        # save the predicate of the abstraction
        parameter = re.sub('\[', '(', absPred[0])
        parameter = re.sub('\]', ')', parameter)

        for j in range(3 * int(self.M)):
            oldPos = "pos\(" + str(j) + "\)"
            newPos = "pos[" + str(j) + "]"
            parameter = re.sub(oldPos, newPos, parameter)

            oldPosRef = "posRef\(" + str(j) + "\)"
            newPosRef = "posRef[" + str(j) + "]"
            parameter = re.sub(oldPosRef, newPosRef, parameter)

            oldWallRef = "wall\(" + str(j) + "\)"
            newWallRef = "wall[" + str(j) + "]"
            parameter = re.sub(oldWallRef, newWallRef, parameter)
        parameter = re.sub("sqrt", "np.sqrt", parameter)
        parameter = re.sub("abs", "np.abs", parameter)
        parameter = re.sub('\^', '**', parameter)

        self.params = parameter

        # Check if theres implication and get the formula
        if absPred[3] != '' and len(absPred[3]) > 0:
            self.implies = 1
            self.impliesmessage = ' & '.join(absPred[3])

        # Determine the direction of the inequality and the distance from safe set
        pref = []
        signTemp = re.findall(r'(<|>)', absPred[0])
        for i in range(np.size(signTemp)):
            if signTemp[i] == '>':
                self.signFS.append(1)
            else:
                self.signFS.append(-1)

        # Determine the size of the safe set (value on RHS of inequality)
        pref = re.findall('[+-]?\d+\.?\d*', self.params)
        if len(self.signFS) == 1:
            self.p = float(pref[-1])
        else:
            self.p = float(np.abs(pref[-1] - pref[0]))

        # Determine the function that needs to be evaluated (will be added with size of safe set)
        if len(self.signFS) == 1:
            funcOfref = re.search(r"(?=(\s|\w)).+?(?=(<|>))", self.params)[0]
        else:
            funcOfref = re.search('(?<=(<|>)).+?(?=(<|>))', self.params)[0]

        if '*' in str(funcOfref):
            dirRef = re.findall(r"(?=\().+?(?=\*)", str(funcOfref))
        else:
            dirRef = funcOfref

        # If we can find a direction use it, if not we can calculate a better one at runtime
        for i in range(len(dirRef)):
            try:
                self.dir.append(re.search(r'(?=p).+(?=\))', dirRef[i])[0])
                self.point.append(-1 * float(re.search('(?=(\+|\-)).+(?=\))', dirRef[i])[0]))
            except:
                try:
                    self.point.append(re.search('(?=posRef\[).+(?<=\])', dirRef[i])[0])
                except:
                    pass
        self.point = np.asarray(self.point)

        # Change function so that it can be read by python.
        self.funcOf = funcOfref

        # Find all of the robots involved for a task
        posUsed = re.findall('pos\[+[+-]?\d+\.?\d*\]', self.params)
        varUsed = np.empty((1, np.size(posUsed)), dtype='int')
        for v in range(np.size(posUsed)):
            varUsed[0, v] = int(re.search('(?<=\[)\d+(?=\])', posUsed[v])[0])

        for j in range(np.size(varUsed, 1)):
            robInv = np.ceil((varUsed[0][j] + 1) / 3)
            self.robotsInvolved.append(robInv)

        self.robotsInvolved = np.asarray(self.robotsInvolved, dtype='int')
        self.robotsInvolved = np.unique(self.robotsInvolved)

        # pre-allocate the nominal controller and assign robot involved
        inter = np.empty([1, np.size(self.dir) + 1], dtype='int')
        if len(self.dir) != 0:
            ll = 0
            for l in range(np.size(inter)):
                if ((l + 1) / 3).is_integer():
                    inter[0, l] = inter[0][l - 1] + 1
                else:
                    intere1 = re.search('pos\[+[+-]?\d+\.?\d*\]', self.dir[ll])
                    try:
                        intere = re.search('(?<=\[)\d+(?=\])', intere1[0])[0]
                    except:
                        pass
                    inter[0, l] = int(intere)
                    ll += 1
            inter[0][-1] = inter[0][-2] + 1
            self.nom[0:] = inter[0]
            self.nom[1:] = [0, 0, 0]
        else:
            self.nom[0:] = [3*self.robotsInvolved[0]-3,3*self.robotsInvolved[0]-2,3*self.robotsInvolved[0]-1]
            self.nom[1:] = [0, 0, 0]


