import re
import numpy as np
from nltk import Tree, ParentedTree
import copy
import time
import matrixDijkstra
import runEvBasedSTL
import networkx as nx
import itertools
from itertools import islice
import os
import pickle


def prepSpec(psi,sizeState,sizeU):
    # Ensure the specification is in negation normal form
    t1 = time.time()
    if '!' in psi:
        psi = negationNormal(psi)

    # Rename events for convenience. Also ensures format of events is correct for parse tree
    psi, inpRef, inpLabels,evProps = fixEvents(psi)

    # Collect all predicates in the specification
    mu, psi = getPreds(psi)

    # Create tree to find parent for all predicates
    parseTree = createTree(psi)
    Pi_mu = []
    for i in range(np.size(mu,0)):
        psi_until = searchUntil(mu[i][1], psi, parseTree)
        if psi_until:
            a,b,type = timingFromRight(mu[i][1],psi)
        else:
            a,b,type= timingFromLeft(mu[i][1],psi)

        alpha_i = findEvents(mu[i][1], parseTree)

        pi_mu = abstractedPred(mu[i],a,b,type,alpha_i,psi_until,i,sizeState, sizeU)
        Pi_mu.append(pi_mu)
    print('Total time to abstract predicates was {} seconds'.format(time.time()-t1))
    gamma = STL2LTL(psi)
    Pi_mu = handleUntil(gamma, Pi_mu)
    return Pi_mu, psi, inpRef, inpLabels,evProps, gamma

def getBuchi(Pi_mu, psi,buchiFile, text1, master, inpRef, inpLabels,evProps,gamma,bypass):
    t1 = time.time()
    print('Total time to translate to LTL Formula was {} seconds'.format(time.time()-t1))
    if not bypass:
        b_gamma, accepting = LTL2Buchi(gamma,buchiFile,bypass)
        # Parse the buchi
        if len(b_gamma) != 0:
            t2 = time.time()
            specattr = spec(b_gamma, accepting,inpRef, inpLabels,evProps)
            buchi = buchiInfo(specattr, Pi_mu, text1, master)
            elapsedT2 = time.time() - t2
            print('Time to organize Buchi into structure with labels translated to predicates: {}'.format(elapsedT2))
            # Further parse the buchi to make it easier to read
            t3 = time.time()
            buchiParsed = buchiInfo.prepBuchi(buchi, text1, master)
            elapsedT3 = time.time() - t3
            print('The total time translate Buchi to be read fast {} seconds.'.format(str(elapsedT3)))
            buchPath = os.path.dirname(os.path.abspath(__file__))
            pickle_file_path = os.path.join(buchPath, 'PickleFiles','buchiRef.pkl')
            with open(pickle_file_path, 'wb') as output:
                pickle.dump(buchiParsed, output, pickle.DEFAULT_PROTOCOL)
    else:
        buchPath = os.path.dirname(os.path.abspath(__file__))
        pickle_file_path = os.path.join(buchPath, 'PickleFiles', 'buchiRef.pkl')
        # Load pickle file
        with open(pickle_file_path, 'rb') as input:
            buchiParsed = pickle.load(input)
    return buchiParsed

def negationNormal(EvSTLForm):
    # Find instances of nested negation
    numToChange = np.size(re.findall('\!\(', EvSTLForm))
    while numToChange > 0:
        splitSTL = re.split(('(\)|\(|\!)'), EvSTLForm)
        splitSTL = [i for i in splitSTL if i != '']
        for i in range(np.size(splitSTL) - 1):
            if splitSTL[i] == '!' and splitSTL[i + 1] == '(':
                # Find the phrase until it the group is closed
                numOpen = 1
                numClose = 0
                j = i + 1
                while numOpen != numClose:
                    j += 1
                    if splitSTL[j] == '(':
                        numOpen += 1
                    if splitSTL[j] == ')':
                        numClose += 1
                strToChange = ''.join(splitSTL[i:j + 1])
                splitStrToChange = list(strToChange)
                # for the phrase that is negated, switch all signs and operators
                for jj in range(np.size(splitStrToChange)):
                    if splitStrToChange[jj] == '>':
                        splitStrToChange[jj] = '<'
                    elif splitStrToChange[jj] == '<':
                        splitStrToChange[jj] = '>'
                    elif splitStrToChange[jj] == '&':
                        splitStrToChange[jj] = '|'
                    elif splitStrToChange[jj] == '|':
                        splitStrToChange[jj] = '&'
                    elif splitStrToChange[jj] == '!':
                        splitStrToChange[jj] = ''

                # replace former negation with negation normal form
                splitSTL[i:j + 1] = ''.join(splitStrToChange)
                EvSTLForm = ''.join(splitSTL)
                break
        # Check if there are more instances of nested negation
        numToChange = np.size(re.findall('\!\(', EvSTLForm))

    # Find all cases of negation on predicates
    numRemaining = np.size(re.findall('\!', EvSTLForm))
    while numRemaining > 0:
        locOfNegation = re.search(('(?=\!).+?(?<= )'), EvSTLForm).span()
        strToChange = EvSTLForm[locOfNegation[0]:locOfNegation[1]]
        strToChange = re.sub('\!', '', strToChange)
        splitStrToChange = list(strToChange)
        # Change to be in negation normal form
        for i in range(np.size(splitStrToChange)):
            if splitStrToChange[i] == '>':
                splitStrToChange[i] = '<'
            elif splitStrToChange[i] == '<':
                splitStrToChange[i] = '>'
        EvSTLForm = EvSTLForm[:locOfNegation[0]] + ''.join(splitStrToChange) + EvSTLForm[locOfNegation[1]:]
        # Check if there are any remaining cases
        numRemaining = np.size(re.findall('\!', EvSTLForm))

    return EvSTLForm

def getPreds(EvSTL):
    # Replace each predicate with a mu to make the tree easier to read
    labelsToSkip = ['(', '=>', '&', '|', ')']
    predLabels = []
    predCount = 0
    STLForm = re.sub('\n', '', EvSTL)
    splitSTL = re.split(('(\)|\(| )'), STLForm)
    # Go through and find all controllable and uncontrollable propositions and label them
    for i in range(np.size(splitSTL)):
        if not any(labelsToSkip[s] in splitSTL[i] for s in range(np.size(labelsToSkip))) and splitSTL[i] != '':
            # Check the type of proposition
            if ('<' in splitSTL[i] and '=' not in splitSTL[i]) or ('>' in splitSTL[i] and '=' not in splitSTL[i]):
                predToReplace = 'props.pred' + str(predCount)
                predCount += 1
                predLabels.append([splitSTL[i], predToReplace])
                splitSTL[i] = predToReplace

    Psi = ''.join(splitSTL)

    return predLabels, Psi

def createTree(treeSTL):
    # The implication, conjunction, and disjunction, operators are not needed for the tree
    treeSTL = re.sub('\=\>', '', treeSTL)
    treeSTL = re.sub('\&', '', treeSTL)
    treeSTL = re.sub('\|', '', treeSTL)
    if treeSTL[0] == 'G':
        treeSTL = treeSTL[1:]
    try:
        t = ParentedTree.fromstring(treeSTL)
    except:
        print('Addition noticed for tree')
        t =  ParentedTree.fromstring('(' + treeSTL +')')
    return t

def searchUntil(mu,psi, parseTree):
    leaf_values = parseTree.leaves()
    if mu in leaf_values:
        psi_until = 0
    else:
        for subtree in parseTree.subtrees():
            searchUntil = 0
            if subtree.label() == mu:
                foundTemp = 0
                pString = 'subtree'
                while foundTemp == 0:
                    # This loop can return none so we need to always use the original subtree
                    pString = pString + '.parent()'
                    parent = eval(pString)
                    if parent is None:
                        # No parent that is a temporal operator. Must be an until case
                        foundTemp = 1
                        searchUntil = 1
                    elif parent.label() != '':
                        if 'alw_' in parent.label() or 'ev_' in parent.label() or 'un_' in parent.label():
                            foundTemp = 1

                if foundTemp:
                    break
        if searchUntil:
            psi_until = 1
        else:
            psi_until = 0
    pref = []
    if psi_until:
        untCases = [(m.start(0), m.end(0)) for m in re.finditer('un_\[', psi)]
        for i in range(np.size(untCases, 0)):
            j = untCases[i][0]
            numOpen = 0
            numClose = 1
            locToStart = [(m.start(0), m.end(0)) for m in re.finditer('\)', psi[:j])][-1][0]

            while numOpen != numClose:
                locToStart -= 1
                if psi[locToStart] == '(':
                    numOpen += 1
                if psi[locToStart] == ')':
                    numClose += 1

            pref.append(psi[locToStart:j])

        if not any(mu in substring for substring in pref):
            psi_until = 0
    return psi_until

def handleUntil(STL, Pi_mu):
    # First find all phrases with until
    untCases = [(m.start(0), m.end(0)) for m in re.finditer('U', STL)]
    pref = []
    suff = []
    for i in range(np.size(untCases, 0)):
        j = untCases[i][0]
        foundOpen = 0
        numOpen = 0
        numClose = 1
        locToStart = [(m.start(0), m.end(0)) for m in re.finditer('\)', STL[:j])][-1][0]

        while numOpen != numClose:
            locToStart -= 1
            if STL[locToStart] == '(':
                numOpen += 1
            if STL[locToStart] == ')':
                numClose += 1

        pref.append(STL[locToStart:j])

        numOpen = 1
        numClose = 0
        locToEnd = [(m.start(0), m.end(0)) for m in re.finditer('\(', STL[j:])][0][0] + j
        while numOpen != numClose:
            locToEnd += 1
            if STL[locToEnd] == '(':
                numOpen += 1
            if STL[locToEnd] == ')':
                numClose += 1
        suff.append(STL[j + 1:locToEnd + 1])

    toAppend = 0
    for i in range(np.size(Pi_mu, 0)):
        for j in range(np.size(suff, 0)):
            toAppend = 0
            if Pi_mu[i].prop_label + ')' in pref[j] or Pi_mu[i].prop_label + ' ' in pref[j] or Pi_mu[i].prop_label in pref[j]:
                toAppend = 1
                break
        if toAppend == 1:
            Pi_mu[i].phiUntil = suff[j]
        else:
            Pi_mu[i].phiUntil = ''

    return Pi_mu
def timingFromRight(mu,psi):
    mu_p = mu + '\)'
    timingLoc = re.split(mu_p, psi)[1]
    a_b = re.findall(r"[^[]*\[([^]]*)\]", timingLoc)[0]
    a_b_split = re.split(',',a_b)
    a = float(a_b_split[0])
    b = float(a_b_split[1])
    type = 'alw'
    # type =  re.findall(r'(alw|ev|un)', timingLoc)[0]


    return a, b, type

def timingFromLeft(mu,psi):
    mu_p = mu + '\)'
    mu_s = mu + ' '
    timingLoc = re.split('('+mu_p+'|'+mu_s+')', psi)[0]
    a_b = re.findall(r"[^[]*\[([^]]*)\]", timingLoc)[-1]
    a_b_split = re.split(',',a_b)
    a = float(a_b_split[0])
    b = float(a_b_split[1])
    type =  re.findall(r'(alw|ev|un)', timingLoc)[-1]
    if type == 'un':
        type = 'ev'
    return a,b,type

def findEvents(mu, parseTree):
    leaf_values = parseTree.leaves()
    if mu in leaf_values:
        leaf_index = leaf_values.index(mu)
        tree_location = parseTree.leaf_treeposition(leaf_index)
        predRef = copy.deepcopy(mu)
        mu = parseTree[tree_location[:-1]].label()
        i = -2
        while mu == '':
            mu = parseTree[tree_location[:i]].label()
            i -= 1
        if 'input' in mu:
            # make sure we dont accidentally get an input
            mu = parseTree[tree_location[:-1]][0].label()

    if 'alw_' not in mu and 'ev_' not in mu and 'un_' not in mu:
        for subtree in parseTree.subtrees():
            searchUntil = 0
            if subtree.label() == mu:
                foundTemp = 0
                pString = 'subtree'
                while foundTemp == 0:
                    # This loop can return none so we need to always use the original subtree
                    pString = pString + '.parent()'
                    parent = eval(pString)
                    if parent is None:
                        # No parent that is a temporal operator. Must be an until case
                        foundTemp = 1
                        searchUntil = 1
                    elif parent.label() != '':
                        if 'alw_' in parent.label() or 'ev_' in parent.label() or 'un_' in parent.label():
                            foundTemp = 1

                if searchUntil == 1:
                    foundTemp = 0
                    pString = 'subtree'
                    while foundTemp == 0:
                        pString = pString + '.parent()'
                        parent = eval(pString)
                        for i in range(np.size(parent)):
                            if 'un_' in parent[i]:
                                foundTemp = 1

                foundImp = 0
                allInputs = []
                while foundImp == 0:
                    if parent is not None:
                        if 'input' in parent.label():
                            allInputs.append(parent.label())
                    else:
                        foundImp = 1
                    if parent is not None:
                        parent = parent.parent()
    else:
        for subtree in parseTree.subtrees():
            if subtree.label() == mu:
                propsInTree = [subtree[i] for i in range(np.size(subtree))]
                if predRef in propsInTree:
                    foundImp = 0
                    allInputs = []
                    parent = subtree
                    while foundImp == 0:
                        parent = parent.parent()
                        if parent is not None:
                            if 'input' in parent.label():
                                allInputs.append(parent.label())
                        else:
                            foundImp = 1
                    break

    return allInputs

def fixEvents(EvSTL):
    # From back to front find all implication formulas
    impFormulas = [(m.start(0), m.end(0)) for m in re.finditer('=>', EvSTL)]
    inputCount = 0
    inputCountP = 0
    envInputs = []
    formStart = []
    formEnd = []
    for i in range(np.size(impFormulas, 0)):
        j = impFormulas[i][0]
        numOpen = 0
        numClose = 1
        if ')' in EvSTL[:j]:
            locToStart = [(m.start(0), m.end(0)) for m in re.finditer('\)', EvSTL[:j])][-1][0]
            adjust = 0
        else:
            locToStart = j
            adjust = 1

        while numOpen != numClose:
            locToStart -= 1
            if EvSTL[locToStart] == '(':
                numOpen += 1
            if EvSTL[locToStart] == ')':
                numClose += 1

        formStart.append(locToStart + adjust)
        formEnd.append(j)

        form = EvSTL[locToStart + adjust:j]
        form = re.sub("sqrt", "np.sqrt", form)
        form = re.sub("abs", "np.abs", form)
        form = re.sub('\^', '**', form)
        form = re.sub('&', ' and ', form)
        form = re.sub('\|', 'or', form)
        form = re.sub('!', 'not ', form)

        posOfI = re.findall("\[(\d+)\]", form)
        form = re.sub('\[', '(', form)
        form = re.sub('\]', ')', form)
        for i in range(np.size(posOfI)):
            oldPos = "x\(" + str(posOfI[i]) + "\)"
            newPos = "x[" + str(posOfI[i]) + "]"
            form = re.sub(oldPos, newPos, form)

            oldPosRef = "xR\(" + str(posOfI[i]) + "\)"
            newPosRef = "xR[" + str(posOfI[i]) + "]"
            form = re.sub(oldPosRef, newPosRef, form)

            oldWallRef = "wall\(" + str(posOfI[i]) + "\)"
            newWallRef = "wall[" + str(posOfI[i]) + "]"
            form = re.sub(oldWallRef, newWallRef, form)


        # make inputs in the formula start with "props"
        splitForm = re.split('( and | or )',form)
        inpInForm = []
        inpInFormRef = []
        for j in range(np.size(splitForm)):
            if ' and ' not in splitForm[j] and ' or ' not in splitForm[j]:
                if '<' not in splitForm[j] and '>' not in splitForm[j]:
                    inpInForm.append(re.findall('\w+', splitForm[j])[0])
                    inpInFormRef.append(re.findall('\w+', splitForm[j])[0])
                else:
                    inpInForm.append(re.search('(?<=(\(|\w)).+?(?=(\s))', splitForm[j])[0])
                    inpInFormRef.append('predInput'+str(inputCountP))
                    inputCountP +=1

        for j in range(np.size(inpInForm)):
            if '<' not in inpInForm[j] and '>' not in inpInForm[j]:
                form = form.replace(inpInForm[j], 'props.' + inpInFormRef[j])
        inpToReplace = 'props.input' + str(inputCount)
        inputCount += 1
        envInputs.append([inpToReplace, form])

    for i in range(len(formStart) - 1, -1, -1):
        EvSTL = EvSTL[:formStart[i]] + envInputs[i][0] + ' ' + EvSTL[formEnd[i]:]

    inpLabels = []
    evProps = propos()
    for i in range(np.size(envInputs,0)):
        allInputs = re.split('( and | or )',envInputs[i][1])
        for j in range(np.size(allInputs,0)):
            if not ('<' in allInputs[j] and '<' in allInputs[j]) and allInputs[j] != ' and ' and allInputs[j] != ' or ':
                cleanLabel = re.search('(?<=(\.)).+?(?=(\s|\)))',allInputs[j])[0]
                if not any(cleanLabel in sublist for sublist in inpLabels):
                    labelToApp = [envInputs[i][0], cleanLabel]
                    inpLabels.append(labelToApp)
                    exec('evProps.' + str(cleanLabel) + '=0')
    return EvSTL, envInputs, inpLabels,evProps

def STL2LTL(spotSTL):
    # This version is almost ready for spot but the timing bounds need to be removed
    spotSTL = re.sub('(?=ev\_).+?(?<= )', 'F', spotSTL)
    spotSTL = re.sub('(?=alw\_).+?(?<= )', 'G', spotSTL)
    spotSTL = re.sub('(?=un\_).+?(?<= )', 'U', spotSTL)

    return spotSTL

def LTL2Buchi(gamma, buchiFile,bypass):
    if not bypass:
        try:
            import spot
            bypass = 0
        except:
            bypass = 1

    ready = 1
    if bypass:
        # Load in the Buchi
        print('Enter this specification in Spot: \n' + gamma)
        try:
            x2 = open(buchiFile, 'r').readlines()
            auto = x2
        except:
            print('No Buchi uploaded. Re-run program and upload buchi generated by Spot')
            ready = 0
    else:
        t = time.time()
        print('LTL Specification: ' + gamma)
        # Generate Buchi automaton and save
        buch = spot.translate(gamma, 'BA', 'deterministic', 'complete', 'sbacc')
        # buch = spot.translate(gamma, 'BA', 'deterministic', 'sbacc')

        accepting = find_accepting_states(buch.to_str('dot'))
        print('Number of states: ' + str(buch.num_states()))
        print('Number of edges: ' + str(buch.num_edges()))

        auto = buch.to_str('HOA')
        auto = re.sub('&', ' and ', auto)
        auto = re.sub('\|', 'or', auto)
        auto = re.sub('!', 'not ', auto)
        auto = re.sub('\[', ' ', auto)
        auto = auto.splitlines()
        auto = auto[1:]

        elapsedT = time.time() - t
        timeToCompile = 'The total time to generate the Buchi automaton was ' + str(elapsedT) + ' seconds.'
        # with open('buchiRef.txt', 'w') as filehandle:
        #     for listitem in auto:
        #         filehandle.write('%s\n' % listitem)
        print(timeToCompile)

    if ready:
        return auto, accepting
    else:
        return [], []

def find_accepting_states(dot_str):
    '''
    to be an accepting state: 'peripheries=2' and there exists transition state -> state
    '''
    states = []
    for line in dot_str.split('\n'):
        if line.find('peripheries=2') != -1:
            s = line.split()[0]
            states.append(s)
    return states

class abstractedPred:
    def __init__(self,pred,a,b,type,alpha,phi_unt,i,sizeState, sizeU):
        self.pred = pred
        self.id = i
        self.a = a
        self.b = b
        self.type = type
        self.alpha = alpha
        self.phi_unt = phi_unt
        self.signFS = []
        self.hxt = []
        self.hxte = []
        self.cbf = []
        self.p = []
        self.prop_label = []
        self.t_e = []
        self.implies = 0
        self.currentTruth = ''
        self.distFromSafe = 0
        self.robotsInvolved = []
        self.dir = []
        self.point = []
        self.time2Finish = ''
        self.sizeState = sizeState
        self.sizeU = sizeU
        self.nom = np.zeros([2, sizeU], dtype='float') # Nominal controller
        self.convenienceTerms()

    def convenienceTerms(self):
        signTemp = re.findall(r'(<|>)', self.pred[0])
        for i in range(np.size(signTemp)):
            if signTemp[i] == '>':
                self.signFS.append(1)
            else:
                self.signFS.append(-1)

        self.pred[0] = re.sub("sqrt", "np.sqrt", self.pred[0])
        self.pred[0] = re.sub("abs", "np.abs", self.pred[0])
        self.pred[0] = re.sub('\^', '**', self.pred[0])

        posOfI = re.findall("\[(\d+)\]", self.pred[0])
        self.pred[0] = re.sub('\[', '(', self.pred[0])
        self.pred[0] = re.sub('\]', ')', self.pred[0])
        for i in range(np.size(posOfI)):
            oldPos = "x\(" + str(posOfI[i]) + "\)"
            newPos = "x[" + str(posOfI[i]) + "]"
            self.pred[0] = re.sub(oldPos, newPos, self.pred[0])

            oldPosRef = "xR\(" + str(posOfI[i]) + "\)"
            newPosRef = "xR[" + str(posOfI[i]) + "]"
            self.pred[0] = re.sub(oldPosRef, newPosRef, self.pred[0])

            oldWallRef = "wall\(" + str(posOfI[i]) + "\)"
            newWallRef = "wall[" + str(posOfI[i]) + "]"
            self.pred[0] = re.sub(oldWallRef, newWallRef, self.pred[0])

        pref = re.findall('[+-]?\d+\.?\d*', self.pred[0])
        if len(self.signFS) == 1:
            self.p = float(pref[-1])
        else:
            self.p = float(np.abs(pref[-1] - pref[0]))

        if len(self.signFS) == 1:
            self.hxt = re.search(r"(?=(\s|\w)).+?(?=(<|>))", self.pred[0])[0]
        else:
            self.hxt = re.search('(?<=(<|>)).+?(?=(<|>))', self.pred[0])[0]

        self.cbf = 'pi.hxte - ((t-pi.t_e-pi.a)*pi.hxte)/(pi.b-pi.a)  + pi.p'
        self.prop_label = self.pred[1]
        if len(self.alpha) > 0:
            self.implies = 1

        posUsed = re.findall('x\[+[+-]?\d+\.?\d*\]', self.hxt)
        varUsed = np.empty((1, np.size(posUsed)), dtype='int')
        for v in range(np.size(posUsed)):
            varUsed[0, v] = int(re.search('(?<=\[)\d+(?=\])', posUsed[v])[0])

        for j in range(np.size(varUsed, 1)):
            robInv = np.ceil((varUsed[0][j] + 1) / self.sizeState)
            self.robotsInvolved.append(robInv)
        self.robotsInvolved = np.asarray(self.robotsInvolved, dtype='int')
        self.robotsInvolved = np.unique(self.robotsInvolved)

        if '*' in str(self.hxt):
            dirRef = re.findall(r"(?=\().+?(?=\*)", str(self.hxt))
        else:
            dirRef = re.findall(r"(?=\().+?(?=$)", str(self.hxt))

        # If we can find a direction use it, if not we can calculate a better one at runtime
        for i in range(len(dirRef)):
            try:
                self.dir.append(re.search(r'(?=x).+(?=\))', dirRef[i])[0])
                self.point.append(-1 * float(re.search('(?=(\+|\-)).+(?=\))', dirRef[i])[0]))
            except:
                try:
                    self.point.append(re.search('(?=xR\[).+(?<=\])', dirRef[i])[0])
                except:
                    pass
        self.point = np.asarray(self.point)

        inter = np.empty([1, np.size(self.dir)], dtype='int')
        if len(self.dir) != 0:
            ll = 0
            for l in range(np.size(inter)):
                if ((l + 1) / 3).is_integer():
                    inter[0, l] = inter[0][l - 1] + 1
                else:
                    intere1 = re.search('x\[+[+-]?\d+\.?\d*\]', self.dir[ll])
                    try:
                        intere = re.search('(?<=\[)\d+(?=\])', intere1[0])[0]
                    except:
                        pass
                    inter[0, l] = int(intere)
                    ll += 1
            # inter[0][-1] = inter[0][-2] + 1
            self.nom[0:] = np.arange(self.sizeU * (np.floor(inter[0][0]/self.sizeU)), self.sizeU * (np.floor(inter[0][0]/self.sizeU)) + self.sizeU)
            self.nom[1:] = self.sizeU *[0]
        else:
            self.nom[0:] = np.arange(self.sizeU * (np.floor(self.robotsInvolved[0]/self.sizeU)), self.sizeU * (np.floor(self.robotsInvolved[0]/self.sizeU)) + self.sizeU)
            self.nom[1:] = self.sizeU *[0]
            self.dir = self.hxt

class spec:
    def __init__(self,spec,accepting,inpRef,inpLabels, evProps):
        self.spec = spec # specification
        self.accepting_states = accepting # pre-allocate accepting states
        self.controllableProp = [] # pre-allocate controllable propositions
        self.props = [] # value of propositions
        self.propositions = []
        self.N = []
        self.inpRef = inpRef
        self.inpLabels = inpLabels
        self.evProps = evProps
        self.initSpec()


    def initSpec(self):
        # Replace the exclamation points with readable values for python and create an object with the
        # values of all propositions
        propo = [re.findall('(?<=\.)\w*', elem) for elem in self.spec]
        flat_propo = [item for sublist in propo for item in sublist]
        indexes = np.unique(flat_propo, return_index=True)[1]
        propositions = [flat_propo[index] for index in sorted(indexes)]
        # propositions.sort()
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

class buchiInfo:
    def __init__(self,specattr, Pi_mu, text1, master):
        self.buchi = specattr.spec #buchi automaton
        self.graph = [] # Transition graph
        self.buchiStates = [] # States in the system
        self.accepting_states = np.array(specattr.accepting_states,dtype='int') # Set of accepting states
        self.props = specattr.props # proposition values
        self.controllableProp = specattr.controllableProp # Set of controllable propositions
        self.acceptingWithCycle = []
        self.Pi_mu = Pi_mu
        self.uncontrollableProp = [] # Set of uncontrollable Propositions
        self.locOfUncontrollable = []
        self.locOfControllable = []
        self.propositions = specattr.propositions
        self.inpRef = specattr.inpRef
        self.inpLabels = specattr.inpLabels
        self.evProps = specattr.evProps
        self.N = []
        self.specToPick = []
        self.input = []
        self.inputVal = []
        self.wall = []
        # self.phi = phi # Parsed spec
        self.controllablePropOrder = [] # Order of controllable propositions used to ensure things do not get mixed
        self.parameters = [] # Parameters of interest (fucntions to evaluate)
        # self.M = M # Number of robots
        # self.nodeGraph = [] # Graph of nodes for map
        # self.nodes = nodes # nodes in map
        # self.map = map # map
        # self.nodeGraph = nodeGraph
        # self.nodeConnections = nodeConnections
        self.beginSpec(specattr,text1,master)


    def beginSpec(self,specattr,text1,master):
        self.buchiStates, self.graph, self.acceptingWithCycle = buchiStates(self.buchi, self.accepting_states, specattr.propositions)
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

            if text1 != '':
                percentage = round(percentages[i], 2)
                text1.configure(state='normal')
                text1.delete("end-1l", "end")
                text1.configure(state='disabled')
                complete_status = str(percentage) + '% complete'
                if percentage != 100:
                    message = '\nFinding Routes through Buchi ' + complete_status
                else:
                    message = '\nFinding Routes through Buchi ' + complete_status + '\n'

                runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData, text1, master, message)

        self.nRoutes = nRoutes
        # Because of the naming convention, the o
        for i in range(np.size(self.controllableProp)):
            propName = self.controllableProp[i]
            # find the phi that matches this prop
            for j in range(np.size(self.Pi_mu)):
                isThere = re.findall(propName, self.Pi_mu[j].prop_label)
                if isThere:
                    break
            if isThere:
                self.controllablePropOrder.append(j)

        # Create a set of the parameters in the order that they appear
        # controllablePropOrder = np.unique(self.controllablePropOrder)
        for i in range(np.size(self.controllablePropOrder)):
            # self.parameters.append(self.Pi_mu[self.controllablePropOrder[i]].pred[0])
            self.parameters.append(self.Pi_mu[i].pred[0])


        controllableProp = np.array(self.controllableProp)
        self.uncontrollableProp = np.setdiff1d(self.propositions, controllableProp).tolist()
        self.locOfUncontrollable = [list(self.propositions).index(s) for s in list(self.uncontrollableProp)]
        self.locOfControllable = np.asarray([list(self.propositions).index(s) for s in list(self.controllableProp)])

        self.N = np.size(self.uncontrollableProp)
        self.input = np.zeros((1, 2 * self.N), dtype=float)[0]

    def prepBuchi(self, text1, master):
        # Track status
        percentages = list(np.linspace(0, 100, np.size(self.buchiStates)+1))
        percentages = percentages[1:]
        reference = 0
        # Transform complex conditions for a transition to a set of  values (0,1,2). This will help with
        # evaluating at runtime
        totalTransitions = 0
        column = [0] * np.size(self.propositions)

        for i in range(np.size(self.buchiStates)):
            self.buchiStates[i].condCNF = np.size(self.buchiStates[i].cond) * [column]
            for j in range(np.size(self.buchiStates[i].cond)):
                allRCond = np.zeros((1,np.size(self.propositions)),dtype = int)
                optionsForTrans = re.split('or', self.buchiStates[i].cond[j])
                for k in range(np.size(optionsForTrans)):
                    thisOption = re.split('and', optionsForTrans[k])
                    CNFrep = np.size(thisOption) * [1]
                    for p in range(np.size(thisOption)):
                        if 'not' in thisOption[p]:
                            CNFrep[p] = 0

                    refNum = []
                    refProp = []
                    for l in range(np.size(self.propositions)):
                        propo = self.propositions[l]+')'
                        if propo in optionsForTrans[k]:
                            refNum.append(l)
                            refProp.append(next(obj for obj, v in enumerate(thisOption) if propo in v))

                    finalPropVals = np.asarray([np.inf] * np.size(self.propositions))
                    finalPropVals[self.locOfUncontrollable] = np.size(self.uncontrollableProp) *[2]
                    finalPropVals = list(finalPropVals)
                    for l in range(np.size(refNum)):
                        finalPropVals[refNum[l]] = CNFrep[refProp[l]]

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
                totalTransitions += np.size(allRCond,0)
                allRCond = allRCond.tolist()

                self.buchiStates[i].condCNF[j] = allRCond

            if text1 != '':
                # Update status of preparation
                percentage = round(percentages[i],2)
                text1.configure(state='normal')
                text1.delete("end-1l", "end")
                text1.configure(state='disabled')
                complete_status = str(percentage) + '% complete'
                if percentage != 100:
                    message = '\nPreparing Buchi. ' + complete_status
                else:
                    message = '\nPreparing Buchi. ' + complete_status + '\n'

                runEvBasedSTL.formData.updateStatus(runEvBasedSTL.formData,text1,master,message)

        print('Total Transitions in Buchi: ' + str(totalTransitions))

        return self
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

def buchiStates(buchi, accepting_states, propositions):
    # Find the number of States
    numStates = int(re.split(' ', buchi[0])[-1])

    # Find the location of all states
    states = [re.findall('(State\:)', elem) for elem in buchi]
    idx = [i for i, x in enumerate(states) if len(x) > 0]
    idx.append(len(states) - 1)

    # Go through all States and find the conditions of transitions, the results of the conditions,
    # and the accepting states
    State = []
    for i in range(numStates):
        State.append(StatesOfI())
        condRef = buchi[idx[i] + 1:idx[i + 1]]
        for j in range(len(condRef)):
            condTemp = re.split('] ', condRef[j])
            if condTemp[0] != 't':
                for k in range(np.size(propositions)):
                    condTemp[0] = re.sub(' ' + str(k) + ' ', ' (props.' + propositions[k] + ') ', condTemp[0])
                    condTemp[0] = re.sub(' ' + str(k) + '$', ' (props.' + propositions[k] + ') ', condTemp[0])

                condTemp[0] = re.sub(' or ', ' ) or ( ', condTemp[0])
                condTemp[0] = '(' + condTemp[0] + ')'
                State[i].cond.append(condTemp[0])
            else:
                State[i].cond.append('All')
            State[i].result.append(int(condTemp[1]))

    graph = np.zeros((np.size(State), np.size(State)))
    for i in range(np.size(State)):
        for j in range(np.size(State[i].result)):
            graph[i, int(State[i].result[j])] = 1

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
            if cost == 1:
                acceptingWithCycle.append(accepting_states[j])

    return State, graph, acceptingWithCycle