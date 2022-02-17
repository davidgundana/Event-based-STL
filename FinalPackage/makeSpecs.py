import re
from nltk import Tree, ParentedTree
import numpy as np
import copy


def eventFormulas(EvSTL):
    # From back to front find all implication formulas
    impFormulas = [(m.start(0), m.end(0)) for m in re.finditer('=>', EvSTL)]
    inputCount = 0
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
        # make inputs in the formula start with "props"
        inpInForm = re.findall('\w+', form)
        for j in range(np.size(inpInForm)):
            form = re.sub(inpInForm[j], 'props.' + inpInForm[j], form)
        inpToReplace = 'input' + str(inputCount)
        inputCount += 1
        envInputs.append([inpToReplace, form])

    for i in range(len(formStart) - 1, -1, -1):
        EvSTL = EvSTL[:formStart[i]] + envInputs[i][0] + ' ' + EvSTL[formEnd[i]:]

    return EvSTL, envInputs


def getSpecs(EvSTL, envInputs):
    # Replace each predicate with a mu to make the tree easier to read
    labelsToSkip = ['(', '=>', '&', '|', ')']
    tempLabels = ['alw_', 'ev_', 'un_']
    inputLabels = []
    predLabels = []
    predCount = 0
    inputCount = 0
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
            # elif not any(tempLabels[s] in splitSTL[i] for s in range(np.size(tempLabels))) and splitSTL[i] != ' ':
            #     inpToReplace = 'props.' + str(inputCount)
            #     inputCount += 1
            #     inputLabels.append([splitSTL[i], inpToReplace])
            #     splitSTL[i] = inpToReplace

    treeSTL = ''.join(splitSTL)
    spotSTL = copy.deepcopy(treeSTL)
    if len(envInputs) != 0:
        for i in range(len(envInputs)):
            spotSTL = re.sub(envInputs[i][0], envInputs[i][1], spotSTL)
    # This version is almost ready for spot but the timing bounds need to be removed
    spotSTL = re.sub('(?=ev\_).+?(?<= )', 'F', spotSTL)
    spotSTL = re.sub('(?=alw\_).+?(?<= )', 'G', spotSTL)
    spotSTL = re.sub('(?=un\_).+?(?<= )', 'U', spotSTL)

    # The implication, conjunction, and disjunction, operators are not needed for the tree
    treeSTL = re.sub('\=\>', '', treeSTL)
    treeSTL = re.sub('\&', '', treeSTL)
    treeSTL = re.sub('\|', '', treeSTL)
    if treeSTL[0] == 'G':
        treeSTL = treeSTL[1:]
    t = ParentedTree.fromstring(treeSTL)

    return t, spotSTL, predLabels

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


def findInfo(t, pred):
    leaf_values = t.leaves()
    if pred in leaf_values:
        leaf_index = leaf_values.index(pred)
        tree_location = t.leaf_treeposition(leaf_index)
        predRef = copy.deepcopy(pred)
        pred = t[tree_location[:-1]].label()
        i = -2
        while pred == '':
            pred = t[tree_location[:i]].label()
            i -= 1
        if 'input' in pred:
            # make sure we dont accidentally get an input
            pred = t[tree_location[:-1]][0].label()

    if 'alw_' not in pred and 'ev_' not in pred and 'un_' not in pred:
        for subtree in t.subtrees():
            searchUntil = 0
            if subtree.label() == pred:
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
                            tempOperator = parent.label()

                if searchUntil == 1:
                    foundTemp = 0
                    pString = 'subtree'
                    while foundTemp == 0:
                        pString = pString + '.parent()'
                        parent = eval(pString)
                        for i in range(np.size(parent)):
                            try:
                                if 'un_' in parent[i]:
                                    foundTemp = 1
                                    tempOperator = parent[i]
                            except:
                                print('here')

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

                break
    else:
        tempOperator = pred
        for subtree in t.subtrees():
            if subtree.label() == pred:
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

    return tempOperator, allInputs


def handleUntil(STL, predLabels):
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
    for i in range(np.size(predLabels, 0)):
        for j in range(np.size(suff, 0)):
            toAppend = 0
            if predLabels[i][1] + ')' in pref[j] or predLabels[i][1] + ' ' in pref[j] or predLabels[i][1] in pref[j]:
                toAppend = 1
                break
        if toAppend == 1:
            predLabels[i].append(suff[j])
        else:
            predLabels[i].append('')

    return predLabels
