import numpy as np
import matrixDijkstra

def forward(specattr,potS):
    for i in range(np.size(specattr)):
        statesToDel = []
        for j in range(np.size(specattr[i].buchiStates)):
            if potS[i] != j:
                (cost, rute) = matrixDijkstra.dijkstra(specattr[i].graph, potS[i], j)
                if cost > 1000000:
                    statesToDel.append(j)
        if len(statesToDel) > 0:
            for j in range(np.size(statesToDel)):
                specattr[i].buchiStates.pop(statesToDel[j])
                if potS[i] > statesToDel[j]:
                    sign = -1
                else:
                    sign = 1
                for k in range(np.size(specattr[i].buchiStates)):
                    idOfDeleted = specattr[i].buchiStates[k].result.index(statesToDel[j])
                    specattr[i].buchiStates[k].cond.pop(idOfDeleted)
                    specattr[i].buchiStates[k].condCNF.pop(idOfDeleted)
                    specattr[i].buchiStates[k].result.pop(idOfDeleted)
                    specattr[i].buchiStates[k].result = [x + sign for x in specattr[i].buchiStates[k].result]

            specattr[i].graph = np.delete(specattr[i].graph, statesToDel, 0)
            specattr[i].graph = np.delete(specattr[i].graph, statesToDel, 1)
            print('buchi edited')
            # Check for accepting/cycle value problems and nRoutes problem

    return specattr, potS