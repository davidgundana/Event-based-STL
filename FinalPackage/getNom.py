import helperFuncs
import numpy as np
import re
from math import fabs, pi

def getNom(pi_mu, roadmap, x, xR, maxV,sizeU,sizeState):
    nom = np.zeros((1,sizeU))
    indOfI = None
    if np.size(pi_mu.point) > 1:
        avoidWallsInPath = 1
        wallDistance = .05
        startPos = x[3 * (pi_mu.robotsInvolved[0] - 1):3 * (pi_mu.robotsInvolved[0] - 1) + 2]
        canReach = 0
        point1 = eval(str(pi_mu.point[0]))
        point2 = eval(str(pi_mu.point[1]))
        isect = helperFuncs.intersectPoint(point1, point2, startPos[0], startPos[1],
                                    roadmap.map[:, 0], roadmap.map[:, 1], roadmap.map[:, 2], roadmap.map[:, 3])

        if not np.any(isect):
            dist2closest1, closestP = helperFuncs.distWall(startPos, np.array([point1,point2]), np.vstack((roadmap.map[:, 0:2], roadmap.map[:, 2:4])))
            if min(dist2closest1) > wallDistance:
                canReach = 1

        if canReach == 0:
            # Find the closest nodes to the goal
            dist2p = np.sqrt((point1 - roadmap.nodes[:, 0]) ** 2 +
                             (point2 - roadmap.nodes[:, 1]) ** 2)
            idx = np.argsort(dist2p)

            # Connect goal to first node it can. starting with closest
            isect = helperFuncs.intersectPointVec(point1, point2, roadmap.nodes[idx, 0], roadmap.nodes[idx, 1],
                                           roadmap.map[:, 0], roadmap.map[:, 1], roadmap.map[:, 2], roadmap.map[:, 3])
            closestGoalInd = idx[isect]

            if avoidWallsInPath:
                iToDel = []
                for i in range(np.size(closestGoalInd)):
                    dist2Walls, closestP = helperFuncs.distWall([point1,point2], roadmap.nodes[closestGoalInd[i], :],
                                               np.vstack((roadmap.map[:, 0:2], roadmap.map[:, 2:4])))
                    if min(dist2Walls) < wallDistance:
                        iToDel.append(i)
                closestGoalInd = np.delete(closestGoalInd, iToDel)

            closestGoalInd = closestGoalInd[0]
            closestGoal = roadmap.nodes[closestGoalInd]

            # Find the closest nodes to the start
            dist2p2 = np.sqrt((startPos[0] - roadmap.nodes[:, 0]) ** 2 + (startPos[1] - roadmap.nodes[:, 1]) ** 2)
            idx = np.argsort(dist2p2)

            isect = helperFuncs.intersectPointVec(startPos[0], startPos[1], roadmap.nodes[idx, 0], roadmap.nodes[idx, 1],
                                           roadmap.map[:, 0], roadmap.map[:, 1], roadmap.map[:, 2],roadmap.map[:, 3])
            closestStartInd = idx[isect]
            closestStartDist = np.sqrt((startPos[0] - roadmap.nodes[closestStartInd, 0]) ** 2 +
                                       (startPos[1] - roadmap.nodes[closestStartInd, 1]) ** 2)

            if avoidWallsInPath:
                iToDel = []
                for i in range(np.size(closestStartInd)):
                    dist2Walls, closestP = helperFuncs.distWall(startPos, roadmap.nodes[closestStartInd[i], :],
                                               np.vstack((roadmap.map[:, 0:2], roadmap.map[:, 2:4])))
                    if min(dist2Walls) < wallDistance:
                        iToDel.append(i)
                closestStartInd = np.delete(closestStartInd, iToDel)
                closestStartDist = np.delete(closestStartDist, iToDel)

            nodesToGo = [roadmap.nodeConnections[i][closestGoalInd][-1] for i in closestStartInd]
            distToGoals = np.asarray(nodesToGo) + np.asarray(closestStartDist)
            if np.size(distToGoals) != 0:
                indOfNext = np.argmin(distToGoals)
                wayPoint = roadmap.nodes[closestStartInd[indOfNext], :]
                tempNom = wayPoint - startPos
                mag = np.linalg.norm(tempNom)
                boundedNom = maxV[0] * (tempNom/mag)

                nom[0,0:2] = boundedNom
                # nom = np.hstack((nom, 0))
                closestStart = roadmap.nodes[closestStartInd[indOfNext]]
                costToStart = np.sqrt((closestStart[0] - startPos[0]) ** 2 + (closestStart[1] - startPos[1]) ** 2)
                costToGoal = np.sqrt((point1 - closestGoal[0]) ** 2 + (point2 - closestGoal[1]) ** 2)
                pathCost = roadmap.nodeConnections[closestStartInd[indOfNext]][closestGoalInd][-1]

                cost = costToStart + costToGoal + pathCost
        else:
            nom[0,0:2] = np.array([point1,point2]) - startPos
            # nom = np.hstack((nom, 0))
            cost = np.sqrt((point1 - startPos[0]) ** 2 + (point2 - startPos[1]) ** 2)
        if sizeU != sizeState:
            newNom = helperFuncs.feedbackLin(nom[0,0],nom[0,1],x[3 * (pi_mu.robotsInvolved[0] - 1) +2],.4,maxV[0])
            nom[0,0:2] = newNom

    else:
        if np.size(pi_mu.point) > 0:
            indOfI = int(re.search('(?<=\[)\d+(?=\])', pi_mu.dir[0])[0])
            goal = eval(str(pi_mu.point[0]))
        else:
            try:
                indOfI = int(re.search('(?<=\[)\d+(?=\])', pi_mu.dir)[0])
                goal = pi_mu.p + pi_mu.signFS[0]*1
            except:
                print('here')

        if indOfI == 2:
            goal = transform_to_pipi(goal)[0]
            current = transform_to_pipi(x[indOfI])[0]
            nominal = goal - current
            if np.abs(nominal) > np.pi:
                nominal = -1*nominal

        else:
            nominal = goal - x[indOfI]
        # if nominal > np.pi:
        #     nominal = x[indOfI] - eval(str(pi_mu.point[0]))

        if sizeState == sizeU:
            offset = 0
        else:
            offset = 1

        cost = nominal

        if nominal > maxV[indOfI-offset]:
            nominal = maxV[indOfI-offset]
        if sizeU != sizeState:
            nom[0,indOfI-1] = nominal
        else:
            nom[0,indOfI] = nominal

    return nom, cost, indOfI

def transform_to_pipi(input_angle):
    revolutions = int((input_angle + np.sign(input_angle) * pi) / (2 * pi))

    p1 = truncated_remainder(input_angle + np.sign(input_angle) * pi, 2 * pi)
    p2 = (np.sign(np.sign(input_angle)
                  + 2 * (np.sign(fabs((truncated_remainder(input_angle + pi, 2 * pi))
                                      / (2 * pi))) - 1))) * pi

    output_angle = p1 - p2

    return output_angle, revolutions

def truncated_remainder(dividend, divisor):
    divided_number = dividend / divisor
    divided_number = \
        -int(-divided_number) if divided_number < 0 else int(divided_number)

    remainder = dividend - divisor * divided_number

    return remainder