import numpy as np

def intersectPoint(x1, y1, x2, y2, x3, y3, x4, y4):
    ua = np.divide(((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)), ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)))
    ub = np.divide(((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)), ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)))

    isect = (ua >= 0) * (ub >= 0) * (ua <= 1) * (ub <= 1)
    return isect


def intersectPointVec(x1, y1, x2, y2, x3, y3, x4, y4):
    with np.errstate(divide='ignore', invalid='ignore'):
        denom = np.outer((x2 - x1), (y4 - y3)) - np.outer((y2 - y1), (x4 - x3))

        ua = np.divide(((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)), denom)
        ub = (np.outer((x2 - x1), (y1 - y3)) - np.outer((y2 - y1), (x1 - x3))) / denom

        isect = (ua >= 0) * (ub >= 0) * (ua <= 1) * (ub <= 1)
        isect = np.any(isect, axis=1)
        isect = np.where(isect == False)[0]
        return isect

def distWall(p1, p2, pt):
    with np.errstate(divide='ignore', invalid='ignore'):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        closestP = []
        try:
            t = ((pt[:,0] - p1[0]) * dx + (pt[:,1] - p1[1]) * dy) / (dx ** 2 + dy ** 2)
        except:
            t = ((pt[0] - p1[0]) * dx + (pt[1] - p1[1]) * dy) / (dx ** 2 + dy ** 2)

        if dx == 0 and dy == 0:
            closestP = p1
            dx = pt[0] - p1[0]
            dy = pt[1] - p1[1]
            if pt.ndim > 1:
                dist = np.zeros((1,np.size(pt,0)))[0]
            # dist = np.sqrt(dx ** 2 + dy ** 2) * np.ones((1,np.size(pt,0)))[0]
        else:
            if pt.ndim > 1:
                dist = np.zeros((1,np.size(pt,0)))[0]

        if np.size(np.where(t<0)[0]) > 0:
            indL = np.where(t<0)[0]
            if pt.ndim > 1:
                dx = pt[indL, 0] - p1[0]
                dy = pt[indL, 1] - p1[1]
                dist[indL] = np.sqrt(dx ** 2 + dy ** 2)
            else:
                dx = pt[0] - p1[0]
                dy = pt[1] - p1[1]
                dist = np.sqrt(dx ** 2 + dy ** 2)
                closestP = np.array([p1[0], p1[1]])

        if np.size(np.where(t>1)[0]) > 0:
            indG = np.where(t>1)[0]
            if np.size(indG) > 1:
                dx = pt[indG,0] - p2[0]
                dy = pt[indG,1] - p2[1]
                dist[indG] = np.sqrt(dx ** 2 + dy ** 2)
            else:
                dx = pt[0] - p2[0]
                dy = pt[1] - p2[1]
                dist = np.sqrt(dx ** 2 + dy ** 2)
                closestP = np.array([p2[0], p2[1]])

        if np.size(np.where((t >= 0) & (t <= 1))[0]) > 0:
            indM = np.where((t >= 0) & (t <= 1))[0]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            if pt.ndim > 1:
                closestP = np.array([p1[0] + t[indM] * dx, p1[1] + t[indM] * dy])
                dx = pt[indM, 0] - closestP[0]
                dy = pt[indM, 1] - closestP[1]
                dist[indM] = np.sqrt(dx ** 2 + dy ** 2)
            else:
                closestP = np.array([p1[0] + t * dx, p1[1] + t * dy])
                dx = pt[0] - closestP[0]
                dy = pt[1] - closestP[1]
                dist = np.sqrt(dx ** 2 + dy ** 2)

        return dist, closestP

def feedbackLin(vx, vy, theta, epsilon,maxV):
    R = np.array([[np.cos(theta), np.sin(theta)], \
                      [-np.sin(theta), np.cos(theta)]])
    dirVelocities = np.array([[vx], [vy]])
    rot = np.array([[1.0, 0.0], [0.0, 1/epsilon]])

    fVelAngVel = np.dot(np.dot(rot, R),dirVelocities).T[0]
    cmdV,cmdW = limitCMDs(fVelAngVel[0],fVelAngVel[1],maxV,0.16)

    return [cmdV,cmdW]

def limitCMDs(fwdVel,angVel,maxV,wheel2Center):
    maxW = maxV / wheel2Center

    ratioV = abs(fwdVel / maxV)
    ratioW = abs(angVel / maxW)
    ratioTot = ratioV + ratioW

    if ratioTot > 1:
        cmdV = fwdVel / ratioTot
        cmdW = angVel / ratioTot
    else:
        cmdV = fwdVel
        cmdW = angVel


    if abs(cmdW) < 1e-5:
        cmdW = 0

    return cmdV,cmdW

def integrateOdom(ut,pose):
    d = ut[0]
    phi = ut[1]

    x = pose[0]
    y = pose[1]
    theta = pose[2]

    if d == 0:
        new_theta = theta + phi
        new_pose = [x, y, new_theta]

    elif phi == 0:
        x_new = x + d * np.cos(theta)
        y_new = y + d * np.sin(theta)

        new_pose = [x_new, y_new, theta]

    else:
        R = d / phi
        L = 2 * R * np.sin(phi / 2)

        L_vec_Body = [L * np.cos(phi / 2),L * np.sin(phi / 2),1]

        R_GB = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        H_GB = np.hstack((R_GB, np.array([[x],[y]])))
        H_GB = np.vstack((H_GB,np.array([0,0,1])))

        L_vec_Global = np.dot(H_GB, L_vec_Body)

        theta_new = theta + phi
        new_pose = L_vec_Global[0:2].tolist()

        new_pose.append(theta_new)

    return new_pose

def robot2global(pose,xyR):
    Tab = np.array([[np.cos(pose[2]), -np.sin(pose[2]), pose[0]],
           [np.sin(pose[2]), np.cos(pose[2]), pose[1]],
           [0, 0, 1]])

    xyG = np.dot(Tab,np.array([xyR[0],xyR[1], 1]).T)

    return xyG


