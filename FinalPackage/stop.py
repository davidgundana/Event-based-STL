import time, socket, json, pickle
import anki_vector
import time
import socket as skt
import os
from geometry_msgs.msg import TransformStamped
import sys, math, time
import socket, struct, threading
import rospy
import message_filters
import numpy as np

class runRobot:
    def __init__(self):
        self.currState = 0
        self.currTime = 0
        self.nRobot = 1
        self.posX = [None] * self.nRobot
        self.posY = [None] * self.nRobot
        self.posTheta = [None] * self.nRobot


    def connectToVectors(self):
        # List of serial numbers of the robots in order
        serialList = ["007039c3","00702c90","00702e03"]
        serialList = ["00903ba4"]
        realRobotList = []

        #Connect to each Vector and save object to a list
        for serialNum in serialList:
            realRobotList.append(anki_vector.Robot(serialNum))
            realRobotList[-1].connect()

        return realRobotList



if __name__ == '__main__':
    #Number of robots
    nRobot = 1
    r = runRobot()
    realRobotList = r.connectToVectors()
    currT = time.time()
    t = time.time()-currT

    for i in range(nRobot):
        realRobotList[i].motors.set_wheel_motors(300, 300)

    time.sleep(2)
    realRobotList[0].motors.set_wheel_motors(0, 0)


