import time, socket, json, pickle
import anki_vector
import time
import socket as skt
import os
from geometry_msgs.msg import TransformStamped
import sys, math, time
import socket, struct, threading
import rospy
import numpy as np
from prepForCommands import getCMD
from runEvBasedSTL import formData

class runRobot:
    def __init__(self,f):
        self.currState = 0
        self.currTime = 0
        self.robotsUsed = [3]
        self.nRobot = len(self.robotsUsed)
        self.masterSerial = ["007039c3","00603393","00702c90","00702e03","005031a6","0040161d"]
        self.serialList = []
        for i in range(self.nRobot):
            self.serialList.append(self.masterSerial[self.robotsUsed[i]-1])
        self.posX = [None] * self.nRobot
        self.posY = [None] * self.nRobot
        self.posTheta = [None] * self.nRobot
        self.f = f
        self.posXinit = []
        self.posYinit = []
        self.posThetainit = []
        self.posPX = []
        self.posPY = []
        self.posPTheta = []
        for i in range(int(f.M)):
            self.posXinit = np.append(self.posXinit,f.initPos[3*i])
            self.posYinit = np.append(self.posYinit,f.initPos[3*i+1])
            self.posThetainit = np.append(self.posThetainit,f.initPos[3*i+2])
        for i in range(int(f.P)):
            self.posPX = np.append(self.posPX, float(f.initPosRef[3 * i]))
            self.posPY = np.append(self.posPY, float(f.initPosRef[3 * i + 1]))
            self.posPTheta = np.append(self.posPTheta, float(f.initPosRef[3 * i + 2]))
        self.input = np.zeros((1, 2 * f.N), dtype=float)[0]

    def connectToVectors(self):
        realRobotList = []

        #Connect to each Vector and save object to a list
        for serialNum in self.serialList[:-1]:
            realRobotList.append(anki_vector.Robot(serialNum))
            realRobotList[-1].connect()

        return realRobotList

    def listener(self):
        # Connect to Vector Robots
        self.realRobotList = self.connectToVectors()
        print('Connecting to Vicon...')


        rospy.init_node('listener', anonymous=True)

        if self.nRobot > 0:
            subTopic = "/vicon/vector{0}/vector{0}".format(str(self.robotsUsed[0]))
            rospy.Subscriber(subTopic, TransformStamped, self.callback0, queue_size=1)
            print('Connected to: ' + subTopic)
        if self.nRobot > 1:
            subTopic = "/vicon/vector{0}/vector{0}".format(str(self.robotsUsed[1]))
            rospy.Subscriber(subTopic, TransformStamped, self.callback1, queue_size=1)
            print('Connected to: ' + subTopic)
        if self.nRobot > 2:
            subTopic = "/vicon/vector{0}/vector{0}".format(str(self.robotsUsed[2]))
            rospy.Subscriber(subTopic, TransformStamped, self.callback2, queue_size=1)
            print('Connected to: ' + subTopic)
        if self.nRobot > 3:
            subTopic = "/vicon/vector{0}/vector{0}".format(str(self.robotsUsed[3]))
            rospy.Subscriber(subTopic, TransformStamped, self.callback3, queue_size=1)
            print('Connected to: ' + subTopic)
        if self.nRobot > 4:
            subTopic = "/vicon/vector{0}/vector{0}".format(str(self.robotsUsed[4]))
            rospy.Subscriber(subTopic, TransformStamped, self.callback4, queue_size=1)
            print('Connected to: ' + subTopic)
        if self.nRobot > 5:
            subTopic = "/vicon/vector{0}/vector{0}".format(str(self.robotsUsed[5]))
            rospy.Subscriber(subTopic, TransformStamped, self.callback5, queue_size=1)
            print('Connected to: ' + subTopic)


        print('Starting Thread')

        self.thread = threading.Thread(target=self.sendAllCMDs)
        self.thread.daemon = True
        self.thread.start()

        rospy.spin()

    def sendAllCMDs(self):
        time.sleep(2)
        startTime = time.time()
        runTime = 0
        triggered = 0
        self.posPX[0] = self.posX[-1]
        self.posPY[0] = self.posY[-1]
        self.posPTheta[0] = self.posTheta[-1]
        # print(self.posPX,self.posPY)

        while True:
            # Stop the Thread if script closed
            global stop_threads
            if stop_threads:
                break
            STL = 1
            if STL:
                # vx, vy, vt, self.currState, dis, newInput, unt = getCMD(self.f, self.posX, self.posY, self.posTheta,
                #                                                                  self.posXinit, self.posYinit,
                #                                                                  self.posThetainit, self.posPX,
                #                                                                  self.posPY, self.posPTheta,
                #                                                                  runTime, 0, self.currState,
                #                                                                  self.input, [])
                # ignore last robot because its uncontrolled
                vx, vy, vt, self.currState, dis, newInput, unt = getCMD(self.f, self.posX[:-1], self.posY[:-1], self.posTheta[:-1],
                                                                                 self.posXinit, self.posYinit,
                                                                                 self.posThetainit, self.posPX,
                                                                                 self.posPY, self.posPTheta,
                                                                                 runTime, 0, self.currState,
                                                                                 self.input, [])
                vx = vx[0]
                vy = vy[0]
                runTime = time.time()-startTime
                print('Time: {}s'.format(str(round(runTime,2))))
                uncontDist = np.sqrt((-1.8 - self.posPX[0])**2 + (-1.6 - self.posPY[0])**2)

                if uncontDist < 0.3 and triggered == 0:
                    if self.input[0] == 0:
                        self.input[1] = runTime
                    self.input[0] = 1
                    triggered = 1
                if runTime - self.input[1] > 3:
                    self.input[0] = 0
                    self.input[1] = 0


                if runTime > 2:
                    self.input[2] = 1
                    self.input[3] = 2
                if runTime > 4:
                    self.input[2] = 0
                    self.input[3] = 0
                if f.N > 2:
                    if runTime > 5:
                        self.input[4] = 1
                        self.input[5] = 5
                    if runTime > 8:
                        self.input[4] = 0
                        self.input[5] = 0
                # self.input = newInput

                self.posPX[0] = self.posX[-1]
                self.posPY[0] = self.posY[-1]
                self.posPTheta[0] = self.posTheta[-1]
                # self.posPX[0]=self.posX[0]
                # self.posPY[0] = self.posY[0]
                # self.posPTheta[0] = self.posTheta[0]

                vr = []
                vl = []
                for i in range(np.size(vx)):
                    if np.abs(vx[i]) < 0.015 and np.abs(vy[i]) < 0.015:
                        vrTemp = 0
                        vlTemp = 0
                        # print('preventing low input movement')
                    else:
                        vrTemp, vlTemp = self.feedbackLin(self.posTheta[i], vx[i], vy[i])
                    vr.append(vrTemp)
                    vl.append(vlTemp)

            else:
                try:
                    vr = []
                    vl = []
                    for i in range(self.nRobot):
                        goalPoint = [1, 1]

                        errorX = goalPoint[0] - self.posX[i]
                        errorY = goalPoint[1] - self.posY[i]
                        normal = np.sqrt(errorX ** 2 + errorY ** 2)
                        errorX = errorX / normal
                        errorY = errorY / normal

                        vx = errorX
                        vy = errorY
                        vrTemp, vlTemp = self.feedbackLin(self.posTheta[i], vx, vy)
                        if normal > .1:
                            vr.append(vrTemp)
                            vl.append(vlTemp)
                        else:
                            vr.append(0)
                            vl.append(0)
                            print('close Enough')
                except Exception as e:
                    print(e)
                    print('Error Sending!')
                    pass
            for i in range(np.size(vr)):
                # Left wheel and then right wheel
                self.realRobotList[i].motors.set_wheel_motors(vl[i], vr[i])
                print('Robot' + str(i) + ': X-' + str(round(self.posX[i], 2)) + ' Y-' + str(
                    round(self.posY[i], 2)) + ' Theta-' + str(round(self.posTheta[i], 2)) + ' vl-' + str(round(vl[i],2)) + ' vr-'+ str(round(vr[i],2)))



    def callback0(self, data):
        self.posX[0] = data.transform.translation.x
        self.posY[0] = data.transform.translation.y
        self.posTheta[0] = self.euler_from_quaternion(data.transform.rotation.x, data.transform.rotation.y,
                                           data.transform.rotation.z, data.transform.rotation.w)
    def callback1(self, data1):
        self.posX[1] = data1.transform.translation.x
        self.posY[1] = data1.transform.translation.y
        self.posTheta[1] = self.euler_from_quaternion(data1.transform.rotation.x, data1.transform.rotation.y,
                                           data1.transform.rotation.z, data1.transform.rotation.w)

    def callback2(self, data1):
        self.posX[2] = data1.transform.translation.x
        self.posY[2] = data1.transform.translation.y
        self.posTheta[2] = self.euler_from_quaternion(data1.transform.rotation.x, data1.transform.rotation.y,
                                           data1.transform.rotation.z, data1.transform.rotation.w)

    def callback3(self, data1):
        self.posX[3] = data1.transform.translation.x
        self.posY[3] = data1.transform.translation.y
        self.posTheta[3] = self.euler_from_quaternion(data1.transform.rotation.x, data1.transform.rotation.y,
                                           data1.transform.rotation.z, data1.transform.rotation.w)
    def callback4(self, data1):
        self.posX[4] = data1.transform.translation.x
        self.posY[4] = data1.transform.translation.y
        self.posTheta[4] = self.euler_from_quaternion(data1.transform.rotation.x, data1.transform.rotation.y,
                                           data1.transform.rotation.z, data1.transform.rotation.w)

    def callback5(self, data1):
        self.posX[5] = data1.transform.translation.x
        self.posY[5] = data1.transform.translation.y
        self.posTheta[5] = self.euler_from_quaternion(data1.transform.rotation.x, data1.transform.rotation.y,
                                           data1.transform.rotation.z, data1.transform.rotation.w)



    def feedbackLin(self,theta,vx,vy):
        epsilon = .025
        l = .028
        maxSpeed = 400
        R = np.array([[np.cos(theta),np.sin(theta)],
                    [-np.sin(theta),np.cos(theta)]])
        dirVel = np.array([[vx],[vy]])
        Rot = np.array([[1,0],[0,1/epsilon]])
        commands = np.matmul(np.matmul(Rot, R),dirVel)
        if commands[1] != 0:
            r = commands[0]/commands[1]
        else:
            r = 0
        vr = int(np.floor(commands[1]*(r + l/2) *maxSpeed))
        vl = int(np.floor(commands[1]*(r - l/2) *maxSpeed))

        return vr, vl

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z  # in radians
if __name__ == '__main__':
    tpath = os.getcwd()
    my_dir = os.path.dirname(os.path.abspath(__file__))
    pickle_file_path = os.path.join(my_dir, 'PickleFiles', 'HadasSpec2.pkl')

    # Load pickle file
    with open(pickle_file_path, 'rb') as input:
        f = pickle.load(input)


    #Number of robots
    nRobot = 3
    r = runRobot(f)
    stop_threads = False

    try:
        r.listener()
    except rospy.ROSInterruptException as e:
        print(e)
        pass

    finally:
        # for i in range(nRobot):
        print('STOPPING ALL ROBOTS')
        stop_threads = True
        r.thread.join()
        time.sleep(1)
        for i in range(r.nRobot-1):
            r.realRobotList[i].motors.set_wheel_motors(0, 0)
        print('Done')