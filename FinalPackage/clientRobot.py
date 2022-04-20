import time, socket, json, pickle
import cv2
import os
import numpy as np
from runEvBasedSTL import formData
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import math
import socket, struct, threading
from prepForCommands import getCMD

class runRobot:
    def __init__(self,f):
        self.currState = 0
        self.currTime = 0
        self.robotsUsed = [1]
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


    def sendObject(self, message):
        # local host IP '127.0.0.1'
        #host = '10.49.33.92'
        host = '192.168.0.148'
        # Define the port on which you want to connect
        port = 65432
        s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        # connect to server on local computer

        s.connect((host,port))
        s.send(pickle.dumps(message))
        # s.send(json.dumps(message).encode())
        s.close()

    def callback0(self,data):
        self.posX[0] = data.pose.position.x
        self.posY[0] = data.pose.position.y
        self.posTheta[0] = self.euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y,
                                                          data.pose.orientation.z, data.pose.orientation.w)

    def callback1(self,data):
        self.posPX[0] = data.pose.position.x
        self.posPY[0] = data.pose.position.y
        self.posPTheta[0] = self.euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y,
                                                          data.pose.orientation.z, data.pose.orientation.w)

    def listener(self):
        rospy.init_node('listener', anonymous=True)

#        rospy.Subscriber('odom', String, callback, queue_size=1)
        rospy.Subscriber('/mocap_node/Robot_2/pose', PoseStamped, self.callback0, queue_size=1)
        rospy.Subscriber('/mocap_node/Robot_1/pose', PoseStamped, self.callback1, queue_size=1)

        print('Starting Thread')

        self.thread = threading.Thread(target=self.sendCMD)
        self.thread.daemon = True
        self.thread.start()

        rospy.spin()

    def sendCMD(self):
        time.sleep(3)
        startTime = time.time()
        runTime = 0
        triggered = 0
        computeTime = []
        while True:
            # Stop the Thread if script closed
            global stop_threads
            if stop_threads:
                break
            if runTime > 50:
                avgCompute = np.sum(computeTime)/np.size(computeTime)
                print('average compute time: ' + str(avgCompute))
                stop_threads = True
                a = kd
                break

            STL = 1
            if STL:
                try:
                    cTime = time.time()
                    vx, vy, vt, self.currState, dis, newInput, unt = getCMD(self.f, self.posX, self.posY, self.posTheta,
                                                                                     self.posXinit, self.posYinit,
                                                                                     self.posThetainit, self.posPX,
                                                                                     self.posPY, self.posPTheta,
                                                                                     runTime, 0, self.currState,
                                                                                     self.input, [])
                    totalCTime = time.time()-cTime
                    computeTime.append(totalCTime)
                except:
                    vx = [[0]]
                    vy = [[0]]
                runTime = time.time()-startTime
                print('Time: {}s'.format(str(round(runTime, 2))))
                vx = vx[0]
                vy = vy[0]

                uncontDist = np.sqrt((self.posPX[0] - self.posX[0])**2 + (self.posPY[0]- self.posY[0])**2)
                if uncontDist > 3 and triggered == 0:
                    self.input[2] = 1
                    self.input[3] = runTime
                    triggered = 1
                if uncontDist < 2 and triggered == 1:
                    self.input[2] = 0
                    self.input[3] = runTime
                    triggered = 0
                if runTime > 25:
                    self.input[0] = 1
                    self.input[1] = 25


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

                for i in range(np.size(vr)):
                    # Left wheel and then right wheel
                    print('Robot' + str(i) + ': X-' + str(round(self.posX[i], 2)) + ' Y-' + str(
                        round(self.posY[i], 2)) + ' Theta-' + str(round(self.posTheta[i], 2)) + ' vx:' + str(
                        round(vx[i], 2)) + ' vy:' + str(round(vy[i], 2)))
                command = {
                "id" : int(time.time()),
                "cmd" : "SetSpeedCommand",
                "priority" : 1,#, Priority, type int
                "leftSpeed" : -vl[0], ## Left speed value, type int
                "rightSpeed" : vr[0], ## Right speed value, type int
                "receivingPort" : 28200
                }

                self.sendObject(command)
                time.sleep(.2)
                # command = {
                #     "id": int(time.time()),
                #     "cmd": "SetSpeedCommand",
                #     "priority": 1,  # , Priority, type int
                #     "leftSpeed": 0,  ## Left speed value, type int
                #     "rightSpeed": 0,  ## Right speed value, type int
                #     "receivingPort": 28200
                # }
                #
                # self.sendObject(command)
                # time.sleep(.2)
            else:
                goalPoint = [self.posX[1], self.posY[1]]

                errorX = goalPoint[0]-self.posX[0]
                errorY = goalPoint[1]-self.posY[0]
                normal = np.sqrt(errorX**2 + errorY**2)
                errorX = errorX/normal
                errorY = errorY/normal

                vx = errorX
                vy = errorY

                vr, vl = self.feedbackLin(self.posTheta[0],vx,vy)
                if normal < .25:
                    vr = 0
                    vl = 0
                    print('close Enough')
                else:
                    msg = 'vr: ' + str(vr) + ', vl: ' + str(vl)
                    print(msg)

                command = {
                "id" : int(time.time()),
                "cmd" : "SetSpeedCommand",
                "priority" : 1,#, Priority, type int
                "leftSpeed" : -vl, ## Left speed value, type int
                "rightSpeed" : vr, ## Right speed value, type int
                "receivingPort" : 28200
                }

                # self.sendObject(command)
                # command = {
                # "id" : int(time.time()),
                # "cmd" : "SetSpeedCommand",
                # "priority" : 1,#, Priority, type int
                # "leftSpeed" : 0, ## Left speed value, type int
                # "rightSpeed" : 0, ## Right speed value, type int
                # "receivingPort" : 28200
                # }
                time.sleep(.3)
                self.sendObject(command)
        else:
            command = {
            "id" : int(time.time()),
            "cmd" : "SetSpeedCommand",
            "priority" : 1,#, Priority, type int
            "leftSpeed" : 0, ## Left speed value, type int
            "rightSpeed" : 0, ## Right speed value, type int
            "receivingPort" : 28200
            }
            self.sendObject(command)

    def feedbackLin(self,theta,vx,vy):
        epsilon = .04
        l = .155
        maxSpeed = 500
        R = np.array([[np.cos(theta),np.sin(theta)],
                    [-np.sin(theta),np.cos(theta)]])
        dirVel = np.array([[vx],[vy]])
        Rot = np.array([[1,0],[0,1/epsilon]])
        commands = np.matmul(np.matmul(Rot, R),dirVel)
        r = commands[0]/commands[1]
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
if __name__ == "__main__":
    # negative left speed goes forward
    # right speed positive forward
    tpath = os.getcwd()
    my_dir = os.path.dirname(os.path.abspath(__file__))
    pickle_file_path = os.path.join(my_dir, 'PickleFiles', 'robotDemo.pkl')

    # Load pickle file
    with open(pickle_file_path, 'rb') as input:
        f = pickle.load(input)

    r = runRobot(f)
    stop_threads = False

    try:
        r.listener()
    except rospy.ROSInterruptException as e:
        print(e)
        pass
    finally:
        stop_threads = True
        r.thread.join()

        command = {
        "id" : int(time.time()),
        "cmd" : "SetSpeedCommand",
        "priority" : 1,#, Priority, type int
        "leftSpeed" : 0, ## Left speed value, type int
        "rightSpeed" : -0, ## Right speed value, type int
        "receivingPort" : 28200
        }
        for i in range(4):
            r.sendObject(command)
        print('STOPPING')
