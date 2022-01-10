import time, socket, json, pickle
import cv2

import numpy as np
import time
import rospy
from std_msgs.msg import String


class runRobot:
    def __init__(self):
        self.currState = 0
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

    def receiveResponse(self,port):
        s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("",port))
        s.listen(5)
        connection, address = s.accept()
        data = []
        while True:
            packet = connection.recv(4096)
            if not packet: break
            data.append(packet)
        s.close()
        data_arr = pickle.loads(b"".join(data))
        # print(data_arr)

    def callback(self,data):
        sendCMD(data.data)
        #time.sleep(.4)

    def listener(self,f):
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber('odom', String, callback, queue_size=1)

        rospy.spin()

    def sendCMD(self,data):
        pos = str(data).split(',')
        if pos[0] != '0.0':
            posX = float(pos[0])
            posY = float(pos[1])
            posTheta = float(pos[2])
            print(posX,posY,posTheta)
            STL = 1
            if STL:
                vx, vy, vtheta, currState, distTotal, newinput, unt = getCMD(f, posX[0], posY[0], posTheta[0],
                                                                             posxinit[0], posyinit[0], posthetainit[0],
                                                                             posXPerson[0], posYPerson[0],
                                                                             posThetaPerson[0], currTime, startTime,
                                                                             currState, input, unt)
            else:
                goalPoint = [1,-2]

                errorX = goalPoint[0]-posX
                errorY = goalPoint[1]-posY
                normal = np.sqrt(errorX**2 + errorY**2)
                errorX = errorX/normal
                errorY = errorY/normal

                vx = errorX
                vy = errorY

            vr, vl = feedbackLin(posX,posY,posTheta,vx,vy)
            if normal < .1:
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

            sendObject(command)
            command = {
            "id" : int(time.time()),
            "cmd" : "SetSpeedCommand",
            "priority" : 1,#, Priority, type int
            "leftSpeed" : 0, ## Left speed value, type int
            "rightSpeed" : 0, ## Right speed value, type int
            "receivingPort" : 28200
            }
            time.sleep(.3)
            sendObject(command)
        else:
            command = {
            "id" : int(time.time()),
            "cmd" : "SetSpeedCommand",
            "priority" : 1,#, Priority, type int
            "leftSpeed" : 0, ## Left speed value, type int
            "rightSpeed" : 0, ## Right speed value, type int
            "receivingPort" : 28200
            }
            sendObject(command)

    def feedbackLin(self,x,y,theta,vx,vy):
        epsilon = .04
        l = .155
        maxSpeed = 300
        R = np.array([[np.cos(theta),np.sin(theta)],
                    [-np.sin(theta),np.cos(theta)]])
        dirVel = np.array([[vx],[vy]])
        Rot = np.array([[1,0],[0,1/epsilon]])
        commands = np.matmul(np.matmul(Rot, R),dirVel)
        r = commands[0]/commands[1]
        vr = int(np.floor(commands[1]*(r + l/2) *maxSpeed))
        vl = int(np.floor(commands[1]*(r - l/2) *maxSpeed))

        return vr, vl
if __name__ == "__main__":
    # negative left speed goes forward
    # right speed positive forward
    tpath = os.getcwd()
    my_dir = os.path.dirname(os.path.abspath(__file__))
    pickle_file_path = os.path.join(my_dir, 'Pickle Files', 'NRIRobot.pkl')

    # Load pickle file
    with open(pickle_file_path, 'rb') as input:
        f = pickle.load(input)


    try:
        time.sleep(3)
        runRobot.listener(f)
    except rospy.ROSInterruptException:
        pass
    finally:
        command = {
        "id" : int(time.time()),
        "cmd" : "SetSpeedCommand",
        "priority" : 1,#, Priority, type int
        "leftSpeed" : 0, ## Left speed value, type int
        "rightSpeed" : -0, ## Right speed value, type int
        "receivingPort" : 28200
        }
        for i in range(4):
            sendObject(command)
        print('STOPPING')
