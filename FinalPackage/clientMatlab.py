import socket
import time
from prepForCommands import getCMD
from runEvBasedSTL import formData
import pickle
import numpy as np
import os

#Find path of pickle file
tpath = os.getcwd()
my_dir = os.path.dirname(os.path.abspath(__file__))
pickle_file_path = os.path.join(my_dir, 'Pickle Files', 'specTest.pkl')

#Load pickle file
with open(pickle_file_path, 'rb') as input:
    f = pickle.load(input)
Headersize = 75
sendSize = 500
conn = 0
nomsg = 1

# Initialize the python client
try:
    s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address2 = ('localhost', 8120)
    s2.bind(server_address2)
    s2.listen(5)
except:
    print("Client could not initialize")

try:
    while True:
        full_msg = ''
        new_msg = True

        while True:
            while nomsg == 1:
                try:
                    #Try to connect to Matlab and recieve the initial message
                    print("Attempting to establish connection to Matlab")
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    server_address = ('localhost', 8052)
                    s.settimeout(2)
                    s.connect(server_address)
                    msg = s.recv(1024)
                    #Parse the initial message recieved from matlab
                    splitIntro = msg.split()
                    f.M = int(splitIntro[0])
                    n_inputs = int(splitIntro[1])
                    n_people = int(splitIntro[2])
                    nomsg = 0
                    print("Connection Established!")
                    #Initialize based on first message
                    posX = np.zeros((1, f.M))
                    posY = np.zeros((1, f.M))
                    posTheta = np.zeros((1, f.M))
                    posxinit = np.zeros((1, f.M))
                    posyinit = np.zeros((1, f.M))
                    posthetainit = np.zeros((1, f.M))
                    posXPerson = np.zeros((1,n_people))
                    posYPerson = np.zeros((1, n_people))
                    posThetaPerson = np.zeros((1, n_people))
                    input = np.zeros((1, n_inputs*2), dtype=float)

                except:
                    # IF you cant connect, wait a second and try again
                    print("Connection not established. Waiting for Matlab server...")
                    time.sleep(1)
            try:
                msg = s.recv(1024)
            except:
                nomsg = 1

            try:
                msglen = int(msg[:Headersize])
                full_msg += msg.decode("utf-8")
            except:
                full_msg = ''
                msglen = 0
            new_msg = False


            if len(full_msg) > 350:
                full_msg = ''

            if len(full_msg)-Headersize == msglen:
                new_msg = True

                full_msg = full_msg[Headersize:]
                parsedMes = full_msg.split()


                for i in range(f.M):
                    posX[0,i] = float(parsedMes[3*i])
                    posY[0,i] = float(parsedMes[3*i+1])
                    posTheta[0,i] = float(parsedMes[3*i+2])
                for i in range(f.M):
                    posxinit[0,i] = float(parsedMes[3*i + 3*f.M])
                    posyinit[0,i] = float(parsedMes[3*i + 1 + 3*f.M])
                    posthetainit[0,i] = float(parsedMes[3*i+ 2 + 3*f.M])
                for i in range(n_people):
                    posXPerson[0,i] = float(parsedMes[3*i + 6*f.M])
                    posYPerson[0,i] = float(parsedMes[3*i + 1 + 6*f.M])
                    posThetaPerson[0,i] = float(parsedMes[3*i + 2 + 6*f.M])

                currTime = float(parsedMes[6*f.M + 3*n_people])
                startTime = float(parsedMes[6*f.M + 1 + 3*n_people])
                currState = int(parsedMes[6*f.M + 2 + 3*n_people])

                input = np.asarray(list(map(float, parsedMes[6 * f.M + 3 + 3 * n_people:])))

                t = time.time()

                full_msg = ''
                unt = 1
                vx, vy, vtheta, currState, distTotal, newinput,unt = getCMD(f, posX[0], posY[0], posTheta[0],
                                                                            posxinit[0], posyinit[0], posthetainit[0],
                                                                            posXPerson[0], posYPerson[0],
                                                                            posThetaPerson[0], currTime, startTime,
                                                                            currState, input, unt)

                elapsed = time.time() - t
                #print(elapsed)
                msg = ""
                for i in range(f.M):
                    msg = msg + str(vx[0,i]) + " "
                    msg = msg + str(vy[0,i]) + " "
                    msg = msg + str(vtheta[0,i]) + " "

                msg = msg + str(currState) + " "

                for i in range(np.size(newinput)):
                    msg = msg + " " + str(newinput[i])
                print(msg)
                msglen = len(msg)
                padL = ' '.ljust(sendSize-msglen-1)
                msg = f"{msg} {padL}"

                s.send(bytes(msg, "utf-8"))
                full_msg = ''
                print('__________________________________________________________________')

except Exception as exception:
    assert type(exception).__name__ == 'NameError'
    assert exception.__class__.__name__ == 'NameError'
    assert exception.__class__.__qualname__ == 'NameError'
    s.close()
    print("Process Terminated")
