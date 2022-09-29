#!/usr/bin/python3
from NatNetClient import NatNetClient
from util import quaternion_to_euler
import numpy as np
import tkinter as tk
from tkinter import ttk
import tkinter.filedialog
from math import fabs, pi
from scipy.io import savemat
import stretch_body.robot
# import rospy
# import tf2_ros
# import tf
import parseEvBasedSTL
import initializeSpec
import prepBuchi
import checkForConflicts
import pickle
import time
import re
import os
import buchiFuncs
from getAllCommands import getAllCommands
from prepForCommands import getCMD
from makeSpecs import findInfo, handleUntil
from activateProp import activateProp
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import preparation as prep
import makeRoadmap
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import control
import helperFuncs
import forwardBuchi


class formData:
    def __init__(self,robot):
        self.robot = robot
        self.M = 2 #Number of Robots
        self.N = 4  # Number of Inputs
        self.P = 1 #Number of Humans/dynamic obstacles
        self.freq = 0 #Frequency of simulatiom
        self.maxV = 0 #Maximum velocity
        self.sizeState = 3
        self.sizeU = 5
        self.wheel2Center =.4
        self.offsetX = -.025
        self.armZero = .19
        self.offsetZ = 0.08
        self.running = True
        # Default spec location
        my_dir0 = os.path.dirname(os.path.abspath(__file__))
        my_dir2 = os.path.join(my_dir0, 'Specs', '')
        my_dir2 = os.path.join(my_dir0, 'Specs', '')
        my_dir3 = os.path.join(my_dir0, 'Maps', 'openMap.txt')
        my_dir4 = os.path.join(my_dir0, 'Maps', 'openNodes.txt')
        my_dir = os.path.join(my_dir0, 'Specs', 'ICRA2023Spec4.txt')
        my_dir5 = os.path.join(my_dir0, 'Maps', 'mapNodes.pkl')
        my_dir6 = os.path.join(my_dir0, 'Maps', 'mapNodeConnections.pkl')
        self.positions = {}
        self.rotations = {}
        self.objectPosition = {}

        #NRI ROUTE 1 DEFAULTS
        # xR = [objectX,objectY,objectZ,thetaOrient,dist2Obj,secondObjectX,secondObjectY, secondObjectZ, dist2Obj2]
        self.default = np.array(['1', '5', '0.2,0.2,0.05,0.07,12', '1.8,1.8,.342,0,0,0', '0,0,0.025,0,1,17,12,0,0,0,0'])
        #NRI ROUTE 2 DEFAULTS
        # self.default = np.array(['1', '5', '1.5,1.5,15', '0.12,67.55,0', '0.12,62,0'])
        #
        #
        # # 2 robots
        # self.default = np.array(
        #     ['2', '5', '1, 1 ,15,1, 1 ,15', '1.8,1.8,15,1.8,8,15', '1.8,1.8,15,1.8,8,15'])

        # 3 robots
        # self.default = np.array(
        #     ['3', '5', '.25, .25 ,15,.25, .25 ,15,.25, .25 ,15', '1.8,-1.25,15,-.7,-.3,15,-.7,.77,15', '-1.8,-1.25,0,1.9,0.3,0'])

        # 5 robots(physical)
        # self.default = np.array(
        #     ['5', '5', '.25, .25 ,15,.25, .25 ,15,.25, .25 ,15,.25, .25 ,15,.25, .25 ,15', '1.8,-1.25,15,-.7,-.3,15,-.7,.77,15,-.4,-.8,15,.4,-.8,15', '-1.8,-1.25,0,1.9,0.3,0,-1.35,-.8,0'])

        # 5 robots
        # self.default = np.array(
        #     ['5', '5', '.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15', '1.75,-1.25,15,-.6,1,0,-.6,-.4,0,-.3,-1,0,.3,-1,0', '-1.7,-2,0,1.75,.4,0,-1.5,.5,0,1.7,-.5,0'])

        # 7 robots
        # self.default = np.array(
        #     ['7', '5', '.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15',
        #      '1.75,-1.25,15,-.6,1,0,-.6,-.4,0,-.3,-1,0,.3,-1,0, 1.5, -.5,0,1.8,-.5,0', '-1.7,-2,0,1.75,.4,0,-1.5,.5,0,1.7,-.5,0'])
        #
        # # 9 robots
        # self.default = np.array(
        #     ['9', '5', '.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15',
        #      '1.75,-1.25,15,-.6,1,0,-.6,-.4,0,-.3,-1,0,.3,-1,0, 1.5, -.5,0,1.8,-.5,0,-1.5,-1,0,-1,-1,0', '-1.7,-2,0,1.75,.4,0,-1.5,.5,0'])


        self.filename1 = my_dir  # Directory of Specification
        self.filename2 = my_dir2
        self.filename3 = my_dir3
        self.filename4 = my_dir4
        self.filename5 = ''
        self.filename6 = ''
        try:
            import spot
            self.bypassbuchi = 0
        except:
            self.bypassbuchi = 1
        self.preFailure = 0
        self.State = [] #pre-allocate state
        self.Conflicts = [] #pre-allocate conflicts
        self.controllableProp = [] #pre-allocate controllable propositions
        self.initX = [] #pre-allocate initial position
        self.initXRef = [] #pre-allocate position od dynamic obstacles

    def upload1(self,master):
        self.filename1 = tk.filedialog.askopenfilename()
        tk.Label(master, text='                               ').grid(row=5, column =2)
        tk.Label(master, text=os.path.split(self.filename1)[-1]).grid(row=5, column =2)
    def upload2(self,master):
        self.filename2 = tk.filedialog.askopenfilename()
        tk.Label(master, text='                               ').grid(row=7, column =2)
        tk.Label(master, text=os.path.split(self.filename2)[-1]).grid(row=7, column =2)
    def upload3(self,master):
        self.filename3 = tk.filedialog.askopenfilename()
        tk.Label(master, text='                               ').grid(row=9, column =2)
        tk.Label(master, text=os.path.split(self.filename3)[-1]).grid(row=9, column =2)
    def upload4(self,master):
        self.filename4 = tk.filedialog.askopenfilename()
        tk.Label(master, text='                               ').grid(row=10, column =2)
        tk.Label(master, text=os.path.split(self.filename4)[-1]).grid(row=10, column =2)
    def upload5(self,master):
        self.filename5 = tk.filedialog.askopenfilename()
        tk.Label(master, text='                               ').grid(row=11, column =2)
        tk.Label(master, text=os.path.split(self.filename5)[-1]).grid(row=11, column =2)
    def upload6(self,master):
        self.filename6 = tk.filedialog.askopenfilename()
        tk.Label(master, text='                               ').grid(row=12, column =2)
        tk.Label(master, text=os.path.split(self.filename6)[-1]).grid(row=12, column =2)
    def enableBypass(self):
        self.bypassbuchi = 1
    def enablePreFailure(self):
        self.preFailure = 1
    def updateStatus(self,text1,master,status):
        #Change the status message
        text1.configure(state='normal')
        text1.insert(tk.END, status, 'text')
        text1.configure(state='disabled')

        master.update()
    def startButton(self):
        self.Start = 1

    def userParams(self,results):
        # Update the values based on what was given in the window
        self.M = int(results[0].get())
        self.freq = results[1].get()
        self.maxV = np.fromstring(results[2].get(), dtype=float, sep=',')
        self.initX = results[3].get()
        self.initX = self.initX.split(',')
        self.initX = [float(i) for i in self.initX]
        self.initX = np.array(self.initX)
        self.initXRef = results[4].get()
        self.initXRef = self.initXRef.split(',')
        self.sizeState = int(np.size(self.initX)/self.M)
        if self.initXRef[0] == '':
            self.initXRef = []
        else:
            self.initXRef = [float(i) for i in self.initXRef]
        self.initXRef = np.array(self.initXRef)
        self.name = re.split('\.',os.path.split(self.filename1)[-1])[0]
        self.P = int(np.floor(np.size(self.initXRef)/3))

    def userData(self, results):
        text1 = results[5]
        master = results[6]
        master.grid_columnconfigure(2, minsize=40)

        #Update the status of the automaton generation
        self.updateStatus(text1, master, '\nAbstracting Event-based STL Propositions...\n')

        #update the user parameters to reflect what was inputted
        self.userParams(results)

        #Read in the Event-Based STL formula
        x1 = open(self.filename1, 'r').readlines()
        self.STL_list = np.asarray(x1).reshape(-1, 1)

        psi = self.STL_list[0][0]
        prepTime = time.time()

        # Initialize Set of Buchis and potential states
        # Run prepare Spec
        Pi_mu, psi, inpRef, inpLabels,evProps = prep.prepSpec(psi,self.sizeState, self.sizeU)
        b_gamma = prep.getBuchi(Pi_mu,psi, self.filename2, text1, master, inpRef, inpLabels,evProps )
        self.psiRef = psi
        # b_gamma = prep.prepSpec(psi, self.filename2, text1, master,self.sizeState, self.sizeU,1)

        # Prepare roadmap
        filenames = [self.filename3, self.filename4, self.filename5, self.filename6]
        roadmap = makeRoadmap.mapInfo(filenames,text1,master)

        del self.CheckVar
        print('Total prep time: {}'.format(time.time()-prepTime))

        # Run specification
        self.CheckVar2 = tk.IntVar(value=self.preFailure)
        tk.Checkbutton(master, text='Pre-failure Warnings?', highlightbackground='orange', command=self.enablePreFailure,
                       variable=self.CheckVar2).grid(row=14,column=2,sticky=tk.W,pady=4)
        self.gamma = b_gamma
        specParams = [self.gamma, roadmap, master,text1]
        import signal
        signal.signal(signal.SIGINT, self.sigint_handler)
        tk.Button(master, text='Execute', command=(lambda e=specParams: self.runSpec(e))).grid(row=14, column=3, sticky=tk.W, pady=6)
        #master.destroy()

    def receive_rigid_body_frame(self, id, position, rotation_quaternion):
        # Position and rotation received
        if id == 21 and position != []:
            self.position = position
            self.position += (self.robot.arm.status['pos'],)
            self.position += (self.robot.lift.status['pos'],)
            # print(rotation_quaternion)
            # The rotation is in quaternion. We need to convert it to euler angles
            newRot = (rotation_quaternion[2],rotation_quaternion[0],rotation_quaternion[1],rotation_quaternion[3])
            rotx, roty, rotz = quaternion_to_euler(newRot)
            # Store the roll pitch and yaw angles
            self.rotations = (rotx, roty, rotz)
        if id == 30 and position != []:
            self.objectPosition = position

    def transform_to_pipi(self,input_angle):
        revolutions = int((input_angle + np.sign(input_angle) * pi) / (2 * pi))

        p1 = self.truncated_remainder(input_angle + np.sign(input_angle) * pi, 2 * pi)
        p2 = (np.sign(np.sign(input_angle)
                      + 2 * (np.sign(fabs((self.truncated_remainder(input_angle + pi, 2 * pi))
                                          / (2 * pi))) - 1))) * pi

        output_angle = p1 - p2

        return output_angle, revolutions

    def truncated_remainder(self, dividend, divisor):
        divided_number = dividend / divisor
        divided_number = \
            -int(-divided_number) if divided_number < 0 else int(divided_number)

        remainder = dividend - divisor * divided_number

        return remainder

    def runSpec(self, specParams):
        clientAddress = "199.168.1.72"
        optitrackServerAddress = "199.168.1.164"
        robot_id = 21

        # This will create a new NatNet client
        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.rigid_body_listener = self.receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        is_running = streaming_client.run()

        self.psi = self.STL_list[0][0]
        self.psinew = self.STL_list[0][0]
        self.master = specParams[2]
        self.specattr = [specParams[0]]
        self.roadmap = specParams[1]
        self.text1 = specParams[3]
        self.master.geometry("1400x700")
        if self.preFailure:
            self.updateStatus(self.text1, self.master, '\nPre-Failure Warnings:\n')
        self.pause = False
        tk.Button(self.master, text="Pause", command=self.pause_animation).grid(row=19, column=3, sticky=tk.W, pady=6)
        tk.Button(self.master, text="Resume", command=self.resume_animation).grid(row=20, column=3, sticky=tk.W, pady=6)

        #add an empty columns for space
        l0 = tk.Label(self.master, width=6, height=3)
        l0.grid(column=4, row=0)
        l02 = tk.Label(self.master, width=6, height=3)
        l02.grid(column=6, row=0)

        #add space for specification modification
        style = tk.ttk.Style()
        style.configure("TEntry", background='white')
        tk.Label(self.master, text="Ψ:").grid(row=19,column=4, padx=10, pady=25)
        self.PsiSTL = tk.Text(self.master,width=60, height=2,font=20)
        self.PsiSTL.grid(row=19, column=5, columnspan=4,sticky=tk.W)
        self.PsiSTL.insert(tk.END, self.psi)
        self.PsiSTL.configure(state='disabled')

        tk.Label(self.master, text="Ψ-new:").grid(row=20,column=4, padx=10, pady=25)
        self.PsiSTLnew = tk.Text(self.master,width=60, height=2,font=20)
        self.PsiSTLnew.grid(row=20, column=5, columnspan=4,sticky=tk.W)
        self.PsiSTLnew.insert(tk.END,self.psinew)
        butt = tk.ttk.Button(self.master, text='Modify', command=self.modifySpec)
        butt.grid(row=20, column=7, sticky=tk.W,pady=6)

        fig = plt.Figure(figsize=(5, 5),dpi=100)
        fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        ax = fig.add_subplot(111)
        self.timeText = ax.text(0, 0, '', transform=ax.transAxes)

        self.robots = {}
        self.robotArc = {}
        self.robotArm = {}

        self.cm = plt.get_cmap('jet')
        for i in range(f.M):
            xPoint = [self.initX[3*i]+self.wheel2Center*np.cos(self.initX[3*i+2]),self.initX[3*i]-self.wheel2Center*np.cos(self.initX[3*i+2])]
            yPoint = [self.initX[3*i+1]-self.wheel2Center*np.sin(self.initX[3*i+2]),self.initX[3*i+1]+self.wheel2Center*np.cos(self.initX[3*i+2])]

            self.robots[str(i)], = ax.plot(xPoint, yPoint, color="red" )
            arc_angles = np.linspace(self.initX[3*i+2], self.initX[3*i+1] + np.pi, 50)
            arc_xs = self.initX[3*i] + self.wheel2Center * np.cos(arc_angles)
            arc_ys = self.initX[3*i+1] + self.wheel2Center * np.sin(arc_angles)
            self.robotArc[str(i)], = ax.plot(arc_xs, arc_ys, color='red')
            armX = [self.initX[3*i],self.initX[3*i]]
            armY = [self.initX[3*i],self.initX[3*i]]
            self.robotArm[str(i)], = ax.plot(armX, armY )
            self.robotArm[str(i)].set_color(self.cm(40))


        plot1 = FigureCanvasTkAgg(fig, master=self.master)
        plot1.get_tk_widget().grid(column=5, row=0, rowspan=18)
        ax.axis('off')

        xwall = []
        ywall = []
        for i in range(np.size(self.roadmap.map, 0)):
            xwall.append(self.roadmap.map[i, 0])
            xwall.append(self.roadmap.map[i, 2])
            xwall.append(None)
            ywall.append(self.roadmap.map[i, 1])
            ywall.append(self.roadmap.map[i, 3])
            ywall.append(None)
        # plt.ion()
        ax.plot(xwall, ywall, color="black")
        ax.plot(self.initXRef[0], self.initXRef[1],'o', color="blue")
        self.x = self.initX
        self.xR = self.initXRef
        self.potS = [0]

        #Create input buttons if exist
        self.initCount = 10
        self.count = self.initCount #used to track the row of button
        self.buttons = []
        for i in range(np.size(self.specattr[0].inpLabels,0)):
            buttonLabel = self.specattr[0].inpLabels[i][1]
            #0 in dToSend represents the spec to plot
            dToSend = [0,buttonLabel,self.count]
            butt = tk.Button(self.master, text=buttonLabel + ' (False)', fg='red',
                      command=(lambda e=dToSend: self.changeInput(e)))

            butt.grid(row=self.count, column=7, sticky=tk.W,pady=6)
            self.buttons.append(butt)
            self.count+=1

        ani = FuncAnimation(fig, self.animate, self.animation_data, interval=1,
                                      repeat=True)
        tk.mainloop()

    def animation_data(self):
        self.t = 0
        time.sleep(1)
        while self.running:
            if not self.pause:
                print('--------------------------------------------------------------')
                angle = self.transform_to_pipi((np.pi/180)*(self.rotations[2])+ pi/2)[0]
                self.x[0]= self.position[2]
                self.x[1] = self.position[0]
                self.x[2] = angle
                self.x[3] = self.position[3]
                self.x[4] = self.position[4]
                self.x[5] = self.robot.end_of_arm.status['stretch_gripper']['pos']
                self.xR[0] = self.objectPosition[2]
                self.xR[1] = self.objectPosition[0]
                self.xR[2] = self.objectPosition[1] - self.offsetZ
                #depot position
                self.xR[5] = 0
                self.xR[6] = 0
                self.xR[7] = .15

                # self.robot.end_of_arm.move_to('stretch_gripper', 30)

                print('X: {}, Y: {}, Theta: {}, D: {}, Z: {}, Grip: {}'.format(round(self.x[0],2),round(self.x[1],2),round(self.x[2],2),round(self.x[3],2),round(self.x[4],2),round(self.x[5],2)))
                print('objectX: {}, objectY: {}, objectY: {}'.format(round(self.xR[0],2),round(self.xR[1],2),round(self.xR[2],2)))
                # print(self.rotations)
                theTime = time.time()
                '''fse.f
                This Simulates the process 
                '''
                self.checkInputs()
                self.updateRef()
                nom, self.specattr = control.synthesis(self.specattr, self.potS, self.roadmap, self.x, self.xR, self.t, self.maxV, self.sizeState,self.sizeU, self.preFailure, self.text1, self.master)
                # self.specattr, self.potS = forwardBuchi.forward(self.specattr,self.potS)
                v = np.zeros((1, int(np.size(self.x)/self.sizeState)))
                omega = np.zeros((1, int(np.size(self.x)/self.sizeState)))
                deltaD = np.zeros((1, int(np.size(self.x)/self.sizeState)))
                deltaZ = np.zeros((1, int(np.size(self.x)/self.sizeState)))
                deltaGrip = np.zeros((1, int(np.size(self.x) / self.sizeState)))

                for i in range(int(np.size(self.x)/self.sizeState)):
                    v[0, i] = nom[0][3 * i]
                    omega[0, i] = nom[0][3 * i + 1]
                    deltaD[0,i] = nom[0][3*i+2]
                    deltaZ[0,i] = nom[0][3*i+3]
                    deltaGrip[0,i] = nom[0][3*i+4]
                time.sleep(.02)
                '''
                End of process
                '''
                elapsedT = time.time()-theTime
                self.t += elapsedT
                formattedTime = "Time: {}s".format(round(self.t,2))
                self.timeText.set_text(formattedTime)
                loopTime = elapsedT

                for i in range(int(self.M)):
                    d = v[0][i]
                    phi = omega[0][i]
                    vD = deltaD[0][i]
                    vZ = deltaZ[0][i]
                    vGrip = deltaGrip[0][i]
                    print('Vd: {}, vOmega: {}, vD: {},vZ: {}, vGrip: {}'.format(d,phi,vD,vZ,vGrip))
                    self.robot.base.set_velocity(v_m=d, w_r=phi)
                    self.robot.arm.set_velocity(v_m=vD)
                    self.robot.lift.set_velocity(v_m=vZ)
                    self.robot.end_of_arm.move_by('stretch_gripper', vGrip)

                    self.robot.push_command()
                    newPos = helperFuncs.integrateOdom([d,phi],self.x[3*i:3*i+3])
                    # self.x[3 * i:3 * i + 3] = newPos
                    # self.x[3*i+3] += deltaD[0][i] * loopTime
                    # self.x[3*i+4] += deltaZ[0][i] * loopTime

            yield self.x

    def animate(self,animation_data):
        x = animation_data
        for i in range(len(self.robots)):
            point1 = helperFuncs.robot2global(x[3*i:3*i+3],[0,self.wheel2Center])
            point2 = helperFuncs.robot2global(x[3*i:3*i+3],[0,-self.wheel2Center])
            xPoint = [point1[0],point2[0]]
            yPoint = [point1[1],point2[1]]
            self.robots[str(i)].set_data(xPoint, yPoint)
            arc_angles = np.linspace(x[3*i+2] + np.pi/2, x[3*i+2] + 3*np.pi/2, 50)
            arc_xs = x[3*i] + self.wheel2Center * np.cos(arc_angles)
            arc_ys = x[3*i+1] + self.wheel2Center * np.sin(arc_angles)
            self.robotArc[str(i)].set_data(arc_xs, arc_ys)
            armpoint1 = helperFuncs.robot2global(x[3*i:3*i+3],[-self.offsetX,0])
            armpoint2 = helperFuncs.robot2global(x[3*i:3*i+3],[-self.offsetX,-x[3*i+3]])
            armX = [armpoint1[0],armpoint2[0]]
            armY = [armpoint1[1],armpoint2[1]]
            self.robotArm[str(i)].set_data(armX, armY)
            self.robotArm[str(i)].set_color(self.cm(round(x[self.sizeU*i+4]*232)))

        return self.robots

    def pause_animation(self):
        self.pause = True

    def resume_animation(self):
        self.pause = False

    def changeInput(self,inps):
        # labName = inps[1]
        spec = inps[0]
        evProps = self.specattr[spec].evProps
        propName = inps[1]
        butt = self.buttons[inps[2]-self.initCount]
        if (eval('evProps.'+propName)):
            exec('evProps.' + propName + ' = 0')
            butt.config(text=propName + '(False)',fg='red')
        else:
            exec('evProps.' + propName + ' = 1')
            butt.config(text=propName + '(True)',fg='green')

        self.specattr[spec].evProps = evProps

    def checkInputs(self):
        for i in range(np.size(self.specattr)):
            props = self.specattr[i].evProps
            x = self.x
            xR = self.xR
            for j in range(np.size(self.specattr[i].inpRef,0)):
                mess = self.specattr[i].inpRef[j][1]
                if eval(mess):
                    self.specattr[i].input[2 * j] = 1
                    if self.specattr[i].input[2 * j + 1] == 0 or self.specattr[i].input[2 * j + 1] > self.t:
                        self.specattr[i].input[2 * j + 1] = self.t
                else:
                    self.specattr[i].input[2 * j] = 0
        # Check to see what if inputs have changed

    def updateRef(self):
        # xR = [objectX,objectY,objectZ,thetaOrient,dist2Obj,secondObjectX,secondObjectY, secondObjectZ, dist2Obj2]
        # xy is on front face of robot. arm is in offsetX
        centroidPoint = helperFuncs.robot2global(self.x[0:3], [-self.offsetX, 0])

        self.xR[3] = np.arctan2(self.xR[1]-centroidPoint[1],self.xR[0]-centroidPoint[0]) + np.pi/2
        self.xR[3] = self.transform_to_pipi(self.xR[3])[0]
        #print(self.xR[3])
        self.xR[4] = np.sqrt((self.x[0]-self.xR[0])**2 + (self.x[1]-self.xR[1])**2) - self.armZero
        self.xR[8] = np.sqrt((self.x[0]-self.xR[5])**2 + (self.x[1]-self.xR[6])**2) - self.armZero

    def modifySpec(self):
        newSTL = self.PsiSTLnew.get("1.0",tk.END)
        newSTL = re.sub('\\n','',newSTL)
        if self.psi != newSTL:
            print('Change in Psi')
            newPreds,psiNew,inpRef,inpLabels,evProps = prep.prepSpec(newSTL, self.sizeState, self.sizeU)
            oldPreds = self.specattr[0].Pi_mu
            for i in range(np.size(oldPreds)):
                if oldPreds[i].a != newPreds[i].a or oldPreds[i].b != newPreds[i].b:
                    print('timing bound changed for {}'.format(i))
                    self.specattr[0].Pi_mu[i].a = newPreds[i].a
                    self.specattr[0].Pi_mu[i].b = newPreds[i].b
                    self.specattr[0].Pi_mu[i].hxte = []
                if oldPreds[i].hxt != newPreds[i].hxt:
                    print('predicate changed')
                    self.specattr[0].Pi_mu[i].hxte = []
                    self.specattr[0].Pi_mu[i].dir = newPreds[i].dir
                    self.specattr[0].Pi_mu[i].hxt = newPreds[i].hxt
                    self.specattr[0].Pi_mu[i].point = newPreds[i].point
                    self.specattr[0].Pi_mu[i].pred = newPreds[i].pred
                    self.specattr[0].Pi_mu[i].signFS = newPreds[i].signFS
            if np.size(newPreds) != np.size(oldPreds):
                psiToAdd = psiNew[len(self.psiRef):]
                gamma = re.findall('(\&|\|)',psiToAdd)[0]
                parenMatches = self.find_parens(psiToAdd)
                firstPar = psiToAdd.index('(')
                lastPar = parenMatches[firstPar]
                newSpec = psiToAdd[firstPar:lastPar+1]
                if 'G' in psiToAdd[0:firstPar]:
                    newSpec = 'G'+newSpec
                propositions = re.findall('(?<=\.)\w*', newSpec)
                Pi_mu = []
                for i in range(np.size(newPreds)):
                    if re.split('\.',newPreds[i].prop_label)[-1] in propositions:
                        Pi_mu.append(newPreds[i])
                inpRefNew = []
                inpLabelsNew = []
                for i in range(np.size(inpRef,0)):
                    if inpRef[i][0] in newSpec:
                        inpRefNew.append(inpRef[i])
                for i in range(np.size(inpLabels,0)):
                    if inpLabels[i][0] in newSpec:
                        inpLabelsNew.append(inpLabels[i])
                b_gamma = prep.getBuchi(Pi_mu, newSpec, '', '', '', inpRef, inpLabels, evProps)
                inpRefAdd = []
                inpLabelsAdd = []
                for j in range(np.size(propositions)):
                    if any(substring in inpRef[j][0] for substring in propositions):
                        inpRefAdd.append(inpRef[j])
                    if any(substring in inpLabels[j][0] for substring in propositions):
                        inpLabelsAdd.append(inpRef[j])
                b_gamma.inpRef = inpRefAdd
                b_gamma.inpLabels = inpLabelsAdd
                if gamma == '&':
                    # Generate Buchi Intersect
                    for i in range(np.size(self.specattr)):
                        self.specattr[i] = buchiFuncs.buchiIntersect(self.specattr[i],self.potS[i], b_gamma,0)
                else:
                    # Add buchi to specattr
                    self.specattr.append(b_gamma)
                    self.potS.append(0)

                for j in range(np.size(inpLabelsNew, 0)):
                    buttonLabel = inpLabelsNew[j][1]
                    # idx = [i for i, sublist in enumerate(self.specattr[i].inpLabels) if buttonLabel in sublist]
                    # 0 in dToSend represents the spec to plot
                    dToSend = [i, buttonLabel, self.count]
                    butt = tk.Button(self.master, text=buttonLabel + ' (False)', fg='red',
                                     command=(lambda e=dToSend: self.changeInput(e)))
                    butt.grid(row=self.count, column=7, sticky=tk.W, pady=6)
                    self.buttons.append(butt)
                    self.count += 1
                print('new specification added')
            self.PsiSTL.configure(state='normal')
            self.PsiSTL.delete("end-1l", "end")
            self.PsiSTL.insert(tk.END, newSTL, 'text')
            self.PsiSTL.configure(state='disabled')

    def makeForm(self):
        #First make the form and create the few entries that are required for initialization
        master = tk.Tk()
        self.master = master
        master.geometry("750x600")
        tk.Label(master, text="Number of Robots").grid(row=0,column=0)
        tk.Label(master, text="Frequency (Hz)").grid(row=1)
        tk.Label(master, text="Max Velocity (x_i, y_i, theta_i,...)").grid(row=2)
        tk.Label(master, text="Init state (x_i, y_i, theta_i,...)").grid(row=3)
        tk.Label(master, text="Init uncontrolled state").grid(row=4)
        tk.Label(master, text="Upload Event-based STL Specification").grid(row=5)
        tk.Label(master, text="Bypass Buchi Generation?").grid(row=6)
        tk.Label(master, text="Directly Upload Buchi").grid(row=7)
        tk.ttk.Separator(master, orient='horizontal').grid(column=0, row=8, columnspan=3, sticky='ew')

        tk.Label(master, text="OPTIONAL").grid(row=8)
        tk.Label(master, text="Upload Map").grid(row=9)
        tk.Label(master, text="Upload roadmap").grid(row=10)
        tk.Label(master, text="Upload node graph").grid(row=11)
        tk.Label(master, text="Upload map paths").grid(row=12)
        tk.ttk.Separator(master, orient='horizontal').grid(column=0, row=13, columnspan=3, sticky='new')
        style = tk.ttk.Style()
        style.configure("TEntry", background='white')
        e1 = tk.ttk.Entry(master,width=5)
        e2 = tk.ttk.Entry(master,width=5)
        e3 = tk.ttk.Entry(master,width=30)
        e4 = tk.ttk.Entry(master,width=30)
        e5 = tk.ttk.Entry(master,width=30)
        # I configure it orange because there are display issues on a mac
        # e1.config(bg="orange")
        # e2.config(bg="orange")
        # e3.config(bg="orange")
        # e4.config(bg="orange")
        # e5.config(bg="orange")
        # e6.config(bg="orange")
        # e7.config(bg="orange")
        # e8.config(bg="orange")
        # e9.config(bg="orange")
        e1.grid(row=0, column=1, columnspan=2,sticky=tk.W)
        e2.grid(row=1, column=1, columnspan=2,sticky=tk.W)
        e3.grid(row=2, column=1, columnspan=2,sticky=tk.W)
        e4.grid(row=3, column=1, columnspan=2,sticky=tk.W)
        e5.grid(row=4, column=1, columnspan=2,sticky=tk.W)
        # Place default values. This is useful for quick entering
        e1.insert(10, self.default[0])
        e2.insert(10, self.default[1])
        e3.insert(10, self.default[2])
        e4.insert(10, self.default[3])
        e5.insert(10, self.default[4])



        #create status window to show updates during compilation
        text1 = tk.Text(master, height=15, width=75)
        scroll = tk.Scrollbar(master, command=text1.yview)
        text1.configure(yscrollcommand=scroll.set)
        text1.grid(row = 18, column=0, rowspan=12, columnspan=3)
        text1.tag_configure('Title', font=('Arial', 14, 'bold'))
        text1.tag_configure('text', font=('Arial', 12, 'bold'))
        text1.configure(state='normal')
        text1.insert(tk.END, '\nStatus Updates\n','Title')
        text1.insert(tk.END, '\nWaiting for parameters...', 'text')
        text1.configure(state='disabled')

        #Create buttons and start filling out form
        tk.Button(master, text='Upload',
                  command=(lambda e=master: self.upload1(e))).grid(row=5,column=1, sticky=tk.W, pady=4)
        tk.Label(master, text=os.path.split(self.filename1)[-1]).grid(row=5, column =2, sticky=tk.W)

        tk.Button(master, text='Upload',
                  command=(lambda e=master: self.upload2(e))).grid(row=7,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename2)[-1]).grid(row=7, column=2, sticky=tk.W)

        tk.Button(master, text='Upload',
                  command=(lambda e=master: self.upload3(e))).grid(row=9,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename3)[-1]).grid(row=9, column=2, sticky=tk.W)

        tk.Button(master, text='Upload',
                  command=(lambda e=master: self.upload4(e))).grid(row=10,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename4)[-1]).grid(row=10, column=2, sticky=tk.W)

        tk.Button(master, text='Upload',
                  command=(lambda e=master: self.upload5(e))).grid(row=11,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename5)[-1]).grid(row=11, column=2, sticky=tk.W)

        tk.Button(master, text='Upload',
                  command=(lambda e=master: self.upload6(e))).grid(row=12,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename6)[-1]).grid(row=12, column=2, sticky=tk.W)

        self.CheckVar = tk.IntVar(value=self.bypassbuchi)
        tk.Checkbutton(master, text='Bypass?', highlightbackground='orange', command=self.enableBypass,
                       variable=self.CheckVar).grid(row=6,column=1,sticky=tk.W,pady=4)

        tk.Button(master, text='Quit', highlightbackground='orange', command=master.destroy).grid(row=14,
                                            column=0,sticky=tk.W,pady=4)
        results = np.array([e1, e2, e3,e4,e5,text1,master])

        tk.Button(master, text='Apply',
                  command=(lambda e=results: self.userData(e))).grid(row=14, column=1, sticky=tk.W, pady=6)

    def find_accepting_states(self,dot_str):
        '''
        to be an accepting state: 'peripheries=2' and there exists transition state -> state
        '''
        states = []
        for line in dot_str.split('\n'):
            if line.find('peripheries=2') != -1:
                s = line.split()[0]
                states.append(s)
        return states

    def find_parens(self,s):
        toret = {}
        pstack = []

        for i, c in enumerate(s):
            if c == '(':
                pstack.append(i)
            elif c == ')':
                if len(pstack) == 0:
                    raise IndexError("No matching closing parens at: " + str(i))
                toret[pstack.pop()] = i

        if len(pstack) > 0:
            raise IndexError("No matching opening parens at: " + str(pstack.pop()))

        return toret

    def sigint_handler(self,sig,frame):
        print('made it to close')
        self.running = False
        self.master.destroy()

if __name__ == "__main__":
    loadOnStart = 0
    robot = stretch_body.robot.Robot()
    robot.startup()
    robot.end_of_arm.move_to('wrist_yaw', 0)
    robot.end_of_arm.move_to('stretch_gripper', 30)
    robot.lift.move_to(1)
    robot.arm.move_to(0)
    robot.push_command()

    try:
        f = formData(robot)
        f.makeForm()
        tk.mainloop()
    except:
        pass
    finally:
        # Stop the robot
        print('Stopping robot')
        robot.base.set_velocity(v_m=0, w_r=0)
        robot.push_command()
        robot.end_of_arm.move_to('stretch_gripper', 25)

        time.sleep(2)
        robot.base.set_velocity(v_m=0, w_r=0)
        robot.push_command()
        print('robot stopped!')

    print('finished')

