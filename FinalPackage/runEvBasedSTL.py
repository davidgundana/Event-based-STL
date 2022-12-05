#!/usr/bin/python3
from NatNetClient import NatNetClient
from util import quaternion_to_euler
import numpy as np
import tkinter as tk
from tkinter import ttk
import tkinter.filedialog
from math import fabs, pi
from scipy.io import savemat
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
# from activateProp import activateProp
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import preparation as prep
import makeRoadmap
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import control
import helperFuncs
import forwardBuchi
from datetime import datetime
import signal


class runSpec:
    def __init__(self,robot,logData):
        self.robot = robot
        self.sizeState = 6 # state size of a robot
        self.sizeU = 6 # size of the control input
        # self.sizeU = 5 # size of the control input

        self.initialState = '6,6,3.8,0,0,30' # initial state of the system
        self.maxV = '0.2,0.2,0.2,0.05,0.07,15' #Maximum velocity
        # self.maxV = '0.2,0.2,0.05,0.1,12' #Maximum velocity

        # stretch reference values
        self.initialStateRef = '-3,-3,0.025,0,-5,-5,-5,0,0,0,0,0,0,0,0,0,0' # Initial state of reference objects
        self.linearControl = 1 # Control affine system (default is True)
        self.running = True # initialize the system to run
        self.logData = logData # Log data flag
        self.modifyTime = 0 # Keeps track of the time to modify a specification
        if logData:
            self.log = []

        # Stretch Robot Parameters
        self.wheel2Center =.4
        self.offsetX = -.05
        self.armZero = .19
        self.offsetZ = [0.08, -.15, .05]

        # Default spec location
        mainDirectory = os.path.dirname(os.path.abspath(__file__))
        self.filenames = []
        self.filenames.append(os.path.join(mainDirectory, 'Specs', 'ICRA2023Spec4.txt'))
        self.filenames.append(os.path.join(mainDirectory, 'buchiRef.txt'))
        self.filenames.append(os.path.join(mainDirectory, 'Maps', 'openMap.txt'))
        self.filenames.append(os.path.join(mainDirectory, 'Maps', 'openNodes.txt'))
        self.filenames.append(os.path.join(mainDirectory, 'Maps', ''))
        self.filenames.append(os.path.join(mainDirectory, 'Maps', ''))

        try:
            import spot
            self.bypassbuchi = 0
        except:
            self.bypassbuchi = 1
        self.preFailure = 0
    def upload(self,master):
        self.filename1 = tk.filedialog.askopenfilename()
        tk.Label(master[0], text='                               ').grid(row=master[1], column =2)
        tk.Label(master[0], text=os.path.split(self.filename1)[-1]).grid(row=master[1], column =2)
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
    def userParams(self,results):
        # Update the values based on what was given in the window
        self.sizeState = int(results[0].get())
        self.initX = np.array([float(i) for i in results[1].get().split(',')])
        self.sizeU = int(results[2].get())
        self.M = int(self.sizeState/np.size(self.initX))
        self.maxV = np.fromstring(results[3].get(), dtype=float, sep=',')
        self.initXRef = np.array([float(i) for i in results[4].get().split(',')])
        self.name = re.split('\.',os.path.split(self.filenames[0])[-1])[0]

    def userData(self, results):
        text1 = results[5]
        master = results[6]
        master.grid_columnconfigure(2, minsize=40)

        #Update the status of the automaton generation
        self.updateStatus(text1, master, '\nAbstracting Event-based STL Propositions...\n')

        #update the user parameters to reflect what was inputted
        self.userParams(results)

        #Read in the Event-Based STL formula
        x1 = open(self.filenames[0], 'r').readlines()
        self.STL_list = np.asarray(x1).reshape(-1, 1)

        psi = self.STL_list[0][0]
        prepTime = time.time()

        # Abstract predicates and formula as an LTL formula
        Pi_mu, psi, inpRef, inpLabels,evProps,self.psiRef = prep.prepSpec(psi,self.sizeState, self.sizeU)
        b_gamma = prep.getBuchi(Pi_mu, self.filenames[1], text1, master, inpRef, inpLabels,evProps,self.psiRef,self.bypassbuchi)

        # Prepare roadmap
        roadmap = makeRoadmap.mapInfo(self.filenames,text1,master)

        del self.CheckVar
        print('Total prep time: {}'.format(time.time()-prepTime))

        # Run specification
        self.CheckVar2 = tk.IntVar(value=self.preFailure)
        tk.Checkbutton(master, text='Pre-failure Warnings?', highlightbackground='orange', command=self.enablePreFailure,
                       variable=self.CheckVar2).grid(row=14,column=2,sticky=tk.W,pady=4)
        self.gamma = b_gamma
        specParams = [self.gamma, roadmap, master,text1]
        signal.signal(signal.SIGINT, self.sigint_handler)
        tk.Button(master, text='Execute', command=(lambda e=specParams: self.runSpec(e))).grid(row=14, column=3, sticky=tk.W, pady=6)

    def receive_rigid_body_frame(self, id, position, rotation_quaternion):
        # Position and rotation received
        if id == 21 and position != []:
            self.position = position
            self.position += (self.robot.arm.status['pos'],)
            self.position += (self.robot.lift.status['pos'],)
            # print(rotation_quaternion)
            # The rotation is in quaternion. We need to convert it to euler angles
            newRot = (rotation_quaternion[0],rotation_quaternion[1],rotation_quaternion[2],rotation_quaternion[3])
            rotx, roty, rotz = quaternion_to_euler(newRot)
            # Store the roll pitch and yaw angles
            self.rotations = (rotx, roty, rotz)
        if id == 30 and position != []:
            self.objectPosition = position
        if id == 32 and position != []:
            self.objectPosition2 = position
        if id == 33 and position != []:
            self.objectPosition3 = position

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
        if self.robot is not None:
            clientAddress = "199.168.1.72"
            optitrackServerAddress = "199.168.1.164"

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
            self.text1.configure(state='normal')
            self.text1.delete("end-8l", "end")
            self.text1.configure(state='disabled')
            self.master.update()
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
        self.objects = {}
        self.objects2 = {}

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
        self.objects, = ax.plot(self.initXRef[0], self.initXRef[1],'o', color="blue")
        self.ax = ax
        # circle1 = plt.Circle((self.initXRef[15], self.initXRef[16]), 1)
        # ax.add_patch(circle1)
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
        theTime = time.time()
        while self.running:
            if not self.pause:
                '''
                This Simulates the process 
                '''
                print('--------------------------------------------------------------')
                if self.robot is not None:
                    angle = self.transform_to_pipi((np.pi/180)*(self.rotations[2]))[0]
                    self.x[0]= self.position[0]
                    self.x[1] = self.position[1]
                    self.x[2] = angle
                    # print(self.x[0:3])
                    self.x[3] = self.position[3]
                    self.x[4] = self.position[4]
                    self.x[5] = self.robot.end_of_arm.status['stretch_gripper']['pos_pct']
                    self.xR[0] = self.objectPosition[0]
                    self.xR[1] = self.objectPosition[1]
                    self.xR[2] = self.objectPosition[2] - self.offsetZ[0]
                    #depot position
                    self.xR[5] = self.objectPosition2[0]
                    self.xR[6] = self.objectPosition2[1]
                    self.xR[7] = self.objectPosition2[2] - self.offsetZ[1]
                    self.xR[10] = self.objectPosition3[0]
                    self.xR[11] = self.objectPosition3[1]
                    self.xR[12] = self.objectPosition3[2] - self.offsetZ[2]

                print('t: {}, X: {}, Y: {}, Theta: {}, D: {}, Z: {}, Grip: {}'.format(round(self.t,2),round(self.x[0],2),round(self.x[1],2),
                   round(self.x[2],2),round(self.x[3],2),round(self.x[4],2),round(self.x[5],2)))
                self.checkInputs()
                self.updateRef()
                t1 = time.time()
                nom, self.specattr,error = control.synthesis(self.specattr, self.potS, self.roadmap, self.x, self.xR, self.t, self.maxV, self.sizeState,self.sizeU, self.preFailure, self.text1, self.master)
                # print('synth Time: ', time.time()-t1)
                if error and self.robot is not None:
                    self.robot.base.set_velocity(v_m=0, w_r=0)
                    self.robot.arm.set_velocity(v_m=0)
                    self.robot.lift.set_velocity(v_m=0)
                    self.robot.end_of_arm.move_by('stretch_gripper', 0)
                    self.robot.push_command()
                    self.robot.base.set_velocity(v_m=0, w_r=0)
                    self.robot.arm.set_velocity(v_m=0)
                    self.robot.lift.set_velocity(v_m=0)
                    self.robot.push_command()

                    time.sleep(2)
                    self.robot.base.set_velocity(v_m=0, w_r=0)
                    self.robot.push_command()
                    print('stopping robot')
                    self.running = False 
                elif error:
                    self.running = False

                '''
                End of process
                '''
                elapsedT = time.time()-theTime
                theTime = time.time()
                self.t += elapsedT
                formattedTime = "Time: {}s".format(round(self.t,2))
                self.timeText.set_text(formattedTime)
                loopTime = elapsedT

                for i in range(int(self.M)):
                    d = nom[0][3 * i]
                    phi = nom[0][3 * i + 1]
                    vD = nom[0][3 * i + 2]
                    vZ = nom[0][3 * i + 3]
                    vGrip = nom[0][3 * i + 4]
                    print('Vd: {}, vOmega: {}, vD: {},vZ: {}, vGrip: {}'.format(d,phi,vD,vZ,vGrip))
                    if self.logData:
                        nextRow = [self.t] + self.x.tolist() + nom[0].tolist() + self.xR.tolist()
                        for j in range(np.size(self.specattr)):
                            nextRow += self.specattr[j].input.tolist()
                        nextRow += [self.modifyTime]
                        nextRow += [self.psi] + [self.psinew]
                        self.log.append(nextRow)

                    if self.robot is not None:
                        self.robot.base.set_velocity(v_m=d, w_r=phi)
                        self.robot.arm.set_velocity(v_m=vD)
                        self.robot.lift.set_velocity(v_m=vZ)
                        self.robot.end_of_arm.move_by('stretch_gripper', vGrip)
                        self.robot.push_command()
                    else:
                        newPos = helperFuncs.integrateOdom([d,phi],self.x[3*i:3*i+3])
                        self.x[3 * i:3 * i + 3] = newPos
                        self.x[2] = self.transform_to_pipi(self.x[2])[0]
                        self.x[3*i+3] += vD * loopTime
                        self.x[3*i+4] += vZ * loopTime
                        self.x[3*i+5] += vGrip * loopTime

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
            self.objects.set_data(self.xR[0],self.xR[1])
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

    def updateRef(self):
        # xR = [objectX,objectY,objectZ,thetaOrient,dist2Obj,secondObjectX,secondObjectY, secondObjectZ, dist2Obj2,thetaOrient2,,thirdObjectX, thirdObjectY, thirdObjectZ]
        # xy is on front face of robot. arm is in offsetX
        centroidPoint = helperFuncs.robot2global(self.x[0:3], [-self.offsetX, 0])
        self.xR[3] = np.arctan2(self.xR[1]-centroidPoint[1],self.xR[0]-centroidPoint[0]) + np.pi/2
        self.xR[3] = self.transform_to_pipi(self.xR[3])[0]
        self.xR[4] = np.sqrt((self.x[0]-self.xR[0])**2 + (self.x[1]-self.xR[1])**2) - self.armZero
        self.xR[8] = np.sqrt((self.x[0]-self.xR[5])**2 + (self.x[1]-self.xR[6])**2) - self.armZero -.2
        self.xR[9] = np.arctan2(self.xR[6]-centroidPoint[1],self.xR[5]-centroidPoint[0]) + np.pi/2
        self.xR[13] = np.sqrt((self.x[0]-self.xR[10])**2 + (self.x[1]-self.xR[11])**2) - self.armZero
        self.xR[14] = np.arctan2(self.xR[11]-centroidPoint[1],self.xR[10]-centroidPoint[0]) + np.pi/2
    def modifySpec(self):
        circle1 = plt.Circle((self.initXRef[15], self.initXRef[16]), .3)
        self.ax.add_patch(circle1)
        currTime = time.time()
        newSTL = self.PsiSTLnew.get("1.0",tk.END)
        newSTL = re.sub('\\n','',newSTL)
        if self.psi != newSTL:
            print('Change in Psi')
            newPreds,psi,inpRef,inpLabels,evProps,psiNew = prep.prepSpec(newSTL, self.sizeState, self.sizeU)
            oldPreds = self.specattr[0].Pi_mu
            newParameters = []
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
                newParameters.append(self.specattr[0].Pi_mu[i].pred[0])
            self.specattr[0].parameters = newParameters[:len(self.specattr[0].parameters)]
            self.specattr[0].inpRef = inpRef[:len(self.specattr[0].inpRef)]
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
                b_gamma = prep.getBuchi(Pi_mu, '', '', '', inpRefNew, inpLabelsNew, evProps, newSpec ,0)
                inpRefAdd = []
                inpLabelsAdd = []
                evPropsAdd = []
                for j in range(np.size(propositions)):
                    if any(substring in inpRef[j][0] for substring in propositions):
                        inpRefAdd.append(inpRef[j])
                    if any(substring in inpLabels[j][0] for substring in propositions):
                        inpLabelsAdd.append(inpRef[j])
                    # if any(substring in list(evProps.__dict__.keys()) for substring in inpLabels):
                    #     evPropsAdd.append(evProps[j])
                b_gamma.inpRef = inpRefAdd
                b_gamma.inpLabels = inpLabelsAdd
                b_gamma.evProps = evPropsAdd
                if gamma == '&':
                    # Generate Buchi Intersect
                    for i in range(np.size(self.specattr)):
                        print('finding intersection')
                        self.specattr[i], self.potS[i] = buchiFuncs.buchiIntersect(self.specattr[i],self.potS[i], b_gamma,0)
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
        self.modifyTime = time.time()-currTime
        print('Total time to modify: {}seconds'.format(round(time.time()-currTime,2)))
    def makeForm(self):
        #First make the form and create the few entries that are required for initialization
        master = tk.Tk()
        self.master = master
        master.geometry("750x600")

        tk.Label(master, text="Size of Robot State").grid(row=0)
        e1 = tk.ttk.Entry(master,width=3)
        e1.grid(row=0, column=1, columnspan=2,sticky=tk.W)
        e1.insert(10, self.sizeState)

        tk.Label(master, text="Robot Initial state").grid(row=1)
        e2 = tk.ttk.Entry(master,width=30)
        e2.grid(row=1, column=1, columnspan=2,sticky=tk.W)
        e2.insert(10, self.initialState)

        tk.Label(master, text="Size of Control Input").grid(row=2)
        e3 = tk.ttk.Entry(master,width=3)
        e3.grid(row=2, column=1, columnspan=2,sticky=tk.W)
        e3.insert(10, self.sizeU)

        tk.Label(master, text="Max Control Input").grid(row=3)
        e4 = tk.ttk.Entry(master,width=30)
        e4.grid(row=3, column=1, columnspan=2,sticky=tk.W)
        e4.insert(10, self.maxV)

        tk.Label(master, text="Initial Uncontrolled Object State").grid(row=4)
        e5 = tk.ttk.Entry(master,width=30)
        e5.grid(row=4, column=1, columnspan=2,sticky=tk.W)
        e5.insert(10, self.initialStateRef)

        tk.Label(master, text="Upload Event-based STL Specification").grid(row=5)
        tk.Label(master, text="Bypass Buchi Generation?").grid(row=6)
        self.CheckVar = tk.IntVar(value=self.bypassbuchi)
        tk.Checkbutton(master, text='Bypass?', highlightbackground='orange', command=self.enableBypass,
                       variable=self.CheckVar).grid(row=6,column=1,sticky=tk.W,pady=4)
        tk.Label(master, text="Directly Upload Buchi").grid(row=7)
        tk.Label(master, text="OPTIONAL").grid(row=8)
        tk.Label(master, text="Upload Map").grid(row=9)
        tk.Label(master, text="Upload roadmap").grid(row=10)
        tk.Label(master, text="Upload node graph").grid(row=11)
        tk.Label(master, text="Upload map paths").grid(row=12)

        uploadRow = [5,7,9,10,11,12]
        for i in range(np.size(uploadRow)):
            infoToPass = [master, uploadRow[i]]
            tk.Button(master, text='Upload',
                      command=(lambda e=infoToPass: self.upload(e))).grid(row=uploadRow[i],column=1, sticky=tk.W, pady=4)
            tk.Label(master, text=os.path.split(self.filenames[i])[-1]).grid(row=uploadRow[i], column =2, sticky=tk.W)

        tk.ttk.Separator(master, orient='horizontal').grid(column=0, row=8, columnspan=3, sticky='ew')
        tk.ttk.Separator(master, orient='horizontal').grid(column=0, row=13, columnspan=3, sticky='new')
        style = tk.ttk.Style()
        style.configure("TEntry", background='white')

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

        tk.Button(master, text='Quit', highlightbackground='orange', command=master.destroy).grid(row=14,
                                            column=0,sticky=tk.W,pady=4)
        results = np.array([e1, e2, e3,e4,e5,text1,master])
        tk.Button(master, text='Apply',
                  command=(lambda e=results: self.userData(e))).grid(row=14, column=1, sticky=tk.W, pady=6)

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


def initializeRobot(realRobots):
    if realRobots:
        import stretch_body.robot
        robot = stretch_body.robot.Robot()
        robot.startup()
        robot.end_of_arm.move_to('wrist_yaw', 0)
        robot.end_of_arm.move_to('stretch_gripper', 60)
        robot.lift.move_to(1)
        robot.arm.move_to(0)
        robot.push_command()
    else:
        robot = None

    return robot

def stopRobot(robot):
    robot.base.set_velocity(v_m=0, w_r=0)
    robot.arm.set_velocity(v_m=0)
    robot.lift.set_velocity(v_m=0)
    robot.push_command()
    robot.end_of_arm.move_to('stretch_gripper', 25)

    time.sleep(2)
    robot.base.set_velocity(v_m=0, w_r=0)
    robot.push_command()

if __name__ == "__main__":
    realRobots = 1 # Use simulation mode if True
    logData = 0 # log data if True

    robot = initializeRobot(realRobots)

    try:
        f = runSpec(robot,logData)
        f.makeForm()
        tk.mainloop()
    except:
        pass
    finally:
        # Stop the robot
        if realRobots:
            print('Stopping robot')
            stopRobot(robot)
            print('robot stopped!')
        if f.logData:
            print('logging data')
            tpath = os.getcwd()
            my_dir = os.path.dirname(os.path.abspath(__file__))
            date = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
            logName = 'logData_' + date + '.txt'
            logPath = os.path.join(my_dir, 'logs', logName)
            with open(logPath, 'w') as g:
                for line in f.log:
                    g.write(f"{line}\n")
            print('data logged')

    print('finished')

