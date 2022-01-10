import numpy as np
import tkinter as tk
from tkinter import ttk
import tkinter.filedialog
from scipy.io import savemat
import parseEvBasedSTL
import initializeSpec
import prepBuchi
import checkForConflicts
import pickle
import time
import re
import os
from getAllCommands import getAllCommands
from prepForCommands import getCMD
from makeSpecs import getSpecs, negationNormal, findInfo, handleUntil, eventFormulas
from activateProp import activateProp


class formData:
    def __init__(self):
        self.M = 2 #Number of Robots
        self.N = 4  # Number of Inputs
        self.P = 1 #Number of Humans/dynamic obstacles
        self.freq = 0 #Frequency of simulatiom
        self.maxV = 0 #Maximum velocity
        # Default spec location
        my_dir = os.path.dirname(os.path.abspath(__file__))
        my_dir2 = os.path.join(my_dir, 'Specs', '')
        my_dir3 = os.path.join(my_dir, 'Maps', 'openMap.txt')
        my_dir4 = os.path.join(my_dir, 'Maps', 'openNodes.txt')
        my_dir = os.path.join(my_dir, 'Specs', 'specTest.txt')

        self.default = np.array(
            ['1', '5', '1.5,1.5,15', '-169,72,0,2', '-169,76,0'])  # pre-filled values
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
        self.State = [] #pre-allocate state
        self.Conflicts = [] #pre-allocate conflicts
        self.controllableProp = [] #pre-allocate controllable propositions
        self.initPos = [] #pre-allocate initial position
        self.initPosRef = [] #pre-allocate position od dynamic obstacles

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
        self.initPos = results[3].get()
        self.initPos = self.initPos.split(',')
        self.initPos = [float(i) for i in self.initPos]
        self.initPos = np.array(self.initPos)
        self.initPosRef = results[4].get()
        self.initPosRef = self.initPosRef.split(',')
        if self.initPosRef[0] == '':
            self.initPosRef = []
        else:
            self.initPosRef = [float(i) for i in self.initPosRef]
        self.initPosRef = np.array(self.initPosRef)
        self.name = re.split('\.',os.path.split(self.filename1)[-1])[0]
        self.P = int(np.floor(np.size(self.initPosRef)/3))

    def userData(self, results):
        t2 = time.time()
        text1 = results[5]
        master = results[6]
        master.grid_columnconfigure(2, minsize=40)

        #Update the status of the automaton generation
        self.updateStatus(text1, master, '\nAbstracting Event-based STL Propositions...')

        #update the user parameters to reflect what was inputted
        self.userParams(results)

        #Read in the Event-Based STL formula
        x1 = open(self.filename1, 'r').readlines()
        self.STL_list = np.asarray(x1).reshape(-1, 1)

        EvSTLForm = self.STL_list[0][0]

        # Save Formulas for environment events
        EvSTLForm, envInputs = eventFormulas(EvSTLForm)

        # Put the formula in negation normal form
        if '!' in EvSTLForm:
            EvSTLForm = negationNormal(EvSTLForm)

        # Parse  the Event-Based STL formula
        t, spotSTL, predLabels = getSpecs(EvSTLForm, envInputs)

        # Option to print tree to make sure its correct
        viewTree = 1
        if viewTree == 1:
            t.pretty_print()

        # Create an object with all of the predicates
        for i in range(np.size(predLabels, 0)):
            tempOperator, allInputs = findInfo(t, predLabels[i][1])
            predLabels[i].append(tempOperator)
            for j in range(np.size(allInputs)):
                allInputs[j] = envInputs[np.where(np.asarray(envInputs) == allInputs[j])[0][0]][1]
            predLabels[i].append(allInputs)

        # Handle phi Until
        predLabels = handleUntil(spotSTL, predLabels)
        phi = []
        for i in range(np.size(predLabels, 0)):
            phi.append(parseEvBasedSTL.Parsed(predLabels[i], i, self.M, self.initPos, self.initPosRef))

        # Create a buchi automaton from our spot specification
        # Some computers are not able to use spot. In this case turn bypass to "1" and you can choose a spot online
        # generated Buchi Automaton
        self.ready = 1
        if self.bypassbuchi == 0:
            t = time.time()
            import spot
            print(spotSTL)
            # Generate Buchi automaton and save
            auto = spot.formula(spotSTL).translate('BA').to_str('spin')
            auto = auto.splitlines()
            auto = auto[1:]

            self.spec = auto

            elapsedT = time.time() - t
            timeToCompile = 'The total time to generate the Buchi automaton was ' + str(elapsedT) + ' seconds.'
            with open('buchiRef.txt', 'w') as filehandle:
                for listitem in self.spec:
                    filehandle.write('%s\n' % listitem)
            print(timeToCompile)
        else:
            #Load in Buchi
            print('Enter this specification in Spot')
            print(spotSTL)

            try:
                x2 = open(self.filename2, 'r').readlines()
                self.spec =x2
            except:
                self.ready = 0
        if self.ready:
            # Edit the specification to be used by python and create object for propositions
            specattr = initializeSpec.spec(self.spec)

            # determine set of uncontrollable propositions and controllable propositions from specification
            controllableProp = [re.findall('pred', elem) for elem in specattr.propositions]
            for i in range(np.size(controllableProp)):
                if len(controllableProp[i]) != 0:
                    self.controllableProp.append(specattr.propositions[i])
            # Find number of inputs
            self.N = np.size(specattr.propositions) - np.size(self.controllableProp)


            # load in map and nodes
            self.map = np.loadtxt(self.filename3)
            self.nodes = np.loadtxt(self.filename4)
            if self.filename5 == "":
                self.nodeGraph = []
            else:
                with open(self.filename5, 'rb') as input:
                    self.nodeGraph = pickle.load(input)
            if self.filename6 == "":
                self.nodeConnections = []
            else:
                with open(self.filename6, 'rb') as input:
                    self.nodeConnections = pickle.load(input)

            #Create State
            self.State = initializeSpec.specInfo(specattr.spec, specattr.props, specattr.propositions, phi,
                        self.controllableProp, self.M, text1, master,self.map,self.nodes,
                                                 self.nodeGraph, self.nodeConnections)
            # Evaluating each transition formula at runtime can be expensive. this function simplifies the formulas.
            self.State = prepBuchi.prep(self.State,specattr.propositions,text1,master)
            self.updateStatus(text1, master, '\nChecking for conflicts')

            # Check for non intersecting sets in all transitions (Can be very expensive)
            # conf = checkForConflicts.check(self.State, self.M, text1, master)
            # self.Conflicts = conf


            elapsedT2 = time.time() - t2
            timeToFinish = 'The total time prepare the Event-based STL formula was ' + str(elapsedT2) + ' seconds.'
            print(timeToFinish)
        del self.CheckVar
        master.destroy()

    def makeForm(self):
        #First make the form and create the few entries that are required for initialization
        master = tk.Tk()
        master.geometry("750x600")
        tk.Label(master, text="Number of Robots").grid(row=0)
        tk.Label(master, text="Frequency (Hz)").grid(row=1)
        tk.Label(master, text="Max Velocity (x_i, y_i, theta_i,...)").grid(row=2)
        tk.Label(master, text="Init state (x_i, y_i, theta_i,...)").grid(row=3)
        tk.Label(master, text="Init uncontrolled state").grid(row=4)
        tk.Label(master, text="Upload Event-based STL Specification").grid(row=5)
        tk.Label(master, text="Bypass Buchi Generation?").grid(row=6)
        tk.Label(master, text="Directly Upload Buchi").grid(row=7)
        tk.ttk.Separator(master, orient='horizontal').grid(column=0, row=8, columnspan=7, sticky='ew')

        tk.Label(master, text="OPTIONAL").grid(row=8)
        tk.Label(master, text="Upload Map").grid(row=9)
        tk.Label(master, text="Upload roadmap").grid(row=10)
        tk.Label(master, text="Upload node graph").grid(row=11)
        tk.Label(master, text="Upload map paths").grid(row=12)
        tk.ttk.Separator(master, orient='horizontal').grid(column=0, row=13, columnspan=7, sticky='new')


        e1 = tk.Entry(master)
        e2 = tk.Entry(master)
        e3 = tk.Entry(master)
        e4 = tk.Entry(master)
        e5 = tk.Entry(master)
        e6 = tk.Entry(master)
        e7 = tk.Entry(master)
        e8 = tk.Entry(master)
        e9 = tk.Entry(master)

        # I configure it orange because there are display issues on a mac
        e1.config(bg="orange")
        e2.config(bg="orange")
        e3.config(bg="orange")
        e4.config(bg="orange")
        e5.config(bg="orange")
        e6.config(bg="orange")
        e7.config(bg="orange")
        e8.config(bg="orange")
        e9.config(bg="orange")

        # Place default values. This is useful for quick entering
        e1.insert(10, self.default[0])
        e2.insert(10, self.default[1])
        e3.insert(10, self.default[2])
        e4.insert(10, self.default[3])
        e5.insert(10, self.default[4])

        e1.grid(row=0, column=1)
        e2.grid(row=1, column=1)
        e3.grid(row=2, column=1)
        e4.grid(row=3, column=1)
        e5.grid(row=4, column=1)

        #create status window to show updates during compilation
        text1 = tk.Text(master, height=15, width=50)
        scroll = tk.Scrollbar(master, command=text1.yview)
        text1.configure(yscrollcommand=scroll.set)
        text1.grid(row = 18, column=0, rowspan=12)
        text1.tag_configure('Title', font=('Arial', 14, 'bold'))
        text1.tag_configure('text', font=('Arial', 12, 'bold'))
        text1.configure(state='normal')
        text1.insert(tk.END, '\nStatus Updates\n','Title')
        text1.insert(tk.END, '\nWaiting for parameters...', 'text')
        text1.configure(state='disabled')

        #Create buttons and start filling out form
        tk.Button(master, text='Upload', highlightbackground='orange',
                  command=(lambda e=master: self.upload1(e))).grid(row=5,column=1, sticky=tk.W, pady=4)
        tk.Label(master, text=os.path.split(self.filename1)[-1]).grid(row=5, column =2)

        tk.Button(master, text='Upload', highlightbackground='orange',
                  command=(lambda e=master: self.upload2(e))).grid(row=7,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename2)[-1]).grid(row=7, column=2)




        tk.Button(master, text='Upload', highlightbackground='orange',
                  command=(lambda e=master: self.upload3(e))).grid(row=9,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename3)[-1]).grid(row=9, column=2)

        tk.Button(master, text='Upload', highlightbackground='orange',
                  command=(lambda e=master: self.upload4(e))).grid(row=10,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename4)[-1]).grid(row=10, column=2)

        tk.Button(master, text='Upload', highlightbackground='orange',
                  command=(lambda e=master: self.upload5(e))).grid(row=11,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename5)[-1]).grid(row=11, column=2)

        tk.Button(master, text='Upload', highlightbackground='orange',
                  command=(lambda e=master: self.upload6(e))).grid(row=12,column=1, sticky=tk.W,pady=4)
        tk.Label(master, text=os.path.split(self.filename6)[-1]).grid(row=12, column=2)

        self.CheckVar = tk.IntVar(value=self.bypassbuchi)
        tk.Checkbutton(master, text='Bypass?', highlightbackground='orange', command=self.enableBypass,
                       variable=self.CheckVar).grid(row=6,column=1,sticky=tk.W,pady=4)

        tk.Button(master, text='Quit', highlightbackground='orange', command=master.quit).grid(row=14,
                                            column=0,sticky=tk.W,pady=4)
        results = np.array([e1, e2, e3,e4,e5,text1,master])

        tk.Button(master, text='Apply', highlightbackground='green',
                  command=(lambda e=results: self.userData(e))).grid(row=14, column=1, sticky=tk.W, pady=6)

class cmdInp:
    def __init__(self):
        self.posX = []
        self.posY = []
        self.posTheta = []
        self.posXinit = []
        self.posYinit = []
        self.posThetainit = []
        self.posXPerson = []
        self.posYPerson = []
        self.posThetaPerson = []
        self.currTime = 0
        self.startTime = 0
        self.currState = 0
        self.input = []
        self.until = []
    def getInputs(self,f):
        # DEBUG MESSAGE MODE TOGGLE
        # Turn this value to "1" if you want to enter an example message from Unity/Matlab as an input. Otherwise, inputs
        # above will be used
        debugMessage = 0

        if not debugMessage:
            for i in range(int(f.M)):
                self.posX = np.append(self.posX,f.initPos[3*i])
                self.posY = np.append(self.posY, f.initPos[3 * i + 1])
                self.posTheta = np.append(self.posTheta, f.initPos[3 * i + 2])
            for i in range(int(f.P)):
                self.posXPerson = np.append(self.posXPerson,f.initPosRef[3*i])
                self.posYPerson = np.append(self.posYPerson, f.initPosRef[3 * i + 1])
                self.posThetaPerson = np.append(self.posThetaPerson, f.initPosRef[3 * i + 2])

            self.posXinit = self.posX
            self.posYinit = self.posY
            self.posThetainit = self.posTheta
            self.input = np.zeros((1,2*f.N), dtype=float)[0]

        else:
            # Example message from Unity/Matlab
            Mes = '0.42426 0.42426 0 0 0 0 0 0 0 7 0 1 1 5 1 6.8 1 6.6'
            parsedMes = Mes.split()
            for i in range(int(f.M)):
                self.posX = np.append(self.posX, float(parsedMes[3 * i]))
                self.posY = np.append(self.posY, float(parsedMes[3 * i + 1]))
                self.posTheta = np.append(self.posTheta, float(parsedMes[3 * i + 2]))
            for i in range(int(f.M)):
                self.posXinit = np.append(self.posXinit,float(parsedMes[3 * i + 3 * f.M]))
                self.posYinit = np.append(self.posYinit,float(parsedMes[3 * i + 1 + 3 * f.M]))
                self.posThetainit = np.append(self.posThetainit,float(parsedMes[3 * i + 2 + 3 * f.M]))
            for i in range(f.P):
                self.posXPerson = np.append(self.posXPerson, float(parsedMes[3 * i + 6 * f.M]))
                self.posYPerson = np.append(self.posYPerson,  float(parsedMes[3 * i + 1 + 6 * f.M]))
                self.posThetaPerson = np.append(self.posThetaPerson, float(parsedMes[3 * i + 2 + 6 * f.M]))

            self.currTime = float(parsedMes[6 * f.M + 3 * f.P])
            self.startTime = float(parsedMes[6 * f.M + 1 + 3 * f.P])
            self.currState = int(float(parsedMes[6 * f.M + 2 + 3 * f.P]))

            inputs = list(map(float, parsedMes[6 * f.M + 3 + 3 * f.P:]))
            self.input = np.append(self.input, inputs)




if __name__ == "__main__":
    loadOnStart = 1
    if loadOnStart == 0:
        f = formData()
        f.makeForm()
        tk.mainloop()
        if f.ready:
            filename = f.name
            filePathP = 'Pickle Files/'+filename+'.pkl'
            filePathM = 'Matlab Files/'+filename+'.mat'
            my_dir = os.path.dirname(os.path.abspath(__file__))
            pickle_file_path = os.path.join(my_dir, filePathP)

            with open(pickle_file_path, 'wb') as output:
                pickle.dump(f, output, pickle.DEFAULT_PROTOCOL)

            I = cmdInp()
            I.getInputs(f)
            # Save matlab file
            dict = {"freq":float(f.freq), "robots":f.M,"humans":f.P,"init_robot":f.initPos,"init_human":f.initPosRef,
                    "input":np.zeros((1,2*f.N))[0],"map":f.map,"inputNames":f.State.uncontrollableProp,"nodes":f.nodes}
            savemat(filePathM, dict)
    elif loadOnStart == 1:
        my_dir = os.path.dirname(os.path.abspath(__file__))
        pickle_file_path = os.path.join(my_dir, 'Pickle Files', 'specTest.pkl')
        with open(pickle_file_path, 'rb') as input:
            f = pickle.load(input)
        #Get the inputs for the function to get robot commands. Inputs can be from gui or from a copied message
        I = cmdInp()
        I.getInputs(f)

    runOnce = 1
    if runOnce and f.ready:
        # Time how long it takes to get a command.
        t = time.time()
        # I.input[0] = 1
        # I.input[2] = 1
        # I.input[8] = 1
        # I.input[10] = 1
        # I.currState = 1
        # I.currTime = I.currTime+1/float(f.freq)
        # vx, vy, vtheta, I.currState, distTotal, newinput, I.until = getCMD(f,[0.1069313], [65.41315], [0.3481325], [0.02], [54.13], [0.], [0.12], [67.55], [191.6], 35.54, 0.0, 2, np.array([ 1.,   26.72,  0.,    0. ]), 1)

        isect = activateProp.intersectPoint([], I.posX[0], I.posY[0], I.posXPerson[0], I.posYPerson[0],
                                            f.map[:, 0], f.map[:, 1], f.map[:, 2], f.map[:, 3])
        vx, vy, vtheta, I.currState, distTotal, newinput, I.until = getCMD(f,I.posX,I.posY,I.posTheta,I.posXinit,I.posYinit,
            I.posThetainit,I.posXPerson,I.posYPerson,I.posThetaPerson,I.currTime,I.startTime,I.currState, I.input,I.until)
        elapsedT = time.time() - t

        I.input = newinput
        print(elapsedT)

        print(vx,vy,vtheta,newinput)
        time.sleep(.1)
