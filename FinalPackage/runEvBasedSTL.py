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
import matplotlib.pyplot as plt


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
        my_dir3 = os.path.join(my_dir, 'Maps', 'RALMapScaled.txt')
        my_dir4 = os.path.join(my_dir, 'Maps', 'RALNodesScaled.txt')
        my_dir = os.path.join(my_dir, 'Specs', 'RALTest.txt')

        # 1 robots
        self.default = np.array(
            ['1', '5', '.25, .25 ,15', '1.8,-1.25,15', '-1.8,-1.25,0'])

        # 5 robots
        # self.default = np.array(
        #     ['5', '5', '.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15', '1.75,-1.25,15,-.6,1,0,-.6,-.4,0,-.3,-1,0,.3,-1,0', '-1.7,-2,0,1.75,.4,0,-1.5,.5,0'])

        # 7 robots
        # self.default = np.array(
        #     ['7', '5', '.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15',
        #      '1.75,-1.25,15,-.6,1,0,-.6,-.4,0,-.3,-1,0,.3,-1,0, 1.5, -.5,0,1.8,-.5,0', '-1.7,-2,0,1.75,.4,0,-1.5,.5,0'])

        # 9 robots
        # self.default = np.array(
        #     ['9', '5', '.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15,.5, .5 ,15',
        #      '1.75,-1.25,15,-.6,1,0,-.6,-.4,0,-.3,-1,0,.3,-1,0, 1.5, -.5,0,1.8,-.5,0,-1.5,-1,0,-1,-1,0', '-1.7,-2,0,1.75,.4,0,-1.5,.5,0'])

        # self.default = np.array(
        #     ['4', '5', '3,3,15,3,3,15,3,3,15,3,3,15', '4,22,0,20,23,0,36,25,0,5,22,0', '-169,76,0'])  # pre-filled values
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
            buch = spot.translate(spotSTL, 'BA', 'deterministic', 'complete', 'sbacc')
            accepting = self.find_accepting_states(buch.to_str('dot'))
            print('Number of states: ' + str(buch.num_states()))
            print('Number of edges: ' + str(buch.num_edges()))

            auto = buch.to_str('HOA')
            # spot.translate(spotSTL, 'sbacc','Small').to_str('spin')
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
            specattr = initializeSpec.spec(self.spec,accepting)

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
            self.State = initializeSpec.specInfo(specattr, specattr.spec, specattr.props, specattr.propositions, phi,
                        self.controllableProp, self.M, text1, master,self.map,self.nodes,
                                                 self.nodeGraph, self.nodeConnections)
            # Evaluating each transition formula at runtime can be expensive. this function simplifies the formulas.
            self.State = prepBuchi.prep(self.State,specattr.propositions,text1,master)

            # self.updateStatus(text1, master, '\nChecking for conflicts')
            # Check for non-intersecting sets in all transitions (Can be very expensive)
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

    def create_buchi(self, mission_spec, remove=True):
        # create buchi
        import spot
        import networkx as nx
        f = spot.translate(mission_spec, 'BA', 'deterministic', 'complete','sbacc')
        file = open('formula.dot', "w")
        file.write(f.to_str('dot'))
        file.close()
        # convert buchi to nx
        f_nx = (nx.drawing.nx_pydot.read_dot('formula.dot'))

        # label accepting states
        accepting_states = self.find_accepting_states(f.to_str('dot'))
        accepting_labels = {k: 'F' for k in accepting_states}
        nx.set_node_attributes(f_nx, accepting_labels, 'accepting')

        # find initial state
        init_buchi = list(f_nx.successors('I'))[0]

        f_temp = f_nx.copy()
        # init_buchi = list(f_temp.successors('I'))[0]        # buchi should only have one init state
        # print(accepting_states, init_buchi)
        if remove:
            f_temp.remove_node('I')

        return f_temp, accepting_states, init_buchi

    def find_accepting_states(self,dot_str):
        '''
        to be an accepting state: 'peripheries=2' and there exists transition state -> state
        '''
        states = []
        for line in dot_str.split('\n'):
            if line.find('peripheries=2') != -1:
                s = line.split()[0]
                check_transition = s + ' -> ' + s
                # if check_transition in dot_str:
                states.append(s)
        return states


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
            Mes = '4.0705 20.8021 0 19.4432 21 -1.0783e-11 36 25 0 4.8297 20.8423 0 4 22 0 20 23 0 36 25 0 5 22 0 -169 76 0 3.6 0 5 1 2.2 1 2.8 1 2 1 2 1 2 1 2 1 2.6 1 2.6 1 2.6 1 2.6'
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
            filePathP = 'PickleFiles/'+filename+'.pkl'
            filePathM = 'MatlabFiles/'+filename+'.mat'
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
        pickle_file_path = os.path.join(my_dir, 'PickleFiles', 'RALTest.pkl')
        with open(pickle_file_path, 'rb') as input:
            f = pickle.load(input)
        #Get the inputs for the function to get robot commands. Inputs can be from gui or from a copied message
        I = cmdInp()
        I.getInputs(f)

    runOnce = 1
    if runOnce and f.ready:
        debug = 0
        # Time how long it takes to get a command.
        if debug:
            t = time.time()
            vx, vy, vtheta, I.currState, distTotal, newinput, I.until = getCMD(f,I.posX,I.posY,I.posTheta,I.posXinit,I.posYinit,
                I.posThetainit,I.posXPerson,I.posYPerson,I.posThetaPerson,I.currTime,I.startTime,I.currState, I.input,I.until)
            elapsedT = time.time() - t

            I.input = newinput
            print(elapsedT)

            print(vx,vy)
            time.sleep(.1)
        else:
            fig = plt.figure()
            ax = fig.add_subplot(111)
            walls = f.map

            xwall = []
            ywall = []
            for i in range(np.size(f.map,0)):
                xwall.append(f.map[i,0])
                xwall.append(f.map[i, 2])
                xwall.append(None)
                ywall.append(f.map[i,1])
                ywall.append(f.map[i, 3])
                ywall.append(None)



            plt.ion()
            plt.show()
            ax.plot(xwall, ywall, color="black")
            dispRoadmap = 0
            if dispRoadmap:
                xNodes = []
                yNodes = []
                for i in range(np.size(f.State.nodeGraph, 0)):
                    for j in range(np.size(f.State.nodeGraph, 1)):
                        if f.State.nodeGraph[i, j] > 0:
                            xLine = [f.State.nodes[i, 0], f.State.nodes[j, 0]]
                            yLine = [f.State.nodes[i, 1], f.State.nodes[j, 1]]
                            ax.plot(xLine, yLine, color="red")
            plt.draw()
            plt.pause(0.001)
            robots = {}
            colors = ["red", "blue", "green","black"]
            for i in range(f.M):
                robots[str(i)] = ax.plot(f.initPos[3 * i], f.initPos[3 * i + 1], marker='o', markersize=3,
                                         color=colors[0])
                # numR = np.floor(f.M/2)
                # if i < int(numR-1):
                #     robots[str(i)] = ax.plot(f.initPos[3*i],f.initPos[3*i+1], marker='o', markersize=3, color=colors[0])
                # else:
                #     robots[str(i)] = ax.plot(f.initPos[3*i],f.initPos[3*i+1], marker='o', markersize=3, color=colors[int(np.floor((i+1)/numR))])

            plt.draw()
            plt.pause(0.001)

        posX = []
        posY = []
        posTheta = []
        posPX = []
        posPY = []
        posPTheta = []
        for i in range(int(f.M)):
            posX = np.append(posX, float(f.initPos[3 * i]))
            posY = np.append(posY, float(f.initPos[3 * i + 1]))
            posTheta = np.append(posTheta, float(f.initPos[3 * i + 2]))
        for i in range(int(f.P)):
            posPX = np.append(posPX, float(f.initPosRef[3 * i]))
            posPY = np.append(posPY, float(f.initPosRef[3 * i + 1]))
            posPTheta = np.append(posPTheta, float(f.initPosRef[3 * i + 2]))
        realTime = 0
        hz = .1
        if realTime:
            startTime = time.time()
            runTime = time.time()-startTime
        else:
            runTime = 0

        currState = 0
        input = I.input
        allTimes = []
        while runTime < 30:
            loopStart = time.time()
            if runTime > 2:
                input[0] = 1
                input[1] = 2
            if runTime > 3:
                input[0] = 0
                input[1] = 0
            if runTime > 3:
                input[2] = 1
                input[3] = 3
            if runTime > 5:
                input[2] = 0
                input[3] = 0
            if f.N > 2:
                if runTime > 9:
                    input[4] = 1
                    input[5] = 9
                if runTime > 10:
                    input[4] = 0
                    input[5] = 0

            vx, vy, vtheta, currState, distTotal, newInput, I.until = getCMD(f, posX, posY, posTheta,
                                                                               I.posXinit, I.posYinit,
                                                                               I.posThetainit, posPX,
                                                                               posPY, posPTheta,
                                                                               runTime, 0, currState,
                                                                               input, I.until)
            print(vx[0],vy[0])
            loopTime = time.time()-loopStart
            input = newInput
            allTimes.append(loopTime)

            for i in range(f.M):
                rob = robots[str(i)].pop(0)
                rob.remove()

            #update positions
            if not realTime:
                loopTime = hz
            for i in range(int(f.M)):
                posX[i] = posX[i] + vx[0][i] * loopTime
                posY[i] = posY[i] + vy[0][i] * loopTime
                posTheta[i] = posTheta[i] + vtheta[0][i] * loopTime
                if i == 0:
                    robots[str(i)] = ax.plot(posX[i],posY[i], marker='o', markersize=3, color=colors[0])
                else:
                    robots[str(i)] = ax.plot(posX[i],posY[i], marker='o', markersize=3, color=colors[int(np.floor((i+1)/numR))])


            # Hard Code pos of human and spills for experiment
            posPX[0] = posX[0]
            posPY[0] = posY[0]
            posPTheta[0] = posTheta[0]
                # robots[str(i)] = ax.plot(posX[i],posY[i], marker='o', markersize=3, color=colors[int(np.floor(i/2))])

            if realTime:
                runTime = time.time()-startTime
            else:
                runTime += hz
            plt.title("Time: " + str(round(runTime,2)) + "s")
            plt.draw()
            plt.pause(0.001)

        avgT = np.average(allTimes)
        maxT = np.max(allTimes)
        print("Average Computation time: " + str(avgT) + "s")
        print("Max Computation time: " + str(maxT) + "s")

        for i in range(np.size(f.map,0)):  # looping statement;declare the total number of frames
            plt.pause(0.1)
