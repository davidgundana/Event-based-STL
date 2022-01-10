# Event-based Signal Temporal Logic (STL) #
Event-based STL is a specification formalism used to satisfy reactive high-level specifications with reaction to uncontrolled external environment events. Given an Event-based STL specification, this toolbox automatically synthesizes controllers to satisfy the task. More information on Event-based STL can be found in the following paper: 

Gundana, David, and Hadas Kress-Gazit. "Event-based Signal Temporal Logic Synthesis for Single and Multi-Robot Tasks." IEEE Robotics and Automation Letters 6.2 (2021): 3687-3694.
-	https://ieeexplore.ieee.org/abstract/document/9372768

# Event-based STL Toolbox #
## System Model ##
- Event-based STL specifications
    -  An Event-based STL specification Ψ can be written using the following syntax
   
    <img width="400" alt="image" src="https://media.github.coecis.cornell.edu/user/5533/files/dd4dbccd-b151-4488-9e9c-ab7636e9169f">
    
    -  More details can be found in the paper referenced above
-  Creating Predicates μ
    - A predicate μ can be a function of controllable and uncontrollable variables  
        - Controllable variables
            - By default, each robot in the system is assumed to have 3 states (x,y, and θ). Control can be generated to change these variables during an execution
        - Uncontrollable variables 
            - Uncontrollable variables can exist in an Event-based STL specification have states (x,y, and θ). Control can not be found for these variables as they are uncontrollable. Examples include people, uncontrollable robots, and dynamic obstacles.
        - Function operators
            - A predicate can contain the operators square root (sqrt) and absolute value (abs)  
- Environment variables π
    - Environment variables are uncontrolled inputs such as signals and alarms.  
## Installing Dependencies ##
Required:
-	Python 3.x
-	Python Libraries:
    -	NumPy - https://numpy.org/install/
    -	SciPy - https://scipy.org/install/
    -	NetworkX - https://networkx.org/documentation/stable/install.html
    -	nltk - https://www.nltk.org/install.html

Optional:
-	Matlab (for simulation purposes)
    -	Required toolbox:
        -	Instrument Control Toolbox	
-	Unity (for simulation purposes)
-	Python Libraries
    -	Spot - https://spot.lrde.epita.fr/install.html

## Overview ##

  1.	Write an Event-based STL specification in the correct format 
  2.	Prepare the specification for execution 
  3.	Manually generate a Büchi automaton if Spot is not installed 
  4.	Simulate an execution of a specification 
  
  ![flowchart](https://media.github.coecis.cornell.edu/user/5533/files/1be19206-24e7-4126-93b1-05ac6d982c7b)


## Writing Specifications ## 
- Syntax of an Event-based STL formula Ψ

<img width="400" alt="image" src="https://media.github.coecis.cornell.edu/user/5533/files/dd4dbccd-b151-4488-9e9c-ab7636e9169f">

- When writing Event-based STL specifications, the following rules should be followed
    - Temporal operators 
        - G_[a,b] is written as alw_[a,b]
        - F_[a,b] is written as ev_[a,b]
        - U_[a,b] is written as un_[a,b]
        - G without a timing bound [a,b] is written as G
-	The state of each robot is a controllable variable. When writing predicates, the state of all robots are in the vector "pos" such that "pos = [x1,y1,θ1,x2,y2,θ2, ... ]" 
    -	For example the states x, y, θ for robot 1 can be represented as pos[0], pos[1], pos[2]
    -	The states x, y, θ for robot 2 can be represented as pos[3], pos[4], pos[5]
    -	If there are 2 robots the size of pos should be 6
-	Parentheses are only used for grouping of predicates, temporal operators, or events
-	Predicates should not contain any spaces (ex. “sqrt[[pos[0]-2]^2+[pos[1]-2]^2]<1” )
-	Temporal operators and timing bounds should not contain spaces (ex. ev_[0,10])
-	All predicates grouped with a temporal operator are surrounded by parenthesis (ex. “(ev_[5,10] pos[0]>1 | alw_[0,5] pos[1]<2)” )
-	All environment events are grouped with parentheses (ex. “((alarm1 & alarm2) | (alarm3 | alarm4))
-	The specification is saved as a one line .txt file ("specification1.txt")
-	Example 1:
    -	G(alarm1 => (ev_[0,10] (pos[0]>10 | pos[3]>11)))
-	Example 2: 
    - G(((alarm1 & alarm2) => ((ev_[0,10] sqrt[[pos[0]-17]^2+[pos[1]-2]^2]<1) | (ev_[0,10] sqrt[[pos[3]-17]^2+[pos[4]-2]^2]<1))))


## Preparing a Specification ##
To prepare a specification for execution, run the file “runEvBasedSTL.py”. This script will open a GUI and ask for several different inputs. 

<img width="600" alt="image" src="https://media.github.coecis.cornell.edu/user/5533/files/ad480c13-9232-4b4c-b4e8-a71e0013fae6">

- Number of Robots: Number of controllable robots that the specification includes
- Frequency (Hz): Frequency at which the specification is run. 
- Max Velocity: Velocity bounds for each robot (x,y,theta)
- Initial state: Initial position of each controllable robot (for simulation purposes)
- Init uncontrolled: Initial position of dynamic obstacles (humans, uncontrolled robots, etc.)
- Upload Event-based STL Specification: Upload a txt file that contains the one line specification. The specification must be in the form described in the “Writing 
Specifications” section of this document
- Bypass Büchi Generation?: By default, the script will use Spot to generate a Büchi automaton from the given specification. If Spot is not installed, the Büchi can be uploaded as a separate txt file. The Büchi can be obtained from the online tool on the Spot website. 
- Directly Upload Büchi: If the bypass option is checked, upload the txt file containing the Büchi automaton generated by the online tool on the Spot website. 
- Upload map: upload an environment map. The map should be a .txt file with each line representing a line segment [x1 y1 x2 y2]
- Upload nodes: upload a roadmap for the map. The nodes should be in a .txt file with each line representing a point [x y]

Output
- The output of this script is a pickle file and mat file that contains the information necessary to execute a specification. The pickle file is saved in the “pickle files” folder and the mat file is saved in the “matlab files” folder. 
- The name of the files is the name of the .txt file for the specification


## Generating a Büchi Automaton using Spot's online tool ## 
-	Select the Bypass option on the GUI after running runEvBasedSTL.py
-	After pressing "Apply" the abstracted Event-based STL specification will be printed in the console
-	Visit https://spot.lrde.epita.fr/app/ and enter the printed specification
-	Select "Acceptance: (State-based) Büchi " and "complete"
-	After pressing enter to generate the automaton, select "NEVERCLAIM" and copy the output into a txt file. 
-	To generate the pickle file containing the necessary information for execution, run "runEvBasedSTL.py", select "bypass", and upload the txt file with the Büchi generated from Spot.


## Executing a specification ##
-	A specification is executed through simulation in Unity (clientUnity.py) or Matlab (clientMatlab.py). 
-	Running in Matlab
    -	In the file “clientMatlab.py” change the pickle_file_path variable to the pickle file for your specification
    -	In a terminal run “clientMatlab.py”. The client will attempt to connect to Matlab to begin the simulation
    -	In Matlab run the script “RunTCPSim.m”. The mat file that is loaded should match pickle file 
    -	A simulation window will appear with a start button to begin the simulation and input buttons which represent environment events. 
-	<img width="600" alt="image" src="https://media.github.coecis.cornell.edu/user/5533/files/b410e2d2-71ac-482f-9522-440dc9aeaa89">
-	Running in Unity
    -	In the file “clientUnity.py” change the pickle_file_path variable to the pickle file for your specification
    -	In a terminal run “clientUnity.py”. The client will attempt to connect to Unity to begin the simulation
    -	To run a simulation in Unity attach the file “TCPServer.cs” to a robot in a Unity environment. 
    -	When you begin the game in Unity, the server will connect and the execution will begin
-	Debugging a Specification
    -	At each time step a message is printed from the Unity or Matlab console. This string contains information about the state of the robots and the system that are used to generate control. To debug a specification change the debugMessage variable to “1” in 
“runEvBasedSTL.py” and change the Mes variable to the string that was printed from the Matlab/Unity console. This will allow you to run the specification at the time step where an error occurred.
