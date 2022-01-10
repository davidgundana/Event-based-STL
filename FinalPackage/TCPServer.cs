using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Data;
using System.IO;
using UnityEngine;
using UnityStandardAssets.Characters.FirstPerson;
using UnityEngine.UI;

// adapted from https://gist.github.com/danielbierwirth/0636650b005834204cb19ef5ae6ccedb

public class TCPServer : MonoBehaviour
{
	#region private members 	
	/// <summary> 	
	/// TCPListener to listen for incomming TCP connection 	
	/// requests. 	
	/// </summary> 	
	private TcpListener tcpListener;
	/// <summary> 
	/// Background thread for TcpServer workload. 	
	/// </summary> 	
	private Thread tcpListenerThread;
	/// <summary> 	
	/// Create handle to connected tcp client. 	
	/// </summary> 	

	private TcpClient connectedTcpClient;
	private TcpClient socketConnection;
	private Thread clientReceiveThread;
	private Thread PyClient;
	#endregion

	// Initialize all variables
	public int connectionEstablished;
    public int route; // which route to escape
	public float initX;
	public float initY;
	public float initTheta;
	public float posPx;
	public float posPy;
	public float posPtheta;
	public int start;
	public float currTime;
	public float initialT;
	public float startTime;
	public int currState;
	public int errorMessage;
	public float cmdVx;
	public float cmdVy;
	public float cmdVTheta;
	public int frameRate;
	public float frameRateFloat;
	public float deltaX;
	public float deltaY;
	public float deltaTheta;
	public int input1;
	public int input2;
	public float input1Time;
	public float input2Time;
	public float theta;
	public int ready2Start;
	public float errorX;
	public float errorY;
	public float errorXexit;
	public float errorYexit;
	public Camera firstPersonCharacter;
	public Camera StartCamera;
	FirstPersonController fpc;
	IdEnter Id;
	startButton begin;
	countdownTimer countTime; 
	public string UserId;
	public string startRun;
	public int userEntered;
	public UnityEngine.UI.Button button;
    public UnityEngine.UI.Button button_route1;
    public UnityEngine.UI.Button button_route2;
    public UnityEngine.UI.InputField inputF;
	public UnityEngine.UI.Text instruct;
	public UnityEngine.UI.Button buttonS;
	public string dataString;
	public int switchView;
	public int specNumber; 
	
	// human object
	public GameObject humanObj;
	public string scriptPath;
	public string specPath;
	public float dist2person;
	public float dist2exit;

	Vector3 humanPos = Vector3.zero;
	public float humanOrientation;

    public GameObject savePersonText;
    public GameObject followText;
    public GameObject route1;
    public GameObject route2;
    public GameObject Sign_Exit_route1;
    public GameObject Sign_Exit_route2;
    // public GameObject followText;

    // robot position
    Vector3 tempPos;
	Vector3 tempOrientation;

	void Start()
	{
		specNumber = 2;
		firstPersonCharacter.enabled = false;
		StartCamera.enabled = true;
		fpc = GameObject.FindObjectOfType<FirstPersonController>();
		fpc.enabled = false;
		Id = GameObject.FindObjectOfType<IdEnter>();
		begin = GameObject.FindObjectOfType<startButton>();
		countTime = GameObject.FindObjectOfType<countdownTimer>();
		button = GameObject.Find("EnterButton").GetComponent<UnityEngine.UI.Button>();
        button_route1 = GameObject.Find("selectRoute1").GetComponent<UnityEngine.UI.Button>();
        button_route2 = GameObject.Find("selectRoute2").GetComponent<UnityEngine.UI.Button>();
        buttonS = GameObject.Find("Start").GetComponent<UnityEngine.UI.Button>();
		inputF = GameObject.Find("InputField").GetComponent<UnityEngine.UI.InputField>();
		instruct = GameObject.Find("Instructions").GetComponent<UnityEngine.UI.Text>();
        savePersonText = GameObject.Find("savePersonText");
        followText = GameObject.Find("followText");

        route1 = GameObject.Find("route1");
        route2 = GameObject.Find("route2");
        Sign_Exit_route1 = GameObject.Find("Sign_Exit_route1");
        Sign_Exit_route2 = GameObject.Find("Sign_Exit_route2");

        // set frame rate and intial position
        frameRate = 40;
		frameRateFloat = frameRate;
		Application.targetFrameRate = frameRate;
		initX = transform.position.x;
		initY = transform.position.z;
		initTheta = 0;
		tempOrientation = transform.eulerAngles;
		tempOrientation.y = initTheta;
		transform.eulerAngles = tempOrientation;

		initTheta = transform.rotation.eulerAngles.y;
		theta = initTheta;

		// Get the position of the human
		humanObj = GameObject.Find("FPSController");
		humanPos = humanObj.transform.position;
		humanOrientation = humanObj.transform.rotation.eulerAngles.y;
		posPx = humanPos[0];
		posPy = humanPos[2];
		posPtheta = humanOrientation;


		start = 0;
        route = 1;
		connectionEstablished = 0;
		userEntered = 0;
		currTime = 0;
		startTime = 0;
		initialT = 0;
		ready2Start = 0;
		currState = 0;
		cmdVx = 0;
		cmdVy = 0;
		errorMessage = 0;
		deltaTheta = 0;
		input1 = 0;
		input2 = 0;
		input1Time = 0;
		input2Time = 0;
		switchView = 0;
		// Start TcpServer background thread to listen for data
		tcpListenerThread = new Thread(new ThreadStart(ListenForIncommingRequests));
		tcpListenerThread.IsBackground = true;
		tcpListenerThread.Start();

		// start python client which should begin listening and transmitting data in a diffeent thread
		scriptPath = Application.dataPath + "/PythonFiles/clientV2.py";
		specPath = Application.dataPath + "/PythonFiles/userData/user";
		//StartPythonClient();
		buttonS.gameObject.SetActive(false);
        savePersonText.gameObject.SetActive(false);

	}

	// Update is called once per frame
	void Update()
	{
		//SendMessage();
		startRun = begin.startGame;
		if (userEntered == 0)
		{
			UserId = Id.id;
			instruct.text = "Welcome to the game! Now you are in a burning building.Your goal is to collect as many items as possible while escaping from the building in 5 minutes! Please enter your 4 digit ID. Are you ready? Click the button to start the game!";

			// if a userId is entered, we can begin the execution
			if (UserId != String.Empty)
			{
				button.gameObject.SetActive(false);
				inputF.gameObject.SetActive(false);
                button_route1.gameObject.SetActive(false);
                button_route2.gameObject.SetActive(false);
                instruct.text = "";
				switchView = 1;
				CreateText(UserId.ToString(), "");
				buttonS.gameObject.SetActive(false);
                savePersonText.gameObject.SetActive(true);
                userEntered = 1;
				ready2Start = 1;

                if (route == 2)
                {
                    route1.gameObject.SetActive(false);
                    Sign_Exit_route1.gameObject.SetActive(true);
                    route2.gameObject.SetActive(true);
                    Sign_Exit_route2.gameObject.SetActive(false);
                    // humanObj.transform.position.x = -169.4;
                    // humanObj.transform.position.z = 72.3;
                }
                else
                {
                    route1.gameObject.SetActive(true);
                    Sign_Exit_route1.gameObject.SetActive(false);
                    route2.gameObject.SetActive(false);
                    Sign_Exit_route2.gameObject.SetActive(true);
                    // humanObj.transform.position.x = 0.12;
                    // humanObj.transform.position.z = 67.55;
                }
			}
		}
		if (switchView == 1)
		{
			//Once an name is inputted, the text fields and buttons are cleared and the camera is switched to the FPS view
			fpc.enabled = true;
			firstPersonCharacter.enabled = true;
			StartCamera.enabled = false;
			switchView = 0;
		}

		SendMessage();
		//SendMessage();
		// slightly outdated but doesnt hurt. Press start to establish connection and then press enter to start execution
		if (switchView == 0 && userEntered == 1)
		{

			if (Input.GetKeyDown(KeyCode.P))
            {
				Application.OpenURL("https://cornell.ca1.qualtrics.com/jfe/form/SV_eM8jvtN17TDeSfr");
			}
				if (ready2Start == 1)
			{
				SendMessage();

                // get human pose
                humanObj = GameObject.Find("FPSController");
                humanPos = humanObj.transform.position;
                humanOrientation = humanObj.transform.rotation.eulerAngles.y;
                posPx = humanPos[0];
                posPy = humanPos[2];
                posPtheta = humanOrientation;

                tempPos = transform.position;
				tempOrientation = transform.eulerAngles;

				errorXexit = -53 - humanPos.x;
				errorYexit = 9 - humanPos.z;

				dist2exit = (float)Math.Sqrt(errorXexit * errorXexit + errorYexit * errorYexit);

				//if (countTime.timeRemaining <= 0 || dist2exit < 2)
                //{
				//	ready2Start = 0;
				//	firstPersonCharacter.enabled = false;
				//	StartCamera.enabled = true;
				//	fpc.enabled = false;
				//	instruct.text = "Congratulations! You have completed the game! One last step! Please go to the link to complete a questionnaire: https://cornell.ca1.qualtrics.com/jfe/form/SV_eM8jvtN17TDeSfr (Press 'p' to open if it did not open automatically)";
				//	Application.OpenURL("https://cornell.ca1.qualtrics.com/jfe/form/SV_eM8jvtN17TDeSfr");

                //    GetComponent<AudioSource>().Stop();
				//}
				

				deltaX = cmdVx * (1 / frameRateFloat);
				deltaY = cmdVy * (1 / frameRateFloat);

				tempPos.x += deltaX;
				tempPos.z += deltaY;
				UnityEngine.Debug.Log(errorMessage);
				theta = cmdVTheta * (1 / frameRateFloat) + theta;

				errorX = posPx - tempPos.x;
				errorY = posPy - tempPos.z;

				//For automatically looking at person
				theta = (float)Math.Atan2(errorX, errorY);
				theta = (theta * 180) / (float)Math.PI;
				//UnityEngine.Debug.Log(theta);
				transform.position = tempPos;
				tempOrientation.y = theta;
				transform.eulerAngles = tempOrientation;


				if (start == 0)
				{
					initialT = Time.time;
					startTime = initialT - Time.time;
				}
				currTime = Time.time - initialT;
				start = 1;

				dist2person = (float)Math.Sqrt(errorX * errorX + errorY * errorY);

				//UnityEngine.Debug.Log(dist2person);
				if (specNumber == 1)
                {
					if (dist2person < 3)
					{
						if (input1 == 0)
                        {
							input1Time = currTime;
                        }
						input1 = 1;
					}
					if (dist2person < 3)
					{
						if (input2 == 0)
						{
							input2Time = currTime;
						}
						input2 = 1;
					}
				}
				if (specNumber == 2)
				{
					if (Input.GetKeyDown(KeyCode.E))
					{
						if (input1 == 0)
						{
							input1Time = currTime;
						}
						input1 = 1;
                        // followText.GetComponent<UnityEngine.UI.Text>().text = "Follow me:)";


                    }
					if (dist2person < 3)
					{
						if (input2 == 0)
						{
							input2Time = currTime;
						}
						input2 = 1;
					}
				}

                if (errorMessage != 0)
                {
                    if (followText != null)
                    {
                        followText.GetComponent<UnityEngine.UI.Text>().text = "Robot \n cannot continue";
                    }
                }


				dataString = currTime.ToString() + ", " + posPx.ToString()+", " + posPy.ToString() + ", " + posPtheta.ToString() + ", " + 
					transform.position.x.ToString() + ", " + transform.position.z.ToString() + ", " + theta.ToString();
				CreateText(UserId.ToString(), dataString);

				dataString = currTime.ToString() + ", " + posPx.ToString() + ", " + posPy.ToString() + ", " + posPtheta.ToString() + ", " +
					transform.position.x.ToString() + ", " + transform.position.z.ToString() + ", " + theta.ToString();
				CreateText(UserId.ToString(), dataString);

			}

		}

	}

	/// <summary> 	
	/// Runs in background TcpServerThread; Handles incomming TcpClient requests 	
	/// </summary> 	
	private void ListenForIncommingRequests()
	{
		try
		{
			// Create listener on localhost port 8052. 			
			tcpListener = new TcpListener(IPAddress.Parse("127.0.0.1"), 8052);
			tcpListener.Start();
			UnityEngine.Debug.Log("Server is listening");
			Byte[] bytes = new Byte[1024];
			while (connectedTcpClient == null)
			{
				using (connectedTcpClient = tcpListener.AcceptTcpClient())
				{
					// Get a stream object for reading 					
					using (NetworkStream stream = connectedTcpClient.GetStream())
					{
						int length;
						// Read incomming stream into byte arrary. 						
						while ((length = stream.Read(bytes, 0, bytes.Length)) != 0)
						{
							var incommingData = new byte[length];
							Array.Copy(bytes, 0, incommingData, 0, length);
							// Convert byte array to string message. 							
							string clientMessage = Encoding.ASCII.GetString(incommingData);
							//UnityEngine.Debug.Log("client message received as: " + clientMessage);
							string[] parsedMessage = clientMessage.Split(' ');
							// UnityEngine.Debug.Log(serverMessage);
							cmdVx = float.Parse(parsedMessage[0]);
							cmdVy = float.Parse(parsedMessage[1]);
							cmdVTheta = float.Parse(parsedMessage[2]);
							currState = int.Parse(parsedMessage[3]);
							errorMessage = int.Parse(parsedMessage[4]);
							UnityEngine.Debug.Log("cmdVx: " + cmdVx + " cmdVy: " + cmdVy + " cmdVtheta" + cmdVTheta + " currState: " + currState + " error: " + errorMessage);

						}
					}
				}
			}
		}
		catch (SocketException socketException)
		{
			connectedTcpClient.Close();
			UnityEngine.Debug.Log("SocketException " + socketException.ToString());
		}
	}

	/// <summary> 	
	/// Setup socket connection. 	
	/// </summary> 	
	private void ConnectToTcpServer()
	{
		try
		{
			clientReceiveThread = new Thread(new ThreadStart(ListenForData));
			clientReceiveThread.IsBackground = true;
			clientReceiveThread.Start();
		}
		catch (Exception e)
		{
			connectedTcpClient.Close();
			UnityEngine.Debug.Log("On client connect exception " + e);
		}
	}
	/// <summary> 	
	/// Runs in background clientReceiveThread; Listens for incomming data. 	
	/// </summary>     
	private void ListenForData()
	{
		try
		{
			socketConnection = new TcpClient("localhost", 8120);
			Byte[] bytes = new Byte[1024];
			while (true)
			{
				// Get a stream object for reading 				
				using (NetworkStream stream = socketConnection.GetStream())
				{
					int length;
					// Read incomming stream into byte arrary. 					
					while ((length = stream.Read(bytes, 0, bytes.Length)) != 0)
					{
						var incommingData = new byte[length];
						Array.Copy(bytes, 0, incommingData, 0, length);
						// Convert byte array to string message. 						
						string serverMessage = Encoding.ASCII.GetString(incommingData);
						//UnityEngine.Debug.Log("server message received as: " + serverMessage);

						string[] parsedMessage = serverMessage.Split(' ');
						// UnityEngine.Debug.Log(serverMessage);
						cmdVx = float.Parse(parsedMessage[0]);
						cmdVy = float.Parse(parsedMessage[1]);
						cmdVTheta = float.Parse(parsedMessage[2]);
						currState = int.Parse(parsedMessage[3]);
						errorMessage = int.Parse(parsedMessage[4]);
						UnityEngine.Debug.Log("cmdVx: " + cmdVx + " cmdVy: " + cmdVy + " cmdVtheta" + cmdVTheta + " currState: " + currState + " error: " + errorMessage);
					}
				}
			}
		}
		catch (SocketException socketException)
		{
			UnityEngine.Debug.Log("Socket exception: " + socketException);
			connectedTcpClient.Close();
		}
	}





	/// <summary> 	
	/// Send message to client using socket connection. 	
	/// </summary> 	
	private void SendMessage()
	{
		//if (connectedTcpClient == null)
		//{
		//	return;
		//}

		try
		{
            // Get a stream object for writing. 
			NetworkStream stream = connectedTcpClient.GetStream();
			if (stream.CanWrite)
			{
				int headersize = 75;
				char pad = ' ';
				//string serverMessage = "This is a message from your server.";
				UnityEngine.Debug.Log("sending Message");
				string serverMessage = transform.position.x.ToString() + ' ' + transform.position.z.ToString() + ' ' + theta.ToString()
					+ ' ' + initX.ToString() + ' ' + initY.ToString() + ' ' + initTheta.ToString() + ' ' + posPx.ToString() + ' '
					+ posPy.ToString() + ' ' + posPtheta.ToString() + ' ' + currTime.ToString() + ' ' + startTime.ToString() + ' '
					+ currState.ToString() + ' ' + input1.ToString() + ' ' + input1Time.ToString() + ' ' + input2.ToString() + ' ' + input2Time.ToString();
				string msglength = serverMessage.Length.ToString();
				string msg = msglength.PadRight(headersize, pad) + serverMessage;
				// Convert string message to byte array.                 
				byte[] serverMessageAsByteArray = Encoding.ASCII.GetBytes(msg);
				// Write byte array to socketConnection stream.               
				stream.Write(serverMessageAsByteArray, 0, serverMessageAsByteArray.Length);
				
				UnityEngine.Debug.Log("Server sent his message - should be received by client");
			}
		}
		// catch (SocketException socketException)
        catch
		{
			//connectedTcpClient.Close();
            // UnityEngine.Debug.Log("Socket exception: " + socketException);
            UnityEngine.Debug.Log("Socket exception");
        }
	}

	IEnumerator waiter()
	{
		//Wait for 4 seconds
		yield return new WaitForSeconds(5);
	}

	/// <summary> 	
	/// Setup socket connection. 	
	/// </summary> 	
	private void StartPythonClient()
	{
		try
		{
			PyClient = new Thread(new ThreadStart(OpenPy));
			PyClient.IsBackground = true;
			PyClient.Start();
		}
		catch (Exception e)
		{
			connectedTcpClient.Close();
			UnityEngine.Debug.Log("On client connect exception " + e);
		}
	}

	private void OpenPy()
	{
		try
		{
			ProcessStartInfo p = new ProcessStartInfo(@"C:\Users\jc3246\AppData\Local\Programs\Python\Python37\python.exe", scriptPath);
			p.UseShellExecute = false;
			p.RedirectStandardError = true;
			p.RedirectStandardOutput = true;

			Process proc = Process.Start(p);

			while (true)
			{
				var err = proc.StandardError.ReadToEnd();
				var msg = proc.StandardOutput.ReadToEnd();

				UnityEngine.Debug.Log(err);
				proc.Kill();
				proc.CloseMainWindow();
			}

		}
		catch (SocketException socketException)
		{
			connectedTcpClient.Close();
			UnityEngine.Debug.Log("Socket exception: " + socketException);
		}
	}

	private void CreateText(string userName, string data)
	{
		//Set the path of the file
		string path = specPath + userName + ".txt";

		// Create the file if it isnt there
		if (!File.Exists(path))
		{
			File.WriteAllText(path, "user " + userName + ": NRI Unity Experiment \n\n");
			File.AppendAllText(path, "Time, posXhuman, posYhuman, posThetaHuman, posXRobot, posYRobot, posThetaRobot \n");

		}
		// add the next line
		if (data != String.Empty)
		{
			File.AppendAllText(path, data + "\n");
		}

	}


}


//public class Global
//{
//	private static int count = 0;
//	private static float initialX = transform.position.x;
//	public static float initX = initialX;
//	public static float initY = transform.position.y;
//}
