using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;
using System.Threading;

public class GameManager : MonoBehaviour {

	public SerialPort arduinoPort;
	public Dropdown dpSerialPorts;
	public Slider slPhi;
	public Slider slTheta;
	public Slider slPsi;
	public Text textPhi;
	public Text textTheta;
	public Text textPsi;
	public Text textConnect;
	public Text textStatus;

	public InputField AccelData;
	public InputField GyroData;

	public Transform transformIMU;
	public Vector3 imuEullerAngles;



	public Quaternion imuQuat;
	public Vector3 imuAccel;
	public Vector3 imuGyro;

	public Quaternion oldimuQuat;
	public Vector3 oldimuAccel;
	public Vector3 oldimuGyro;

	public Vector3 earthGravity;
	public Vector3 actualAccel;

	public Queue<Vector3> accel_values;

	public Thread aquiringThread;
	public bool threadRunningFlag;
	public bool threadWorkingFlag;
	public Mutex accessControl;

	byte[] received_packet = new byte[20];

	public Queue<Vector3> accel_data;
	public Queue<Vector3> gyro_data;
	public Queue<Quaternion> quat_data;

	// Use this for initialization
	void Start () {
		quat_data = new Queue<Quaternion> (1024);
		gyro_data = new Queue<Vector3> (1024);
		accel_data = new Queue<Vector3> (1024);


		accel_values = new Queue<Vector3>();
	 	imuEullerAngles = new Vector3();
	 	earthGravity = new Vector3();
	 	actualAccel = new Vector3();
	 	imuQuat = new Quaternion();
		oldimuQuat = new Quaternion ();

		arduinoPort = new SerialPort("/dev/ttyS101", 115200);
		arduinoPort.ReadTimeout = 100;
		arduinoPort.ReadBufferSize = 4096;

		populateSerialPorts();
	
		aquiringThread = new Thread (new ThreadStart (ThreadRoutine));
		aquiringThread.Priority = System.Threading.ThreadPriority.Highest;

		accessControl = new Mutex ();
			
		SlidersPhi_OnValueChanged(0.0f);SlidersTheta_OnValueChanged(0.0f);SlidersPsi_OnValueChanged(0.0f);
	}

	// Update is called once per frame
	void Update () {
		
		//Get Data
		accessControl.WaitOne ();
		textStatus.text = "Fifo Count: " + quat_data.Count.ToString ();
		//quat_data.Count 
		while(quat_data.Count > 0) {
			imuQuat = quat_data.Dequeue ();
			imuAccel = accel_data.Dequeue ();
			imuGyro = gyro_data.Dequeue ();
		}
		accessControl.ReleaseMutex ();

		//Update Quaternion
		if (imuQuat != oldimuQuat) {
			oldimuQuat = imuQuat;
			transformIMU.rotation = imuQuat;
			updateSliders();
		}
		//Calulate Gyro AHRS

	}

	private void ThreadRoutine(){
		while (threadRunningFlag) {
			if (threadWorkingFlag) {
				readArduinoData ();
			}
		}
	}

	private void readArduinoData(){
		if (arduinoPort.BytesToRead > 0) {
			if (arduinoPort.ReadByte () == '$') {
				if (arduinoPort.Read (received_packet, 0, 20) == 20) {
					if (arduinoPort.ReadByte() == '\n') {
						disassemblePacket ();
					}
				}
			}
		}
	}

	public void disassemblePacket(){
		Quaternion q = new Quaternion ();
		Vector3 a = new Vector3 ();
		Vector3 g = new Vector3 ();
		q.w = (float) (received_packet [0] << 8 | received_packet [1]) / 16384.0f;
		q.x = (float) (received_packet [2] << 8 | received_packet [3]) / 16384.0f;
		q.y = (float) (received_packet [4] << 8 | received_packet [5]) / 16384.0f;
		q.z = (float) (received_packet [6] << 8 | received_packet [7]) / 16384.0f;

		q.w = q.w < 2? q.w:q.w-4;
		q.x = q.x < 2? q.x:q.x-4;
		q.y = q.y < 2? q.y:q.y-4;
		q.z = q.z < 2? q.z:q.z-4;
	
		a.x = (float) (received_packet [8] << 8 | received_packet [9]) / 8192.0f;
		a.y = (float) (received_packet [10] << 8 | received_packet [11]) / 8192.0f;
		a.z = (float) (received_packet [12] << 8 | received_packet [13]) / 8192.0f;

		a.x = a.x < 2? a.x:a.x-4;
		a.y = a.y < 2? a.y:a.y-4;
		a.z = a.z < 2? a.z:a.z-4;

		g.x = (float) (received_packet [14] << 8 | received_packet [15]) / 131.0f;
		g.x = (float) (received_packet [16] << 8 | received_packet [17]) / 131.0f;
		g.x = (float) (received_packet [18] << 8 | received_packet [19]) / 131.0f;

		g.x = g.x < 250? g.x:g.x-500;
		g.y = g.y < 250? g.y:g.y-500;
		g.z = g.z < 250? g.z:g.z-500;

		accessControl.WaitOne ();
		quat_data.Enqueue (q);
		accel_data.Enqueue (a);
		gyro_data.Enqueue (g);
		accessControl.ReleaseMutex ();
	}

	private void StartThread(){
		threadRunningFlag = true; //Running
		threadWorkingFlag = false; //but doing nothing
		aquiringThread.Start();
		Debug.Log ("Thread Started");
	}

	private void RunThread(){
		threadWorkingFlag = true; //doing something
		Debug.Log("Thread Running");
	}

	private void PauseThread(){
		threadWorkingFlag = false; //thread do not do what she used to do
		Debug.Log("Thread Paused");
	}

	private void StopThread(){
		threadWorkingFlag = false; //Dont do what she used to do
		threadRunningFlag = false; //Stop infinite lace
		Debug.Log("Thread Stopped");
	}


	private void populateSerialPorts(){
		//dpSerialPorts.AddOptions(SerialPort.GetPortNames());
		dpSerialPorts.ClearOptions();
		List<string> available_serial_ports = new List<string>();
		foreach (string available_serial_port in SerialPort.GetPortNames()){
			available_serial_ports.Add(available_serial_port);
        }
		dpSerialPorts.AddOptions (available_serial_ports);
		dpSerialPorts.value = 1;
	}

	public void btnConnect_Clicked(){
		Debug.Log("Botão Conexao Clicado.");
		if (arduinoPort.IsOpen) {
			Debug.Log ("Desconectando...");
			try {
				arduinoPort.Close ();
				textConnect.text = "Conectar";
			} catch (System.Exception ex) {
				Debug.Log ("Erro: " + ex.ToString ());
			}

		} else {
			arduinoPort.PortName = dpSerialPorts.options[dpSerialPorts.value].text.ToString();
			Debug.Log ("Conectando a Porta Serial: " 
				+ arduinoPort.PortName.ToString() 
				+ "|BaudRate: " 
				+ arduinoPort.BaudRate.ToString());
			try {
				arduinoPort.Open ();
				textConnect.text = "Desconectar";
			} catch (System.Exception ex) {
				Debug.Log ("Erro: " + ex.ToString ());
			}
		}


	}
	public void btnStart_Clicked(){
		Debug.Log("Botão Start Clicado.");
		arduinoPort.Write ("1");
		StartThread ();
		RunThread ();

	}
	public void btnStop_Clicked(){
		Debug.Log("Botão Stop Clicado.");
		arduinoPort.Write ("2");
		PauseThread ();
		StopThread ();
	}

	public void btnStep_Clicked(){
		Debug.Log("Botão Step Clicado.");
		/*
		actualAccel = accel_values.Dequeue();
		Debug.Log(Vector3.back);
		Debug.Log(actualAccel.normalized);
		imuQuat = Quaternion.FromToRotation(new Vector3(0.0f,0.0f,1.0f), actualAccel.normalized);
		transformIMU.rotation = imuQuat;
		updateSliders();
		*/
	}
	public void updateSliders(){
		imuEullerAngles = transformIMU.rotation.eulerAngles;
		slPhi.value = imuEullerAngles.x > 180 ? -360 + imuEullerAngles.x : imuEullerAngles.x;
		slTheta.value =  imuEullerAngles.y > 180 ? -360 + imuEullerAngles.y : imuEullerAngles.y;
		slPsi.value = imuEullerAngles.z > 180 ? -360 + imuEullerAngles.z : imuEullerAngles.z;
	}

	public void btnLoadData_Clicked(){
		Debug.Log("Botão Carregar Clicado.");
		/*
		string[] linhas = AccelData.text.Split('\n');
		AccelData.text = "";
		foreach(string linha_de_dados in linhas){
			//Debug.Log(linha_de_dados);
			string[] vector_values = linha_de_dados.Split();
			if(vector_values.Length == 3){
				Vector3 parsing_result = new Vector3();
				parsing_result.x = float.Parse(vector_values[0]);
				parsing_result.y = float.Parse(vector_values[1]);
				parsing_result.z = float.Parse(vector_values[2]);
				accel_values.Enqueue(parsing_result);
			}
		}
		Debug.Log(accel_values.Count);*/
	}

	public void btnRotate_Clicked(){
		Debug.Log("Botão Rotate Clicado.");
		Debug.Log("Slider Phi: " + slPhi.value.ToString());
		Debug.Log("Slider Theta: " + slTheta.value.ToString());
		Debug.Log("Slider Psi: " + slPsi.value.ToString());
		imuEullerAngles.x = slPhi.value;
		imuEullerAngles.y = slTheta.value;
		imuEullerAngles.z = slPsi.value;
		imuQuat = Quaternion.Euler(imuEullerAngles);
		transformIMU.rotation = imuQuat;

	}
	public void SlidersPhi_OnValueChanged(float sv){
		textPhi.text = "Phi: " + slPhi.value.ToString("F2") + " graus.";
	}
	public void SlidersTheta_OnValueChanged(float sv){
		textTheta.text = "Theta: " + slTheta.value.ToString("F2") + " graus.";
	}
	public void SlidersPsi_OnValueChanged(float sv){
		textPsi.text = "Psi: " + slPsi.value.ToString("F2") + " graus.";
	}
	public void DropdownSerialPorts_OnValueChanged(int dp_idx){
		Debug.Log("Seleção do Dropdown alterado para: " + dpSerialPorts.options[dpSerialPorts.value].text.ToString());
	}


}
