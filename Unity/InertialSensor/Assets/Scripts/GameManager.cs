using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;
using System.Threading;

public class GameManager : MonoBehaviour {

	public SerialPort arduinoPort;
	public Dropdown dpSerialPorts;
	public Dropdown dpQntSensores;
	public Slider slPhi;
	public Slider slTheta;
	public Slider slPsi;
	public Text textPhi;
	public Text textTheta;
	public Text textPsi;
	public Text textConnect;
	public Text textStatus;

	public InputField AccelData;
	public InputField GyroData;//novo

	public Transform transformIMUrwr;
	public Transform transformIMUrel;
	public Transform transformIMUtor;
	public Transform transformIMUlwr;
	public Transform transformIMUlel;

	public Vector3 imuEulerAngles;


	public List<Quaternion> imuQuat;
	public List<Vector3> imuAccel;
	public List<Vector3> imuGyro;

	public List<Quaternion> oldimuQuat;
	public List<Vector3> oldimuAccel;
	public List<Vector3> oldimuGyro;

	public Vector3 earthGravity;
	public Vector3 actualAccel;

	public Queue<Vector3> accel_values;

	public Thread aquiringThread;
	public bool threadRunningFlag;
	public bool threadWorkingFlag;
	public Mutex accessControl;

	byte[] received_packet = new byte[20];
	List<byte[]> sensor_packet;

	public List<Queue<Vector3>> accel_data;
	public List<Queue<Vector3>> gyro_data;
	public List<Queue<Quaternion>> quat_data;

	public List<Quaternion> offset_quat;

	public int qnt_sensores = 1;

	// Use this for initialization
	void Start () {
		sensor_packet = new List<byte[]> ();
		quat_data = new List<Queue<Quaternion>>(5);
		offset_quat = new List<Quaternion>(5);
		gyro_data = new List<Queue<Vector3>>(5);
		accel_data = new List<Queue<Vector3>>(5);

		imuQuat = new List<Quaternion> (5);
		imuAccel = new List<Vector3> (5);
		imuGyro = new List<Vector3> (5);

		oldimuQuat = new List<Quaternion> (5);
		oldimuAccel = new List<Vector3> (5);
		oldimuGyro = new List<Vector3> (5);

		for (int i = 0; i < 5; i++) {
			quat_data.Add (new Queue<Quaternion> (255));
			gyro_data.Add (new Queue<Vector3> (255));
			accel_data.Add (new Queue<Vector3> (255));
			imuQuat.Add (new Quaternion ());
			imuAccel.Add (new Vector3 ());
			imuGyro.Add (new Vector3 ());
			oldimuQuat.Add (new Quaternion ());
			oldimuAccel.Add (new Vector3 ());
			oldimuGyro.Add (new Vector3 ());
			sensor_packet.Add (new byte[8]);
			offset_quat.Add (Quaternion.identity);
		}
	

	 	imuEulerAngles = new Vector3();

		arduinoPort = new SerialPort("/dev/ttyS102", 115200);
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
		textStatus.text = "Fifo Count: " + quat_data[0].Count.ToString ();
		//quat_data.Count 
		for (int i = 0; i < 5; i++) {
			while (quat_data [i].Count > 0) {
				imuQuat[i] = quat_data [i].Dequeue ();
				//imuAccel[i] = accel_data [i].Dequeue ();
				//imuGyro[i] = gyro_data [i].Dequeue ();
			}
		}
		accessControl.ReleaseMutex ();


		//Update Quaternions
		if (qnt_sensores == 1) {
			if (imuQuat [0] != oldimuQuat [0]) {
				oldimuQuat [0] = imuQuat [0];
				transformIMUrwr.localRotation = imuQuat [0]* new Quaternion(
					-offset_quat [0].x,
					-offset_quat [0].y,
					-offset_quat [0].z,
					offset_quat [0].w);
				transformIMUrwr.localRotation = Quaternion.Euler(
					transformIMUrwr.localRotation.eulerAngles.x,
					-transformIMUrwr.localRotation.eulerAngles.y,
					-transformIMUrwr.localRotation.eulerAngles.z
				);
				updateSliders ();
			}
		}
		if (qnt_sensores >= 2) {
			if (imuQuat [1] != oldimuQuat [1]) {
				oldimuQuat [1] = imuQuat [1];
				transformIMUrel.localRotation = imuQuat [1]* new Quaternion(
					-offset_quat [1].x,
					-offset_quat [1].y,
					-offset_quat [1].z,
					offset_quat [1].w);
				updateSliders ();
			}
		}

		if (qnt_sensores >= 3) {
			if (imuQuat [2] != oldimuQuat [2]) {
				oldimuQuat [2] = imuQuat [2];
				transformIMUtor.localRotation = imuQuat [3]* new Quaternion(
					-offset_quat [4].x,
					-offset_quat [4].y,
					-offset_quat [4].z,
					offset_quat [4].w);
				updateSliders ();
			}
		}
		if (qnt_sensores >= 4) {
			if (imuQuat [3] != oldimuQuat [3]) {
				oldimuQuat [3] = imuQuat [3];
				transformIMUlel.localRotation = imuQuat [2]* new Quaternion(
					-offset_quat [2].x,
					-offset_quat [2].y,
					-offset_quat [2].z,
					offset_quat [2].w);
				updateSliders ();
			}
		}
		if (qnt_sensores >= 5) {
			if (imuQuat [4] != oldimuQuat [4]) {
				oldimuQuat [4] = imuQuat [4];
				transformIMUlwr.localRotation = imuQuat [4]* new Quaternion(
					-offset_quat [3].x,
					-offset_quat [3].y,
					-offset_quat [3].z,
					offset_quat [3].w);
				updateSliders ();
			}
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
		/*
		 * UART_ST = 0x7F  			#Start transmission
		 * UART_ET = 0x7E				#End transmission
		*/
		/*
		if (arduinoPort.BytesToRead > 0) {
			if (arduinoPort.ReadByte () == '$') {
				if (arduinoPort.Read (received_packet, 0, 20) == 20) {
					if (arduinoPort.ReadByte() == '\n') {
						disassemblePacket ();
					}
				}
			}
		}
		*/
		int qnt_received = 0;
		if (arduinoPort.BytesToRead > 0) {
			if (arduinoPort.ReadByte () == '$') {
				for (int i = 0; i < 5; i++) {
					qnt_received += arduinoPort.Read (sensor_packet[i], 0, 8);
				}
				if (qnt_received == 40) {
					if (arduinoPort.ReadByte() == '\n') {
						disassemblePacket ();
					}
				}
			}
		}
	}


	/*
	Summary: Method for reading packages
		Standard MPU6050 package contains:
		Sensor data is read from DMP
		1st byte: Start transmission = '$'
			Next 8 bytes, each measure is composed by two bytes (MSB first)
	Quaternion from sensor1 (w,x,y,z)
		...
		End Byte = '\n'
	*/
	public void disassemblePacket(){
		/*
		Quaternion q = new Quaternion ();
		Vector3 a = new Vector3 ();
		Vector3 g = new Vector3 ();
		q.w = (float) (received_packet [0] << 8 | received_packet [1]) / 16384.0f;
		q.x = (float) (received_packet [2] << 8 | received_packet [3]) / 16384.0f;
		q.z = (float) (received_packet [4] << 8 | received_packet [5]) / 16384.0f;
		q.y = (float) (received_packet [6] << 8 | received_packet [7]) / 16384.0f;

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
		for (int i = 0; i < 5; i++) {
			quat_data[i].Enqueue (q);
			accel_data[i].Enqueue (a);
			gyro_data[i].Enqueue (g);
		}
		accessControl.ReleaseMutex ();
		*/
		Quaternion q = new Quaternion ();
		for (int i = 0; i < 5; i++) {
			q.w = (float) (sensor_packet[i] [0] << 8 | sensor_packet[i] [1]) / 16384.0f;
			q.z = (-1.0f) * (float) (sensor_packet[i] [2] << 8 | sensor_packet[i] [3]) / 16384.0f;
			q.x = (float) (sensor_packet[i] [4] << 8 | sensor_packet[i] [5]) / 16384.0f;
			q.z = (float) (sensor_packet[i] [6] << 8 | sensor_packet[i] [7]) / 16384.0f;
			q.w = q.w < 2? q.w:q.w-4;
			q.x = q.x < 2? q.x:q.x-4;
			q.y = q.y < 2? q.y:q.y-4;
			q.z = q.z < 2? q.z:q.z-4;
			accessControl.WaitOne ();
			quat_data[i].Enqueue (q);
			accessControl.ReleaseMutex ();
		}
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
		imuQuat = Quaternion.FromTolocalRotation(new Vector3(0.0f,0.0f,1.0f), actualAccel.normalized);
		transformIMUrwr.localRotation = imuQuat;
		updateSliders();
		*/
	}
	public void updateSliders(){
		imuEulerAngles = transformIMUrwr.localRotation.eulerAngles;
		slPhi.value = imuEulerAngles.x > 180 ? -360 + imuEulerAngles.x : imuEulerAngles.x;
		slTheta.value =  imuEulerAngles.y > 180 ? -360 + imuEulerAngles.y : imuEulerAngles.y;
		slPsi.value = imuEulerAngles.z > 180 ? -360 + imuEulerAngles.z : imuEulerAngles.z;
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
		imuEulerAngles.x = slPhi.value;
		imuEulerAngles.y = slTheta.value;
		imuEulerAngles.z = slPsi.value;
		imuQuat[0] = Quaternion.Euler(imuEulerAngles);
		transformIMUrwr.localRotation = imuQuat[0];

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
	public void DropdownQntSensores_OnValueChanged(int dp_idx){
		Debug.Log("Seleção do Dropdown alterado para: " + dpQntSensores.options[dpQntSensores.value].text.ToString());
		qnt_sensores = dpQntSensores.value + 1;
	}
	public void btnSetOffset_Clicked(){
		offset_quat [0] = transformIMUrwr.rotation;
		offset_quat [1] = transformIMUrel.rotation;
		offset_quat [2] = transformIMUlel.rotation;
		offset_quat [3] = transformIMUlwr.rotation;
		offset_quat [4] = transformIMUtor.rotation;

		for (int i = 0; i < 5; i++) {
			offset_quat[i] = offset_quat[i] * Quaternion.identity;
		}
		/*
		newRot = quaternion.product(_quaternion,quaternion.conjugate(self.quaternion))
			self.quaternion = quaternion.product(_quaternion,quaternion.conjugate(self.quaternion_offset))
			self.rotquaternion = self.quaternion
		*/
		//quaternion.product(_quaternion, quaternion.conjugate(quat_desejado))

	}

}
