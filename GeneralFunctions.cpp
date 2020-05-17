#include "GeneralFunctions.H"

Result linspace(vector<double>& Vec, double InitVal, double FinVal, int stepsNum) {
	if (stepsNum < 2) return FAIL;
	int i = 1;
	Vec.push_back(InitVal);
	for (i = 1; i < stepsNum; i++) {
		Vec.push_back(Vec[i - 1] + (FinVal - InitVal) / (stepsNum - 1));
	}
	return SUCCESS;
}

// sending data and retrieving data (raspberypi <-> arduino)
vector<double> serialComunication(vector<double> thetasDouble) {
#ifdef __linux__
#define SERIAL_PORT "/dev/ttyACM0"
#endif

	// convert double type data to string so we can send it over serial comunication
	vector<string> sendThetasStr;
	int i = 0;
	for (i = 0; i < thetasDouble.size(); i++) {
		sendThetasStr.push_back(to_string(thetasDouble[i]));
	}

	// Serial object
	serialib serial;

	// Connection to serial port
	char errorOpening = serial.openDevice(SERIAL_PORT, 115200);
	// If connection fails, return the error code otherwise, display a success message
	if (errorOpening != 1) return errorOpening;
	printf("Successful connection to %s\n", SERIAL_PORT);

	// send data to arduino
	for (i = 0; i < sendThetasStr.size(); i++) {
		if (!serial.writeString(sendThetasStr[i].c_str())) throw "faild to send data to arduino";
		usleep(1000); //wait before sending the next theta
	}
	
	// retrieve data from arduino
	vector<double> realThetasRetrieved;
	char buffer[10];
	char recievedString[10] = "";
	// Display ASCII characters (from 32 to 128)
	while (1)
	{
		if (serial.readString(recievedString, '\0', 10, 1000) > 0){ ////////////// -------------- we need to define the size of the retrieved data.
			// if data was sent then put it inside the vector (after converting it to double)
			realThetasRetrieved.push_back(atof(recievedString));
		}
		if (realThetasRetrieved.size() == 3) { // if all 3 thetas were read from the serial exit loop
			break;
		}
		//usleep(1000);
	}
	// Close the serial device
	serial.closeDevice();
	return realThetasRetrieved;
}