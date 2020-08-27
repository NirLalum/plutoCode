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
Result serialComunication(vector<double> thetasDouble, char LegNum, serialib& serial, int Rate) {
#ifdef __linux__
#define SERIAL_PORT "/dev/ttyACM0"
#endif


	// convert double type data to string so we can send it over serial comunication
	vector<string> sendThetasStr;
	int i = 0;
	// convert double array to string
	for (i = 0; i < thetasDouble.size(); i++) {
		sendThetasStr.push_back(to_string(thetasDouble[i]));
	}

	// send data to arduino
	cout << "start sending data" << endl;
	usleep(10000000/Rate); // Rate was 1000 and its good for parallel movement. wait between each message
	// send leg number id first: (1 = lf, 2 = rf, 3 = rb, 4 = lb )
	//serial.writeChar(state);
	//serial.writeChar('\n');
	serial.writeChar(LegNum);
	serial.writeChar('\n');
	for (i = 0; i < sendThetasStr.size(); i++) {
		if (!serial.writeString(sendThetasStr[i].c_str())) throw "faild to send data to arduino";
        serial.writeChar('\n'); // check if works without this line
		cout << sendThetasStr[i].c_str() << endl;
	}

    cout << "done seding data" << endl;
    //usleep(1000); //wait before sending the next theta
	// retrieve data from arduino
	vector<double> realThetasRetrieved;
	char buffer[10];
	char recievedString[10] = "";
	char ArduinoFeedback = '0';
	// Display ASCII characters (from 32 to 128)

	/*cout << "start retrieving data" << endl;
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
	cout << "done retrieving data" << endl; */
	// Close the serial device
    //serial.readString(recievedString, '\0', 3, 10000);

	//cout << "arduino feedback: " << recievedString << endl;
	//if(recievedString[0] == 'O') return SUCCESS;
	//else throw "comunication error!";
	return SUCCESS;
}
