
#include <mcp_can.h>
#include <SPI.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif

// no joystick deifne
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned char len = 0;
unsigned char buf[8]= {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char can_msg[8]= {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/// Value Limits Definition /// need to validate with molex.
const float P_MIN = -15.2*2*PI; 
const float P_MAX = 15.2*2*PI; // AFTER CHECKING. BUT NEED TO CHECK ON THE SETUP MENU (MOLEX) - also, need to check the issue of 12 bits encoder.
const float V_MIN = -45.0; 
const float V_MAX = 45.0; // CHECKED
const float KP_MIN = 0.0; 
const float KP_MAX = 500.0; // NEED TO CHECK
const float KD_MIN = 0.0;
const float KD_MAX = 5.0; // NEED TO CHECK
const float T_MIN = -18.0;
const float T_MAX = 18.0; // NEED TO CHECK
const float I_MIN = -40.0;
const float I_MAX = 40.0; // NEED TO CHECK

// variables for commands and echo
float p_float = 0;
float v_float = 0;
float i_float = 0;

uint16_t p_int = 0;
uint16_t v_int = 0;
uint16_t i_int = 0;
char idRecived = 0;

// desired angles for drawing a circle:

// --------------- Convertion Functions ------------------------- //

// converts angle to int (for motor input)
unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  
  return ((float)(x - offset) / span * (pow(2,bits)-1)) ;

}

// converts int to angle (for motor output)
float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int bitspan = (pow(2,bits)-1);
  
return  ((float)(x_int * span / bitspan + offset)); //
}
// --------------------------------------------------------------- //

// Initial Command - choose command in physical units

float floatpos = 0; //[rad]
float floatvel = 0; //[rad/s] . "initilized" to  +-45.
float floatkp = 5;//[Nm/rad] . "initilized" to 0 to 500 NM/rad . need to check im molex.
float floatkd = 0.2; //[rad/s] .   0 to 5 NMs/rad .  need to check im molex.
float floatff = 0.0; //[Nm] . -18 [NM] to +18 [NM] 

  
uint16_t pos = float_to_uint( floatpos , P_MIN , P_MAX , 16 ); // 16 bit 
uint16_t vel = float_to_uint( floatvel , V_MIN , V_MAX , 12 ); // 12 bit 
uint16_t kp = float_to_uint( floatkp , KP_MIN , KP_MAX , 12 );  // 12 bit 
uint16_t kd = float_to_uint( floatkd , KD_MIN , KD_MAX , 12 ); // 12 bit
uint16_t ff = float_to_uint( floatff , T_MIN , T_MAX , 12 );// 12 bit

unsigned long t;
// int index = 0;

void getMsg()
{
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
     idRecived = buf[0];
     p_int = (buf[1]<<8)|buf[2];
     v_int = (buf[3]<<4)|(buf[4]>>4);
     i_int = ((buf[4]&0xF)<<8)|buf[5];

     p_float = uint_to_float(p_int,P_MIN, P_MAX, 16);
     v_float = uint_to_float(v_int,V_MIN, V_MAX, 12);
     i_float = uint_to_float(i_int,I_MIN, I_MAX, 12);
    
      unsigned long canId = CAN.getCanId();
//        SERIAL.print(idRecived,DEC);
//        SERIAL.print(",");
//        SERIAL.println(p_float, DEC);
//        SERIAL.print(",");
//        Serial.println(t);
//        SERIAL.println(v_float, DEC);
//        SERIAL.print(",");
//        SERIAL.println(i_float, DEC);
    }
}

class Motor
{
  public:
    Motor(int id,float kp, float kd, float ff, float WorkingPoint);
    void move(float ff);
    void enterMotorMode();
    void setZeroPosition();
    void exitMotorMode();
    float getWorkingPoint();
    float getOldFloatVal();
    void setOldFloatVal(float oldFloatVal);
    private:
    int id;
    float desired_pos;
    //float floatkp;
    //float floatkd;
    unsigned int IntKp;
    unsigned int IntKd;
    unsigned int intFF; 
    float WorkingPoint_; // initial value of the joint according to simulation
    float oldFloatVal_; // last position of motor 
};

Motor::Motor(int id, float kp, float kd, float ff, float WorkingPoint)
{
  this->id = id;
  //floatkp=kp;
  //floatkd=kd;
  IntKp = float_to_uint(kp,KP_MIN,KP_MAX,12);
  IntKd = float_to_uint(kd,KD_MIN,KD_MAX,12);
  intFF = float_to_uint(ff , T_MIN , T_MAX , 12);// 12 bit
  WorkingPoint_ = WorkingPoint;
  oldFloatVal_ = WorkingPoint;
}

float Motor::getWorkingPoint(){
  return WorkingPoint_;
}

float Motor::getOldFloatVal(){
  return oldFloatVal_;
}

void Motor::setOldFloatVal(float oldFloatVal){
  oldFloatVal_ = oldFloatVal;
}

void Motor::move(float pos)
{
   unsigned int IntPos = float_to_uint(pos, P_MIN, P_MAX, 16);
   //unsigned int IntKp = float_to_uint(floatkp,KP_MIN,KP_MAX,12);
   //unsigned int IntKd = float_to_uint(floatkd,KD_MIN,KD_MAX,12); 
   can_msg[0] = IntPos>>8;                                       
   can_msg[1] = IntPos&0xFF;
   can_msg[2] = vel>>4;
   can_msg[3] = ((vel&0xF)<<4)|(IntKp>>8);
   can_msg[4] = IntKp&0xFF;
   can_msg[5] = IntKd>>4;
   can_msg[6] = ((IntKd&0xF)<<4)|(ff>>8);
   can_msg[7] = intFF&0xFF;
   CAN.sendMsgBuf(this->id, 0, 8, can_msg);  
//   delay(10);
}

void Motor::enterMotorMode()
{
    stmp[0] = 0xFF;
    stmp[1] = 0xFF;
    stmp[2] = 0xFF;
    stmp[3] = 0xFF;
    stmp[4] = 0xFF;
    stmp[5] = 0xFF;
    stmp[6] = 0xFF;
    stmp[7] = 0xFC;
   
    if (CAN.sendMsgBuf(this->id, 0, 8, stmp)==CAN_FAIL)
    {
      //SERIAL.println("MSG_FAIL  ENTER MOTOR MODE ID = ");
    }
    else
    {
//      SERIAL.print("MSG_SENT - ENTER MOTOR MODE ID =  ");
//      SERIAL.println(this->id);
    }
//    delay(10);
}

void Motor::setZeroPosition()
{
    stmp[0] = 0xFF; //Zero Position Sensor - sets the mechanical position to zero.
    stmp[1] = 0xFF;
    stmp[2] = 0xFF;
    stmp[3] = 0xFF;
    stmp[4] = 0xFF;
    stmp[5] = 0xFF;
    stmp[6] = 0xFF;
    stmp[7] = 0xFE;
    if (CAN.sendMsgBuf(this->id, 0, 8, stmp)==CAN_FAIL)
    {
      //SERIAL.println(" MSG_FAIL - ZERO ENCODER ID = ");
      //SERIAL.println(this->id);
      //Serial.println();
      //Serial.println();
    }
    else
    {
//      SERIAL.print("MSG_SENT - ZERO ENCODER ID =  ");
//      SERIAL.println(this->id);
//      Serial.println();
//      Serial.println();
    }
    delay(10);
}

void Motor::exitMotorMode()
{
    stmp[0] = 0xFF; //Exit Motor Mode 
    stmp[1] = 0xFF;
    stmp[2] = 0xFF;
    stmp[3] = 0xFF;
    stmp[4] = 0xFF;
    stmp[5] = 0xFF;
    stmp[6] = 0xFF;
    stmp[7] = 0xFD;
    if (CAN.sendMsgBuf(this->id, 0, 8, stmp)==CAN_FAIL)
    {
//      SERIAL.println(" MSG_FAIL - EXIT MOTOR MODE ID = ");
//      SERIAL.println(this->id);
//      Serial.println();
//      Serial.println();
    }
    else
    {
//      SERIAL.print("MSG_SENT - EXIT MOTOR MODE ID =  ");
//      SERIAL.println(this->id);
//      Serial.println();
//      Serial.println();
    }
    //delay(10);
}

// motors defenition:
// LfLeg motors
/*
float kp_1st = 5;
float kp_2nd = 5;
float kp_3rd = 5;
float kd_1st = 0.5;
float kd_2nd = 0.7;
float kd_3rd = 0.3;
Motor motor1(1, kp_1st, kd_1st, -1.5707); // id,kp,kd % kp was 5
Motor motor2(2, kp_2nd, kd_2nd, 4.725);
Motor motor3(3, kp_3rd, kd_3rd, -2.424);
// RfLeg motors:
Motor motor4(4, kp_1st, kd_1st, 1.5707); // id,kp,kd
Motor motor5(5, kp_2nd, kd_2nd, -1.584);
Motor motor6(6, kp_3rd, kd_3rd, 2.424);
// RbLeg motors
Motor motor7(7, kp_1st, kd_1st, -1.5707); // id,kp,kd
Motor motor8(8, kp_2nd, kd_2nd, 2.301);
Motor motor9(9, kp_3rd, kd_3rd, 2.424);
// LbLeg motors
Motor motor10(10, kp_1st, kd_1st, 1.5707); // id,kp,kd
Motor motor11(11, kp_2nd, kd_2nd, 0.8404);
Motor motor12(12, kp_3rd, kd_3rd, -2.424);
*/

float kp_1st = 80; // was 80
float kp_2nd = 40; // was 40
float kp_3rd = 30; // was 30

float kd_1st = 1.7; // was 1.7
float kd_2nd = 1.2; // was 1.2
float kd_3rd = 1.0; // was 1

float ff_1st = 0; // was 0
float ff_2nd = 0; // was 0
float ff_3rd = 0; // was 0i

// update working points according to i command ---------------------------------------------------
Motor motor1(1, kp_1st, kd_1st, ff_1st,-1.5707); // id,kp,kd
Motor motor2(2, kp_2nd, kd_2nd, ff_2nd,4.725);
Motor motor3(3, kp_3rd, kd_3rd, ff_3rd,-2.424);
// RfLeg motors:
Motor motor4(4, kp_1st, kd_1st, ff_1st ,1.5707); // id,kp,kd,ff,workingPoint 
Motor motor5(5, kp_2nd, kd_2nd, ff_2nd,-1.584);
Motor motor6(6, kp_3rd, kd_3rd, ff_3rd, 2.424);
// RbLeg motors
Motor motor7(7, kp_1st, kd_1st, ff_1st,-1.5707); // id,kp,kd
Motor motor8(8, kp_2nd, kd_2nd, ff_2nd,2.301);
Motor motor9(9, kp_3rd, kd_3rd, ff_3rd,2.424);
// LbLeg motors
Motor motor10(10, kp_1st, kd_1st, ff_1st,1.5707); // id,kp,kd
Motor motor11(11, kp_2nd, kd_2nd, ff_2nd,0.7329);
Motor motor12(12, kp_3rd, kd_3rd, ff_3rd,-2.393);

Motor MotorArray[12] = {motor1, motor2, motor3, motor4, motor5, motor6, motor7, motor8, motor9, motor10, motor11, motor12};
// Arduino start:


// comucication part --------------


const byte numChars = 32;
char receivedChars[numChars]; // an array to store the received data

boolean newData = false;

int i = 0;
float floatVal[4] = {0 ,0, 0, 0}; // id, 3 thetas
float oldFloatVals[3] = {0, 0, 0};
void recvWithEndMarker() {
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;
 //Serial.println(Serial.available());
 // if (Serial.available() > 0) {
  //Serial.print("time recvWithEndMarker begin : ");
  //Serial.println(millis());
  
  while (Serial.available() > 0 && newData == false) {
    //Serial.println("read data");
    rc = Serial.read();
    //Serial.println(rc);
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
    //Serial.print("time recvWithEndMarker ended : ");
    //Serial.println(millis());
  
}

void showNewData() {
 if (newData == true) {
  //Serial.print("This just in ... ");
  //Serial.println(receivedChars);
  newData = false;
  floatVal[i] = atof(receivedChars);
  //Serial.println("float value: ");
  //Serial.println(floatVal[i], 5);
  i++; 
  }
}

void sendData(){
  char c[3] = "OK";
  //Serial.write(c,3);
  //Serial.println("c value: ");
  //Serial.println(c);
}


// -------- setup and loop

void setup() {
    Serial.begin(115200);
    delay(10);
    while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 1000k
    {
        //SERIAL.println("CAN BUS Shield init fail");
        //SERIAL.println(" Init CAN BUS Shield again");
        delay(10);
    }
//    SERIAL.println("CAN BUS Shield init ok!");
//    SERIAL.println("Setup over");
//    SERIAL.println();


//    motor1.exitMotorMode();
//    motor1.enterMotorMode();
//    motor1.setZeroPosition();
//
//    motor2.setZeroPosition();
//    motor2.exitMotorMode();
//    motor2.enterMotorMode();
//
//    motor3.setZeroPosition();
//    motor3.exitMotorMode();
//    motor3.enterMotorMode();

  // initialize motors
  //delay(1000*3); // delay for wating the motors to start 
  int j = 0;
  for(j=0; j<12; j++){
    MotorArray[j].setZeroPosition();
    delay(10);
    MotorArray[j].exitMotorMode();
    delay(10);
    //MotorArray[j].setZeroPosition();
    delay(10);
    MotorArray[j].enterMotorMode();
    delay(10);
    //MotorArray[j].setZeroPosition();
    delay(10);
    MotorArray[j].move(0);
    delay(10);
  }
  
    
  delay(10);
}

void loop() {
  recvWithEndMarker();
  showNewData();
  int j = 0;
  
  //if(floatVal[0] == 'g'){

  //}
  
  // start engine function here (only if float val is full)
  if ( floatVal[0] != 0 && floatVal[1] != 0 && floatVal[2] != 0 && floatVal[3] != 0){
    // choose which motors to start according to the leg sent from raspbery
    if((int)(floatVal[0]) == 1) j = 0;
    if((int)(floatVal[0]) == 2) j = 3;
    if((int)(floatVal[0]) == 3) j = 6;
    if((int)(floatVal[0]) == 4) j = 9;
    
    //Serial.println(j);
      
    if (abs(MotorArray[j].getOldFloatVal() - floatVal[1]) < 1 && abs(MotorArray[j+1].getOldFloatVal()- floatVal[2]) < 1 && abs(MotorArray[j+2].getOldFloatVal() - floatVal[3]) < 1){
      MotorArray[j].move(-(floatVal[1] - MotorArray[j].getWorkingPoint()));
      //Serial.println(j);
      //Serial.println(-(floatVal[1] - MotorArray[j].getWorkingPoint()));
      //Serial.print("time end motor1 move : ");
      //Serial.println(millis());
      MotorArray[j].setOldFloatVal(floatVal[1]);
      //getMsg();
      MotorArray[j+1].move(-(floatVal[2] - MotorArray[j+1].getWorkingPoint()));
      //Serial.print("time end motor2 move : ");
      //Serial.println(millis());
      MotorArray[j+1].setOldFloatVal(floatVal[2]);
      //getMsg();
      MotorArray[j+2].move(-1.33*(floatVal[3] - MotorArray[j+2].getWorkingPoint())); // gear ratio 1.33
      //Serial.print("time end motor3 move : ");
      //Serial.println(millis());
      MotorArray[j+2].setOldFloatVal(floatVal[3]);
      //getMsg();  
    }
  }
  //else{
    //MotorArray[j].move(0);
    //getMsg();
//    MotorArray[j+1].move(0);
//    getMsg();
//    MotorArray[j+2].move(0);
//    getMsg();
//    }
  if (i == 4){
    for (i=0; i<4; i++){
      floatVal[i] = 0;
    }
    i = 0;
    sendData(); // sending the thetas reached according to the encoder
  }
  //delay(10);
  //int t = millis();
  //Serial.print("time: ");
  //Serial.println(t);
}
