#include <AccelStepper.h>
#include <NewPing.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"



//============ Change these for each robot
// Robot 1: Good Wiring Robot
// Robot 2: Bad wiring Robot
// Robot 3: Best Wiring Robot

#define robotnum 2          // Robot Number (1, 2, 3)
#define LF 9                // Left Motor Forward
#define LR 8                // Left Motor Reverse
#define RF 10               // Right Motor Reverse
#define RR 11               // Right  Motor Reverse
#define QTIsense1 46        // Front QT
#define QTIsense2 3         // Back QTI
#define IR_F 44             // Front IR Sensor
#define IR_B 12             // Back IR Sensor
#define RED 26              // Red Led R2
#define GREEN 22            // Green Led R2
#define YELLOW 24           // YELLOW LED R2
#define S1 4                // PIN 1 for stepper
#define S2 5                // PIN 2 for stepper
#define S3 6                // PIN 3 for stepper
#define S4 7                // PIN 4 for stepper
#define FULLSTEP 4

//============ Change these based off Measured Values

#define QTI1Match 900     // Value greater than floor but less than black tape for Robot 1 QTI 
#define QTI2Match 800     // Value greater than floor but less than black tape for Robot 2 QTI
#define QTI3Match 1000    // Value greater than floor but less than black tape for Robot 2 QTI 
#define MinDist 5         // Minimum Measurable Distance
#define LiftDist 320      // Distance Lift Box travels from top to bottom
#define IR_DELAY 200000   // Amount of time pulseIn will wait before declaring that there is no IR signal

int QTIMatch;

int Sp = 1 * 255;        // Straight Speed limiting value to help encoders keep up
int TSp = .7 * 255;      // Turn Speed limiting value to help encoders keep up
int SPC = .95;           // Motor Catch up value if encoder is greater than other (<1)
int SPCI = 1;            // Motor Catch up value if encoder is less than other (>1)


bool DEBUG = false;     // Change this to true to enable debug (Printing to serial moniter) Caution slows down program

//============ Gyro and PID

#define MS 210      // Max Speed for driving forward
#define TS 170      // Max Turn Speed
#define AINC 1      // Incrementing value for Driving (0-255)
#define TINC 1      // Degree increment for turning

int movespeed = 0;          // Base value written to motors
int LEFT, RIGHT, yawDiff;   // Variables for controlling motors
int yawDesired = 0;         // Desired heading for the robot

double yawSetpoint, modifiedCurrentYaw, motorOffsetOutput;  // PID controller desired value, current value, output
double currentYaw;
double Kp = 3, Ki = 0, Kd = 0.5;                            // PID controller valuees
PID steeringPID(&modifiedCurrentYaw, &motorOffsetOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);
const int StabilizeSeconds = 15;                            // Settle time for the gyro
double initialPose = 0.0;                                   // initial position for the gyro

MPU6050 mpu;

const uint8_t InterruptPin = 2;                             // Gyro interupt pin
const uint8_t LedPin = GREEN;                               // Gyro code debug set to led green

boolean CDIR = false;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

const double RAD2DEG = 180.0 / M_PI;    // Radian to degree
const double DEG2RAD = M_PI / 180.0;    // Degree to radian

volatile bool mpuInterrupt = false;     // Indicates whether MPU interrupt pin has gone high
void dmpDataReady() {                   // Declares interupt true if values are ready to be read
  mpuInterrupt = true;
}

void blink(uint8_t flashes, int pause) {    // Function for led debugging of gyro code
  for (uint8_t i = 0; i < flashes; i++) {
    digitalWrite(LedPin, HIGH);
    delay(pause);
    digitalWrite(LedPin, LOW);
    delay(pause);
  }
}


//============ Global Values

#define motorPin1  S1     // IN1 on the ULN2003 driver 1
#define motorPin2  S2     // IN2 on the ULN2003 driver 1
#define motorPin3  S3     // IN3 on the ULN2003 driver 1
#define motorPin4  S4     // IN4 on the ULN2003 driver 1
#define FULLSTEP 4


AccelStepper stepper1(FULLSTEP, motorPin1, motorPin3, motorPin2, motorPin4);  // Initializes the stepper control

int QTI1;           // Front QTI
int QTI2;           // Back QTI
int DesiredRobot;   // Robot # that the hub is wanting to control
int B;              // Driving commands from hub value
int C;              // Other commands from hub

// Variables for hub Communication
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;


//============ Program Stage Checks

//bool LIFT_POS = false;
//bool LIFTED = false;
//bool LIFT_BEGIN = false;
//bool FIRST = true;
bool LIFT_COMPL = false;  // Has the robot lifted its forklift
bool DES_FRONT = false;   // Is this the designated front robot

//============

void setup() {

  Serial.begin(38400);    // Initialize serial for debugging
  Serial2.begin(9600);    // Initialize serial for xbee

  Wire.begin();          // I2C for gyro
  Wire.setClock(400000); // 400kHz I2C clock

  // setup/initialization for the PID controller
  setYaw(0.0);
  steeringPID.SetOutputLimits(-MS, MS);  // Output Limits for PID Controller
  steeringPID.SetSampleTime(10);         // Sample time for PID controller
  steeringPID.SetMode(AUTOMATIC);        // PID controllers calculation mode

  mpu.initialize();                      // Initialize the Gyro
  pinMode(InterruptPin, INPUT);          // Set gyro interrupt pin to input

  Serial.println(F("Testing device connections..."));   // Testing connection to Gyro
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    // stay here, we can't do anything
    while (true) {
      blink(5, 50);
      delay(500);
    }
  }
  Serial.println(F("MPU6050 connection successful"));

  if (robotnum == 1) {              // Setting Gyro offset limits depending on which robot is being uploaded
    mpu.setXAccelOffset(-426);
    mpu.setYAccelOffset(71);
    mpu.setZAccelOffset(1193);
    mpu.setXGyroOffset(164);
    mpu.setYGyroOffset(-23);
    mpu.setZGyroOffset(9);
  }

  if (robotnum == 2) {
    mpu.setXAccelOffset(-4957);
    mpu.setYAccelOffset(-430);
    mpu.setZAccelOffset(1765);
    mpu.setXGyroOffset(-103);
    mpu.setYGyroOffset(-51);
    mpu.setZGyroOffset(-79);
  }

  if (robotnum == 3) {
    mpu.setXAccelOffset(-1918);
    mpu.setYAccelOffset(-2408);
    mpu.setZAccelOffset(2009);
    mpu.setXGyroOffset(-87);
    mpu.setYGyroOffset(-17);
    mpu.setZGyroOffset(-8);
  }



  Serial.println(F("Initializing DMP..."));   // Begin Communication with GYRO
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection on pin "));
    Serial.print(InterruptPin);
    Serial.println(F("..."));
    attachInterrupt(digitalPinToInterrupt(InterruptPin), dmpDataReady, RISING);   // Attach interupt to Gyro pin
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {                                                      // Account for error in GYRO communication
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    while (true) {
      blink(10, 50);
      delay(500);
    }
  }

  Serial.println(F("Stabilizing..."));                // Delay rest of program for GYRO settling time
  for (uint8_t i = 0; i < StabilizeSeconds; i++) {
    digitalWrite(LedPin, HIGH);
    delay(500);
    digitalWrite(LedPin, LOW);
    delay(500);
  }

  // Set motor pins and LED pins to outputs
  pinMode(LF, OUTPUT);
  pinMode(LR, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RR, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);


  // Setup stepper control values
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(900.0);
  stepper1.setSpeed(900);
  stepper1.setCurrentPosition(0);

  motorOff();  // Ensure values to motors are at zero to begin with

  ledTest();        // Blink the LEDs to show Robot ready
  GYRO();           // Call the Gyro for first time to initialize it and set initial position
  Hub(1, robotnum); // Tell Hub that Robot is ready to start

}

//============

void loop() {
  GYRO();                     // Call Gyro function to get currentheading value
  DataReceive();              // Check for information from HUB
  controllerMap();            // Set the movespeed values based off of value B from hub

  if(LIFT_COMPL==false){  // Checks QTI sensors and responds accordingly to returned values
    QTICheck();
  }

  if (DES_FRONT == false && DesiredRobot == robotnum) { // Normal motor control if robot is not front robot and desiredrobot is current robot
    motorMapping();
  }

  if (DES_FRONT == false && LIFT_COMPL == true) { // Normal motor control if robot is not front robot and lifting of forklift is complete
    motorMapping();
  }

  if (DES_FRONT == true && LIFT_COMPL == true) {  // Reverse motor control if lifitng is complete and robot is designated front robot
    reverseMotorMapping();
  }

  if (DEBUG == true) {    // Calls debug function
    debug();
  }


}

////////////// Stages
//============

void StageAssign() {  // Calls functions or changes boolean values based of received value C
  if (DesiredRobot == robotnum && C == 1 ) {  // Hub command to reset desired heading to current heading
    GYRO();
    yawSetpoint = modifiedCurrentYaw;
  }

  if (DesiredRobot == robotnum && C == 12 ) { // HUB command to call IR checking function
    IR();
  }

  if ((DesiredRobot == robotnum && C == 24) || (DesiredRobot == 0 && C == 24)) {  // Hub command to lift forklift only called if desiredrobot=robotnum or desiredrobot =0
    Lift();
  }

  if ((DesiredRobot == robotnum && C == 25) || (DesiredRobot == 0 && C == 25)) {  // Hub command to lower forklift only called if desiredrobot=robotnum or desiredrobot =0
    Lower();
  }

  if (C == 10) {                              // Set lift complete value to true
    LIFT_COMPL = true;
    digitalWrite(GREEN, HIGH);
  }

  if (C == 11 && DesiredRobot == robotnum) {  // Assign robot to be front robot
    DES_FRONT = true;
    digitalWrite(YELLOW, HIGH);
  }
}


//============

void Lift() {                   // Initializes lifting mechanism to begin lift forklift
  digitalWrite(GREEN, HIGH);     
  stepper1.move(LiftDist);      
  stepper1.runToPosition();
  digitalWrite(GREEN, LOW);
  LIFT_COMPL = true;
}

//============


void Lower() {                // Initializes lifting mechanism to begin lower forklift     
  stepper1.move(-LiftDist);
  stepper1.runToPosition();
  stepper1.run();

}


////////////// Driving
//============

void controllerMap() {    // Sets movespeed values based off of controller input values
  GYRO();
  switch (B) {
    case 2:                // Forward
      movespeed = MS;
      break;

    case 6:                // Reverse
      movespeed = MS;
      break;

    case 4:                // Right
      movespeed = 0;
      break;

    case 8:                // Left
      movespeed = 0;
      break;

    case 0:                // Stop
      movespeed = 0;
      break;
  }

}

//============

void motorMapping() {
  int LS, RS, LT, RT, LO, RO;

  GYRO();                                           // Calls GYRO function to get current heading
  steeringPID.Compute();                            // Calls PID controller to calculate motor offset value

  switch (B) {
  /*  Sets motor values based off of B value so forward, backward, left, right.
   *  LS,RS are for straight lines (forward backward)
   *  LT,RT are the turn motor speeds
   *  LO,RO are the motoroffset values from the PID Controller
   */

    case 2:               // Forward
      LS = movespeed;
      RS = movespeed;
      LT = 0;
      RT = 0;
      LO = motorOffsetOutput;
      RO = -motorOffsetOutput;
      break;

    case 6:               // Reverse
      LS = -movespeed;
      RS = -movespeed;
      LT = 0;
      RT = 0;
      LO = -motorOffsetOutput;
      RO = motorOffsetOutput;
      break;

    case 4:               //Right
      LS = 0;
      RS = 0;
      LT = TS;
      RT = -TS;
      LO = 0;
      RO = 0;
      yawSetpoint = modifiedCurrentYaw;
      break;

    case 8:               //LEFT
      LS = 0;
      RS = 0;
      LT = -TS;
      RT = TS;
      LO = 0;
      RO = 0;
      yawSetpoint = modifiedCurrentYaw;
      break;

    case 0:               //STOP
      LS = 0;
      RS = 0;
      LT = 0;
      RT = 0;
      LO = 0;
      RO = 0;
      break;
  }

  LEFT = LS + LT + LO;      // Value to be written to left motor
  RIGHT = RS + RT + RO;     // Value to be written to right motor

  /* Sets the output pin based off of LEFT,RIGHT value
   * if postitive value is sent to corresponding forward pin
   * if negative value is sent to corresponding reverse pin
   */
  
  if (LEFT > 0) {
    analogWrite(LF, LEFT);
    analogWrite(LR, 0);
  }

  if (RIGHT > 0) {
    analogWrite(RF, RIGHT);
    analogWrite(RR, 0);
  }
  if (LEFT < 0) {
    analogWrite(LR, abs(LEFT));
    analogWrite(LF, 0);
  }
  if (RIGHT < 0) {
    analogWrite(RR, abs(RIGHT));
    analogWrite(RF, 0);
  }
  if (LEFT == 0) {
    analogWrite(LR, 0);
    analogWrite(LF, 0);
  }
  if (RIGHT == 0) {
    analogWrite(RR, 0);
    analogWrite(RF, 0);
  }


}

//============

void reverseMotorMapping() {
  /*  Same exact function as MotorMapping();
   *  except it reverses forward and backward
   */
  

  int LS, RS, LT, RT, LO, RO;

  GYRO();
  steeringPID.Compute();
  yawDiff = abs(yawSetpoint - modifiedCurrentYaw);

  switch (B) {

    case 6:               // Forward
      LS = movespeed;
      RS = movespeed;
      LT = 0;
      RT = 0;
      LO = motorOffsetOutput;
      RO = -motorOffsetOutput;
      break;

    case 2:               // Reverse
      LS = -movespeed;
      RS = -movespeed;
      LT = 0;
      RT = 0;
      LO = -motorOffsetOutput;
      RO = motorOffsetOutput;
      break;

    case 4:               //Right
      LS = 0;
      RS = 0;
      LT = TS;
      RT = -TS;
      LO = 0;
      RO = 0;
      yawSetpoint = modifiedCurrentYaw;
      break;

    case 8:               //LEFT
      LS = 0;
      RS = 0;
      LT = -TS;
      RT = TS;
      LO = 0;
      RO = 0;
      yawSetpoint = modifiedCurrentYaw;
      break;

    case 0:               //STOP
      LS = 0;
      RS = 0;
      LT = 0;
      RT = 0;
      LO = 0;
      RO = 0;
      break;
  }

  LEFT = LS + LT + LO;
  RIGHT = RS + RT + RO;

  if (LEFT > 0) {
    analogWrite(LF, LEFT);
    analogWrite(LR, 0);
  }

  if (RIGHT > 0) {
    analogWrite(RF, RIGHT);
    analogWrite(RR, 0);
  }
  if (LEFT < 0) {
    analogWrite(LR, abs(LEFT));
    analogWrite(LF, 0);
  }
  if (RIGHT < 0) {
    analogWrite(RR, abs(RIGHT));
    analogWrite(RF, 0);
  }
  if (LEFT == 0) {
    analogWrite(LR, 0);
    analogWrite(LF, 0);
  }
  if (RIGHT == 0) {
    analogWrite(RR, 0);
    analogWrite(RF, 0);
  }

}


//============


void motorOff() {
  analogWrite(LF, 0);                      // turn off left motor
  analogWrite(LR, 0);                      // turn off right motor
  analogWrite(RF, 0);                      // turn off left motor
  analogWrite(RR, 0);                      // turn off right motor
  delay(10);
}

////////////// Sensors
//============

void setYaw(double setpoint) {
  // computes a real yaw from a relative yaw
  // cleans up angle issues
  yawSetpoint = setpoint + initialPose;
  if (yawSetpoint > 180.0)
    yawSetpoint -= 360.0;
  else if (yawSetpoint < -180.0)
    yawSetpoint += 360.0;
}

//============

void GYRO() {
  static bool firstTime = true;

  if (mpuInterrupt || (fifoCount >= packetSize)) {

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();


    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));


      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      //digitalWrite(RED,HIGH);
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.resetFIFO();


      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      if (firstTime) {
        initialPose = (double)ypr[0] * RAD2DEG;
        setYaw(modifiedCurrentYaw);
        firstTime = false;
      }

    }
  }
  currentYaw = (double)ypr[0] * RAD2DEG;        // Pulls the yaw value from received gyro data
  if (abs(currentYaw - yawSetpoint) > 180.0) {  // Sets it in terms of relative yaw
    if (currentYaw >= 0) {
      modifiedCurrentYaw = currentYaw - 360.0;
    }
    else {
      modifiedCurrentYaw = currentYaw + 360.0;
    }
  }
  else {
    modifiedCurrentYaw = currentYaw;
  }
}

//============
void QTICheck() {   // Checks QTI Sensors
  //  QTI1 = QTIVal(QTIsense1); // Calls the QTI Read function for front QTI Sensor
  QTI2 = QTIVal(QTIsense2); // Calls the QTI Read function for back QTI Sensor
  
  // Sets the compared QTI value to read value based off of robot num
  if (1 == robotnum) {  
    QTIMatch = QTI1Match;
  }

  if (2 == robotnum) {
    QTIMatch = QTI2Match;
  }

  if (3 == robotnum) {
    QTIMatch = QTI3Match;
  }

  if (DEBUG == true) {
    debug();
  }

  if (QTI2 > QTIMatch)      // Turns off motors and freezes robot if black line is detected
  {
    digitalWrite(RED, HIGH);
    motorOff();
    boolean stop = true;
    while (stop == true) {
      delay(100);
    }
  }
}

//============

long QTIVal(int sensorIn) { // Reads QTI sensor values
  long duration = 0;
  pinMode(sensorIn, OUTPUT);
  digitalWrite(sensorIn, HIGH);
  delay(1);
  pinMode(sensorIn, INPUT);
  digitalWrite(sensorIn, LOW);
  while (digitalRead(sensorIn)) {
    duration++;
  }
  return duration;
}

//============

void IR() { // Checks the IR Sensor
  float d = pulseIn(IR_F, HIGH, IR_DELAY);  //Reads IR Sensor
  float x = 1 / ((d / 1000000) * 2);

  // Turns on LEDs based off of returned values
  if (x >= 9.5 && x <= 10.5) {
    digitalWrite(GREEN, HIGH);
    digitalWrite(YELLOW, LOW);
  }

  if (x >= 1 && x < 9.5) {
    digitalWrite(GREEN, LOW);
    digitalWrite(YELLOW, HIGH);
  }

  if (d == 0) {
    digitalWrite(GREEN, LOW);
    digitalWrite(YELLOW, LOW);
  }
}

////////////// Sensors
//============

void debug() { // Prints values to serial monitor for debugging
  Serial.print(yawSetpoint);
  Serial.print(',');
  Serial.print(modifiedCurrentYaw);
  Serial.print(',');
  Serial.print(yawDiff);
  Serial.print(',');
  Serial.print(motorOffsetOutput);
  Serial.print(',');
  Serial.print(B);
  Serial.print(',');
  Serial.print(LEFT);
  Serial.print(',');
  Serial.println(RIGHT);


}

//============
void ledTest() {  // Blinks all three leds as a test and to show that the Robot is ready
  digitalWrite(GREEN, HIGH);
  delay(100);
  digitalWrite(GREEN, LOW);
  delay(100);
  digitalWrite(YELLOW, HIGH);
  delay(100);
  digitalWrite(YELLOW, LOW);
  delay(100);
  digitalWrite(RED, HIGH);
  delay(100);
  digitalWrite(RED, LOW);
}

////////////// Communication
//============

void DataReceive() {  
  recvWithStartEndMarkers();          // Checks the xbee for data
  if (newData == true) {              // if the data is new
    strcpy(tempChars, receivedChars); // Copies data
    parseData();                      // Parses the data into variables
    newData = false;                  // Resets new data varaible
  }
}

//============

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char StartMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial2.available() > 0 && newData == false) {   // Pulls all data from the xbee
    rc = Serial2.read();

    if (recvInProgress == true) {   // Reads data until endmarker is detected '>'
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {  // Resets index varible if too high
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == StartMarker) {   // declares that receiving data
      recvInProgress = true;
    }
  }
}

//============

void parseData() {
  // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part Desired Robot
  DesiredRobot = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  B = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  C = atoi(strtokIndx);     

  if (C > 0) {
    StageAssign();
  }
}

//============
void Hub(int a1, int b1) {  // Function to send values to hub
  Serial2.print('<');
  Serial2.print(a1);
  Serial2.print(',');
  Serial2.print(b1);
  Serial2.println('>');
  delay(25);
}

