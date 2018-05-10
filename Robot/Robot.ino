#include <AccelStepper.h>
#include <NewPing.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"



//============ Change these for each robot
// Robot 1: Good Wiring Robot
// Robot 2: Bad wiring Robot

#define robotnum 1          // Robot Number (1, 2, 3)
#define LF 9                // Left Motor Forward
#define LR 8                // Left Motor Reverse
#define RF 10               // Right Motor Reverse
#define RR 11               // Right  Motor Reverse
#define QTIsense1 46        // Front QTI
#define QTIsense2 3         // Back QTI
#define IR_F 44             // Front IR Sensor
#define IR_B 12             // Back IR Sensor
#define TRIGGER_PIN  2      // Front Distance Trigger Pin
#define ECHO_PIN 13         // Front Distance Echo Pin
#define MAX_DISTANCE 400    // Distance Sensor Max Distance
#define LE 18               // Left Encoder
#define RE 19               // Right Encoder
#define RED 26              // Red Led R2
#define GREEN 22            // Green Led R2
#define YELLOW 24           // YELLOW LED R2
#define S1 4                // PIN 1 for stepper
#define S2 5                // PIN 2 for stepper
#define S3 6                // PIN 3 for stepper
#define S4 7                // PIN 4 for stepper
#define FULLSTEP 4

//============ Change these based off Measured Values

#define QTI1Match 2500      // Value greater than floor but less than black tape for front QTI 
#define QTI2Match 500       // Value greater than floor but less than black tape for back QTI
#define MinDist 5           // Minimum Measurable Distance
#define LiftDist 320        // Distance Lift Box travels from top to bottom
#define IR_DELAY 30000
#define INC 2
#define AVG 20

int Sp = 1 * 255;         // Straight Speed limiting value to help encoders keep up
int TSp = .7 * 255;         // Turn Speed limiting value to help encoders keep up
int SPC = .95;              // Motor Catch up value if encoder is greater than other (<1)
int SPCI = 1;            // Motor Catch up value if encoder is less than other (>1)


bool DEBUG = true;     // Change this to true to enable debug (Printing to serial moniter) Caution slows down program

//============ Gyro and PID

#define MS 220        // Max Speed for driving forward
#define TS 200        // Max Turn Speed
#define AINC 1      // Incrementing value for Driving (0-255)
#define TINC 1      // Degree increment for turning

int UR=30;
int I=0;

int movespeed = 0;
int LEFT, RIGHT, yawDiff;
int yawDesired = 0;

double yawSetpoint, modifiedCurrentYaw, motorOffsetOutput;
double currentYaw;
double Kp = 3, Ki = 0, Kd = 0;
PID steeringPID(&modifiedCurrentYaw, &motorOffsetOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);
const int StabilizeSeconds = 15;
double initialPose = 0.0;

MPU6050 mpu;

const uint8_t InterruptPin = 2;
const uint8_t LedPin = GREEN;

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

const double RAD2DEG = 180.0 / M_PI;
const double DEG2RAD = M_PI / 180.0;


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void blink(uint8_t flashes, int pause) {
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


AccelStepper stepper1(FULLSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


int QTI1;
int QTI2;
int DesiredRobot;
int B;
int C;
int dist;
int DEBBUGER;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;

unsigned long TIME;
unsigned long LTIME;
unsigned long RTIME;

//============ Program Stage Checks

bool LIFT_POS = false;
bool LIFTED = false;
bool LIFT_BEGIN = false;
bool FIRST = true;
bool LIFT_COMPL = false;


//============

void setup() {


  Serial.begin(115200);
  Serial2.begin(9600);

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock

  // setup/initialization for the PID controller
  setYaw(0.0);
  steeringPID.SetOutputLimits(-MS, MS);  // **************** Check this ***************
  steeringPID.SetSampleTime(10);
  steeringPID.SetMode(AUTOMATIC);

  mpu.initialize();
  pinMode(InterruptPin, INPUT);

  Serial.println(F("Testing device connections..."));
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    // stay here, we can't do anything
    while (true) {
      blink(5, 50);
      delay(500);
    }
  }
  Serial.println(F("MPU6050 connection successful"));

  if (robotnum == 1) {
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



  Serial.println(F("Initializing DMP..."));
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
    attachInterrupt(digitalPinToInterrupt(InterruptPin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
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

  Serial.println(F("Stabilizing..."));
  for (uint8_t i = 0; i < StabilizeSeconds; i++) {
    digitalWrite(LedPin, HIGH);
    delay(500);
    digitalWrite(LedPin, LOW);
    delay(500);
  }

  pinMode(LF, OUTPUT);
  pinMode(LR, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RR, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);



  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(900.0);
  stepper1.setSpeed(900);
  stepper1.setCurrentPosition(0);

  motorOff();

  ledTest();


}



void loop() {

  DataReceive();
  controllerMap();
  motorMapping();

  if (DEBUG == true) {
    debug();
  }


}

////////////// Stages
//============

void StageAssign() {
  if (C == 24) {
    Lift();
  }

  if (C == 25) {
    Lower();
  }
}

//============
void Hub(int a1, int b1) {
  Serial2.print('<');
  Serial2.print(a1);
  Serial2.print(',');
  Serial2.print(b1);
  Serial2.println('>');
  delay(25);
}

//============

void Lift() {
  digitalWrite(GREEN, HIGH);
  stepper1.move(LiftDist);
  stepper1.runToPosition();
  digitalWrite(GREEN, LOW);
  LIFT_COMPL = true;
}

//============


void Lower() {
  stepper1.move(-LiftDist);
  stepper1.runToPosition();
  stepper1.run();

}

//============

////////////// Driving
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

void GYRO() {

  static bool firstTime = true;

  currentYaw = (double)ypr[0] * RAD2DEG;
  if (abs(currentYaw - yawSetpoint) > 180.0) {
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
      digitalWrite(GREEN, HIGH);

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      if (firstTime) {
        initialPose = (double)ypr[0] * RAD2DEG;
        firstTime = false;
      }

      digitalWrite(GREEN, LOW);

    }
  }
}

//============

void controllerMap() {

  switch (B) {
    case 2:               // Forward
      if (movespeed < MS) {
        movespeed += AINC;
      }
      else {
        movespeed = MS;
      }
      break;

    case 6:                 // Reverse
      if (movespeed < MS) {
        movespeed += AINC;
      }
      else {
        movespeed = MS;
      }
      break;

    case 4:                // Right
      movespeed = 0;
      setYaw(yawDesired);
      if(I==0){
      yawDesired += TINC;
      I=UR;
      }
      break;

    case 8:                // Left
      movespeed = 0;
      yawDesired -= TINC;
      setYaw(yawDesired);
      if(I==0){
      yawDesired -= TINC;
      I=UR;
      }
      
      break;

    case 0:                // Stop
      movespeed = 0;
      break;
  }
  I--;
}

//============

void motorMapping() {
  int LS, RS, LT, RT, LO, RO;
  int LB;             //last case state
  int margin = 2;

  GYRO();
  steeringPID.Compute();
  yawDiff = abs(yawSetpoint - modifiedCurrentYaw);



  switch (B) {

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
      LO = motorOffsetOutput;
      RO = -motorOffsetOutput;
      if (yawDiff < margin) {
        LT = 0;
        RT = 0;
      }
      break;

    case 8:               //LEFT
      LS = 0;
      RS = 0;
      LT = -TS;
      RT = TS;
      LO = -motorOffsetOutput;
      RO = motorOffsetOutput;
      if (yawDiff < margin) {
        LT = 0;
        RT = 0;
      }
      break;

    case 0:               //STOP
      LS = 0;
      RS = 0;
      LT = 0;
      RT = 0;
      LO = 0;
      RO = 0;
      LB = 0;
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

void motorReverseMapping() {

  if (B == 0) {               // Off
    motorOff();
  }

  if (DEBUG == true) {
    debug();
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

void QTICheck() {
  //  QTI1 = QTIVal(QTIsense1);
  QTI2 = QTIVal(QTIsense2);
  QTI1 = 0;
  if (DEBUG == true) {
    debug();
  }

  if (QTI1 < QTI1Match && QTI2 < QTI2Match)
  {
    digitalWrite(RED, LOW);
  }
  if (QTI1 >= QTI1Match && QTI2 < QTI2Match)
  {
    digitalWrite(RED, LOW);
  }
  if (QTI2 > QTI2Match)
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

long QTIVal(int sensorIn) {
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

void Distance() {
  dist = sonar.ping_cm();
  if (dist == 0) {
    dist = MAX_DISTANCE;
  }
  if (DEBUG == true) {
    debug();
  }
}

//============

void IR() {
  float d = pulseIn(IR_F, 0, IR_DELAY);
  float x = 1 / ((d / 100000) * 2);
  //  float c = pulseIn(IR_B, 0, IR_DELAY);
  //  float y = 1 / ((c / 1000000) * 2);

  if (x >= 210 && x <= 250) {
    digitalWrite(GREEN, HIGH);
    digitalWrite(YELLOW, LOW);
  }

  if (x >= 1 && x < 21 || x > 250) {
    digitalWrite(GREEN, LOW);
    digitalWrite(YELLOW, HIGH);
  }

  if (d == 0) {
  }
}

////////////// Sensors
//============

void debug() {
  Serial.print(yawSetpoint);
  Serial.print(',');
  Serial.print(modifiedCurrentYaw);
  Serial.print(',');
  Serial.print(yawDiff);
  Serial.print(',');
  Serial.print(motorOffsetOutput);
  Serial.print(',');
  Serial.print(LEFT);
  Serial.print(',');
  Serial.println(RIGHT);



  //  Serial.print(" | Robot: ");
  //  Serial.print(DesiredRobot);
  //  Serial.print(" | DPad: ");
  //  Serial.print(B);
  //  Serial.print(" | C: ");
  //  Serial.print(C);
  //  Serial.print(" | DEBUGGER: ");
  //  Serial.print(DEBBUGER);
  //  Serial.print(" | Dist: ");
  //  Serial.print(dist);
  //  Serial.print(" | Stepper: ");
  //  Serial.print(stepper1.currentPosition());
  // Serial.print(" | LE: ");
  //  Serial.print(LEVal);
  //  Serial.print(" | RE: ");
  //  Serial.print(REVal);

}

//============
void ledTest() {
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


void LEDDebug() {
  //  if (LEVal == REVal) {
  //    digitalWrite(RED, LOW);
  //    digitalWrite(GREEN, LOW);
  //    digitalWrite(YELLOW, LOW);
  //  }
  //
  //  if (LEVal > REVal) {
  //    digitalWrite(RED, HIGH);
  //    digitalWrite(GREEN, LOW);
  //    digitalWrite(YELLOW, LOW);
  //  }

}
////////////// Communication
//============

void DataReceive() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    newData = false;
  }
}

//============

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char StartMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial2.available() > 0 && newData == false) {
    rc = Serial2.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
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

    else if (rc == StartMarker) {
      recvInProgress = true;
    }
  }
}

//============

void parseData() {
  // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  DesiredRobot = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  B = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  C = atoi(strtokIndx);     // convert this part to a float

  if (C > 0) {
    StageAssign();
  }
}

