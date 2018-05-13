

const char HEADER       = 'H';
const char A_TAG    = 'M';
const char B_TAG    = 'X';
const int  TOTAL_BYTES  = 8; // the total bytes in a message
int X;
int Y;
int RED = 44;
int GREEN = 42;
int YELLOW = 40;

int A;
int B;

int INST=0;
int TEST=3;

bool R1_IN_POS = false;
bool R2_IN_POS = false;
bool R1_LIFTED = false;
bool R2_LIFTED = false;
bool LIFT_COMP = false;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
}

void loop() {
  DataReceive();
  
  if ( Serial.available() >= TOTAL_BYTES) // Receives data from processing and sends to SendData()
  {
    if ( Serial.read() == HEADER)
    {
      char tag = Serial.read();
      if (tag == A_TAG)
      {
        //Collect integers
        //Converts from ASCII to int
        int a = Serial.read() * 256;
        a = a + Serial.read();
        int b = Serial.read() * 256;
        b = b + Serial.read();
        int c = Serial.read() * 256;
        c = c + Serial.read();

 
        SendData(a, b, c);
  
      }
    }
  }
}

void StageAssign(){
  if(A==1){ // Prints to Processing that the robot is ready
    Serial.print("Robot #");
    Serial.print(B);
    Serial.println(" is ready to begin!");
  }
}
//============

void SendData(int Aa, int Bb, int Cc) { // Sends data packet to robots
  Serial1.print('<');
  Serial1.print(Aa);
  Serial1.print(',');
  Serial1.print(Bb);
  Serial1.print(',');
  Serial1.print(Cc);
  Serial1.println('>');
}


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

  strtokIndx = strtok(tempChars, ",");     // get the first variable 
  A = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  B = atoi(strtokIndx);     // convert this part to an integer
  

  if (A > 0) {
    StageAssign();
  }
}


