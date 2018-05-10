// BinaryDataFromProcessing
// These defines must mirror the sending program:
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
  if ( Serial.available() >= TOTAL_BYTES)
  {
    if ( Serial.read() == HEADER)
    {
      char tag = Serial.read();
      if (tag == A_TAG)
      {
        //Collect integers
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
//============

void SendData(int Aa, int Bb, int Cc) {
  Serial1.print('<');
  Serial1.print(Aa);
  Serial1.print(',');
  Serial1.print(Bb);
  Serial1.print(',');
  Serial1.print(Cc);
  Serial1.println('>');
}


//============



