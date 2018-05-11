#include <NewPing.h>

#define robotnum = 1     // Robot Number (1, 2, 3)
#define QTIsense1 46     // Front QTI
#define QTIsense2 3      // Back QTI
#define IR_F 44          // Front IR Sensor
#define IR_B 12          // Back IR Sensor
#define TRIGGER_PIN  2   // Front Distance Trigger Pin
#define ECHO_PIN 13      // Front Distance Echo Pin
#define MAX_DISTANCE 400 // Distance Sensor Max Distance
#define LE 32            // LE
#define RE 30            // RE
#define IR_DELAY 200000  // Amount of time the IR Sensors wait to receive signal


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);

}
float x;
float d;

void loop() {

  int QTI1 = QTIVal(QTIsense2);
  IR();
  Serial.print(x);
  Serial.print(',');
  Serial.println(QTI1);
  delay(25);

}

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


void IR() {
  d = pulseIn(IR_F, HIGH, IR_DELAY);
  x = 1 / ((d / 1000000) * 2);
}

