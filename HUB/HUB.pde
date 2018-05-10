import processing.serial.*;
import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;


ControlIO control;
int x,y,z,a,b,c,d,X,Y,NX,NY,But,LSTICK,RSTICK,START,D,STAT;
ControlDevice device;


import processing.serial.*;

Serial myPort;  // Create object from Serial class
public static final char HEADER    = 'H';
public static final char A_TAG = 'M';
public static final char B_TAG = 'X';

void setup()
{
  size(512, 512);
  ControlButton button;
  ControlHat hat;
  ControlSlider slider;
  control = ControlIO.getInstance(this);
  device = control.getMatchedDevice("ArdCont");

  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
}

void draw(){
y=-1*int(map(device.getSlider("LSTICKY").getValue(), 0,1,0,255));
x=int(map(device.getSlider("LSTICKX").getValue(),0,1,0,255));
z=int(map(device.getSlider("TRIGGERS").getValue(),0,1,0,255));
a=int(device.getButton("A").getValue());
b=int(device.getButton("B").getValue());
c=int(device.getButton("X").getValue());
d=int(device.getButton("Y").getValue());
LSTICK=int(device.getButton("LSTICK").getValue());
RSTICK=int(device.getButton("RSTICK").getValue());
START=int(device.getButton("START").getValue());
D=int(device.getButton("DPAD").getValue());

STAT=0;
if(a==8){But=2;}
if(b==8){But=3;}
if(c==8){But=1;}
if(d==8){But=0;}
if(START==8){STAT=24;};
if(LSTICK==8){STAT=25;};
if(D==0){D=1;};
if(RSTICK==8){D=0;};


 if (z<0) 
 {
  
  print(But);
  print(",");
  print(D);
  print(",");
  println(STAT);
 
  sendMessage(A_TAG, But,D,STAT);
  }
}

//void serialEvent(Serial p) {
//  // handle incoming serial data
//  String inString = myPort.readStringUntil('\n');
//  if(inString != null) {     
//    print( inString );   // echo text string from Arduino
//  }
//}

//void mousePressed() {
//  sendMessage(A_TAG, But,b,c,d);
//}

void sendMessage(char tag, int a, int b, int c){
  // send the given index and value to the serial port
  myPort.write(HEADER);
  myPort.write(tag);
  myPort.write((char)(a / 256)); // msb
  myPort.write(a & 0xff);  //lsb
  myPort.write((char)(b / 256)); // msb
  myPort.write(b & 0xff);  //lsb
  myPort.write((char)(c / 256)); // msb
  myPort.write(c & 0xff);  //lsb

}
