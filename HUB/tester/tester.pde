import processing.serial.*;
import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;


ControlIO control;
int x,y,z,a,b,c,d,X,Y,NX,NY,But,LSTICK,D;
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
D=int(device.getHat("DPAD").getValue());



if(a==8){But=2;}
if(b==8){But=3;}
if(c==8){But=1;}
if(d==8){But=0;}

 if (z<0) 
 {
  
  print(But);
  println(D);
  delay(25);

}
}
