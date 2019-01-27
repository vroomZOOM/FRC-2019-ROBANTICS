#include <Pixy2.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>




const int gainPot = A0;
const int led = 11;

boolean autoRun;

Pixy2 pixy;

Servo panServo;
Servo tiltServo;

int tiltangle = 90;
int panangle = 90;
int sendval;


void setup() {

Wire.begin(1);
Wire.onReceive(gotRate);
 Wire.onRequest(requestEvent);
pinMode(led, OUTPUT);
  
  pixy.init();

  panServo.attach(10);
  tiltServo.attach(9);
  panServo.write(90);
  tiltServo.write(85);

  Serial.begin(9600);

}

void loop() {


  int x;
  int y;
  float angleDif;
  float angleYDif;

  int gainAdj = map(analogRead(gainPot), 0, 1024, 0, 30);


  int xMove = pixy.ccc.blocks[x].m_x;
  int yMove = pixy.ccc.blocks[y].m_y;
  int blocks = pixy.ccc.getBlocks();

  if (blocks == 1){

  sendval = xMove;

  }
Serial.println(sendval);
/*
  int gain = abs(158 - xMove);
  int autoGain = map(gain, 0, 158, 0, gainAdj);

  int Ygain = abs(104 - yMove);
  int YautoGain = map(Ygain, 0, 104, 0, gainAdj);

  int value = Wire.read();

 //  Serial.println(blocks);

  if (value == 1) {
    digitalWrite(7, HIGH);
    autoRun = true;

  }

  if (value == 0) {
    digitalWrite(7, LOW);
    autoRun = false;

  }



 

  if (blocks == 1) {

    angleDif = map(xMove, 0, 316, -autoGain, autoGain);

    panServo.write(panangle + angleDif);

    angleYDif = map(yMove, 0, 208, -YautoGain, YautoGain);

    tiltServo.write(tiltangle - angleYDif);

    tiltangle = tiltangle - angleYDif;

    panangle = panangle + angleDif;

    panangle = constrain(panangle, 15, 170);

    tiltangle = constrain(tiltangle, 75, 100);


  }

  else {

    panServo.write(90);
    tiltServo.write(85);
  }
*/
}
void gotRate() {
  if (Wire.available()) {

   // delay(1000);


  }
}
void requestEvent() {
  Wire.write(sendval); // respond with message of 6 bytes
  // as expected by master
}



