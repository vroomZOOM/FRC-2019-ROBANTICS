#include <Pixy2.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>





const int led = 3;
int sendymabob[2]; // Creates an array to store data. In this case, it creates 2 variables to store data


Pixy2 pixy;

Servo panServo;
Servo tiltServo;

int p;




void setup() {

  Wire.begin(1);
  Wire.onReceive(gotRate);
  Wire.onRequest(requestEvent);
  pinMode(led, OUTPUT);

  pixy.init();



  Serial.begin(19200);

}

void loop() {


  int x;

  float angleDif;



//digitalWrite(led, HIGH);


  int xMove = pixy.ccc.blocks[x].m_x;

  int blocks = pixy.ccc.getBlocks();



  int gain = abs(158 - xMove);
  int autoGain = map(gain, 0, 158, 0, 30);



  
    int value = Wire.read();
    
Serial.println(value);
    if (value == 1) {
      digitalWrite(led, HIGH);

    }
    if (value == 0) {
      digitalWrite(led, LOW);

    }
  
  if (blocks == 1) {
    angleDif = map(xMove, 0, 316, -autoGain, autoGain);

sendymabob[0] = angleDif; // Stores angleDif into the first section of the sendymabob array
sendymabob[1] = 1; // Stores a 1 into the second section of the sendymabob array. In this case, it is enabled.

    

  }

  else {

    sendymabob[1] = 0; // Otherwise, sendymabob[1] is disabled
    
//Serial.println("no ball");
  }
}
void gotRate() {
  if (Wire.available()) {

   int value = Wire.read();

     delay(2);


  }
}

  void requestEvent() {
  Wire.write(sendymabob[0]);
  Wire.write(sendymabob[1]);// respond with message of 6 bytes
  // as expected by master
  }


