#include <Servo.h>

Servo servos[8]; // create servo object array to control 8 servos
/*
Servo indexing is as follows:
0 = Index_Intermediate; 
1 = Index_Proximal;
2 = Middle_Intermediate;
3 = Middle_Proximal;
4 = Ring_Intermediate;
5 = Ring_Proximal;
6 = Pinkie_Intermediate;
7 = Pinkie_Proximal;
*/

//Mux variables
int s0 = 4;
int s1 = 5;
int s2 = 6;
int s3 = 7;
int SIG_pin = A3;

const int sizeAngle = 12;
const int sizeCollisionArray = 8; // Receive the truncated contact array from the HEDAS python script via serial
bool newData = false;
/*
The indexing for the truncated contact array is as following, where 1 = Proximal, 2 = Intermediate, 3 = Distal phalanges:

0 = index1 x
1 = index2
2 = middle1 x
3 = middle2
4 = pinky1 x
5 = pinky2
6 = ring1 x
7 = ring2
*/

float angles[sizeAngle];
int collision[sizeCollisionArray];

//Servo positions to engage/disengage pawls
int disengage = 90;
int engage = 180;

int muxChannel[16][4]={
  {0,0,0,0}, //channel 0
  {1,0,0,0}, //channel 1
  {0,1,0,0}, //channel 2
  {1,1,0,0}, //channel 3
  {0,0,1,0}, //channel 4
  {1,0,1,0}, //channel 5
  {0,1,1,0}, //channel 6
  {1,1,1,0}, //channel 7
  {0,0,0,1}, //channel 8
  {1,0,0,1}, //channel 9
  {0,1,0,1}, //channel 10
  {1,1,0,1}, //channel 11
  {0,0,1,1}, //channel 12
  {1,0,1,1}, //channel 13
  {0,1,1,1}, //channel 14
  {1,1,1,1}  //channel 15
};

void setup(){
  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  servos[0].attach(2);  // attaches the servo on pin 9 to the servo object
  servos[1].attach(3);
  servos[2].attach(8);
  servos[3].attach(9);
  servos[4].attach(10);
  servos[5].attach(11);
  servos[6].attach(12);
  servos[7].attach(13);
  
  Serial.begin(115200);
    
}


void checkForNewData () {
    if (Serial.available() >= sizeCollisionArray && newData == false) {

      for(int j = 0; j < sizeCollisionArray; j++)
      {
        collision[j] = Serial.read();
        
        if (collision[j] == 1){
          servos[j].write(engage);
        }
        else{
          servos[j].write(disengage);
        }
        newData = true;
      }
    }
}

void loop(){
 //-------Finger joints----------//
  for(int i = 0; i<4;i++){
      int channel = i*3+2;
      // PIP joints
      angles[i] = readMux(channel);
      // MCP x joints
      angles[i+4] = readMux(channel+1);
      // MCP z joints
      angles[i+8] = readMux(channel+2);
  }
  
// -Printing on the serial monitor for Python to receive the data-//
 for(int i = 0; i < sizeAngle-1; i++)
 {
   Serial.print(angles[i]);Serial.print(" ");
 }
 Serial.println(angles[sizeAngle-1]);
 if (newData == true) {
  newData = false;
 }

// //Checking PIPs
//   Serial.print(angles[0]); Serial.print(" ");
//   Serial.print(angles[1]); Serial.print(" ");
//   Serial.print(angles[2]); Serial.print(" ");
//   Serial.print(angles[3]); Serial.print(" ");
// //  Serial.println(angles[3]);

// // Checking MPCx
//   Serial.print(angles[4]); Serial.print(" ");
//   Serial.print(angles[5]); Serial.print(" ");
//   Serial.print(angles[6]); Serial.print(" ");
//   Serial.print(angles[7]); Serial.print(" ");
// //  Serial.println(angles[7]);

// //Checking MCPz
//   Serial.print(angles[8]); Serial.print(" ");
//   Serial.print(angles[9]); Serial.print(" ");
//   Serial.print(angles[10]); Serial.print(" ");
//   Serial.println(angles[11]);
//   delay(1000);

// Wait to receive collision array from python via serial
  checkForNewData();
}

//functions
int readMux(int channel){
  int controlPin[] = {s0, s1, s2, s3};
  //loop through the 4 sig
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the SIG pin
  int val = analogRead(SIG_pin);

  //return the value
  return val;
}