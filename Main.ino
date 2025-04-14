/*
   Project Name: Train Collision Avoidance System (TCAS)
   Author(s): Aryan Chavan, Benjamin Ponka, Krish Patel, Namra Patel
   Date Started: 2025-03-24            Submission Date: 2025-04-17
   Version: V.1.00
   Description: Second year Mechatronics Engineering Diploma Project in which a train collision avoidance system is made with basic sensors(IR, sonar), actuators(Servos) and microcontroller(Arduino Mega).
   Licence:
   Links:
   Development Board: Arduino Mega 2560
*/

// Dependancies
#include <Servo.h>


#define LEDPIN 13  // indicator for the ir beams( if beam is broken the light turns on)


// Setting up the Ir Pins
int IRPins[15] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};


// Variables related to the IR sensor.
// Try to put this in an array.
int sensor_state[15][3]; // each row corresponds to each sensor. cols: current state, last state, on track number.
int nonPairedSensors[] = {3, 4, 5, 6, 7, 8, 15};

// Variables for the servos
int pos[] = {0, 30, 60, 90, 120, 150, 180};  // all of the positions we would wan the servo to be in.
int servoPos[] = {0,0,0,0,0,0,0,0};          // current/live servo positions.

// Creating servo objects
Servo myservo;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
Servo myservo7;
Servo myservo8;


// Train variables
int trainDirection1 = 0;  // 1 is left, 0 is right
int trainDirection2 = 1;  // 1 is left , 0 is right

// Track Variables
int trackStates[] = {0,0,0,0,0,0};   // each of the elements corresponds to the track segments 0 being the outer most track in section 1 and 6 being the outer track in section 2.
int trainOnTrack[] = {0,0,0,0,0,0};  // keeps a track of which train is on which track.(since the direction of a train cannot be changed. 0 = no train, 1 = train1, 2 = train2

void setup() {
  // initialize the LED pin as an output:
  pinMode(LEDPIN, OUTPUT);

  IRInit();                             // initialize the sensor pins as inputs and setting them to HIGH.
  attachServos();                       // attaching the pins of the servos to the object servo in code
  zeroServo();                          // zero all of the servo
  zeroIRStates();

  Serial.begin(9600);                 // Serial monitor
}


int count = 0; // to set the inital directions of the train.
// main loop.
void loop() {
  for (int i = 0; i < 15; i++) {        // loop to read the sensors.
    sensor_state[i][0] = IRPins[i];     // storing the value from the sensor to the state variable.
  }
  for (int i = 0;  i < 15; i++) {       // loop to check if any sensor was triggered.
      if (!nonPaired(i)) {              // if it is a paired sensors
        currentTrack = sensor_state[i][2];  // setting the triggered sensors track number to the variable.
        if(trackStates[currentTrack] == 0){  // if it is 0 no train in on the track.
          servoNum = servoNumCheck(i);             // checks which servo is connected to the Ir sensor. (If multiple use data of which train in on)
          //do a check of the servo positions.
          // if the position of the servo is open for the track currently being checked then dont do anything,
          // if servo position is set to a different track check if that track is being used if not do nothing
          // if it is being used; change servo position in the servo position array.
          // update tracks under use.
          //           
        }
      }
      else if(nonPaired(i)){            // if it is a non paired sensor.
        
      }

      count += 1;
    }
  }
  changeServoPos();// change servo position to the ones in the array.


}


// assigns and initialises the ir sensors.
void IRInit() {
  for (int i = 0; i < 15; i++) {
    pinMode(IRPins[i], INPUT_PULLUP);
  }
}

// assigns 0 as value to the sensor state array.
void zeroIRStates() {
  for (int i = 0; i < 15; i++) {
    digitalWrite(sensor_state[i][0], 0);
    digitalWrite(sensor_state[i][1], 0);
    // sets the track numbers of each of the sensors.
    if(i == 1 || i == 2 || i == 3 || i == 8 || i==9 || i == 10){
      digitalWrite(sensor_state[i][2], 1);  
    }
    else if(i == 4 || i == 5 || i == 6 || i == 7){
      digitalWrite(sensor_state[i][2], 2);
    }
    else if(i == 15){
      digitalWrite(sensor_state[i][2], 3);
    }
    else if(i == 11 || i == 12 || i == 13 || i==14){
      digitalWrite(sensor_state[i][2], 4);
    }
  }

  for (int i = 0; i < 15; i++) {  // prints the array.
    for (int j = 0; j < 3; j++) {
      Serial.print(sensor_state[i][j]);
      Serial.print(" "); // Delimiter between elements
    }
  }
  
}

// attach the servos to the digital pins.
void attachServos() {
  myservo.attach(2);
  myservo2.attach(3);
  myservo3.attach(4);
  myservo4.attach(5);
  myservo5.attach(6);
  myservo6.attach(7);
  myservo7.attach(8);
  myservo8.attach(9);
}

// zero all of the servos at once.
void zeroServo() {
  myservo.write(0);
  myservo2.write(0);
  myservo3.write(0);
  myservo4.write(0);
  myservo5.write(0);
  myservo6.write(0);
  myservo7.write(0);
  myservo8.write(0);
}

// boolean function to see if the triggered sensor is in a pair.

bool nonPaired(int sensorNum) {
  for (int i = 0; i < 7; i++) {
    if (nonPairedSensors[i] == sensorNum) {
      return true;
    }
  }
  return false;
}

void changeServoPos(){
  myservo.write(servoPos[0]);
  myservo2.write(servoPos[1]);
  myservo3.write(servoPos[2]);
  myservo4.write(servoPos[3]);
  myservo5.write(servoPos[4]);
  myservo6.write(servoPos[5]);
  myservo7.write(servoPos[6]);
  myservo8.write(servoPos[7]);
}

void servoNumCheck(int i){
   int servoNum = 0;
   if (i == 1 || i == 2){
    servoNum = 1;
   }
   else if(i == 3){
    // check the direction of the current train 
   }

   return servoNum
}
