/*
   Project Name: Train Collision Avoidance System (TCAS)
   Author(s): Aryan Chavan, Benjamin Ponka, Krish Patel, Namra Patel
   Date Started: 2025-03-24            Submission Date: 2025-04-17
   Version: V.1.03
   Description: Second year Mechatronics Engineering Diploma Project in which a train collision avoidance system is made with basic sensors(IR, sonar), actuators(Servos) and microcontroller(Arduino Mega).
   Licence:
   Links:
   Development Board: Arduino Mega 2560
*/

//Dependancies
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
int servoPos[] = {0, 0, 0, 0, 0, 0, 0, 0};   // current/live servo positions.

// Creating servo objects
Servo myservo;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
Servo myservo7;
Servo myservo8;


//
int pairedSensors[] = {1, 2, 9, 10, 11, 12, 13, 14};
int pairedOuter[] = {1, 10, 11, 14};
int pairedInner[] = {2, 9, 12, 13};


void setup() {
  // initialize the LED pin as an output:
  pinMode(LEDPIN, OUTPUT);

  IRInit();                             // initialize the sensor pins as inputs and setting them to HIGH.
  attachServos();                       // attaching the pins of the servos to the object servo in code
  zeroServo();                          // zero all of the servo
  zeroIRStates();

  Serial.begin(9600);                 // Serial monitor
}


// New Logic
int Etimer = 1; //Global timer.
int sensorTimes[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int timerDelay = 1000; // time delay for neglecting second sensor.
int timerDelay2 = 2500; // timer for specific case of ir 3(case 2) and 8( case 7)

int trackStates[] = {0, 0, 1, 0, 0, 0, 1, 1};

void loop() {


  readSensors();
  for (int i = 0; i < 15; i++) {
    if (sensor_state[i][0] == LOW) {
      int a = 0; // timer check variable.
      int servoNum = 0;
      int IrNum1 = 0;
      int IrNum2 = 0;
      int trackNumber1 = 0;
      int trackNumber2 = 0;
      int trackToUse = 0;

      switch (i) {
        case 0:       // logic for Ir Sensor 1
          Serial.println("Case1");
          sensorTimes[i] = Etimer;
          a = Etimer - sensorTimes[1];
          servoNum = 0;
          if (a > timerDelay) {
            internalLogic1(1, 2, servoNum);
          }
          Serial.println("Case1 finish");
          break;

        case 1:       // Logic for Ir sensor 2
          Serial.println("Case2");
          sensorTimes[i] = Etimer;
          a = Etimer - sensorTimes[0];
          IrNum1 = 2;
          IrNum2 = 3;
          trackNumber1 = 1;
          trackNumber2 = 2;
          trackToUse = sensor_states[i][2];
          if (a > timerDelay) {
            internalLogic2(IrNum1, IrNum2, trackNumber1, trackNumber2, trackToUse);
          }
          Serial.println("Case2 finish");
          break;

          }
      }
    }
    changeServoPos();
    Etimer += 1;
    Serial.println(Etimer);





  }// end main loop


  // Functions

  void internalLogic2(int IrNumber1, int IrNumber2, int tracknumber1, int tracknumber2, int trackInUse) {

    if (sensorTimes[IrNumber1] > sensorTimes[IrNumber2]) {
      trackStates[tracknumber2] = 0;
    }
    else if (sensorTimes[IrNumber1] < sensorTimes[IrNumber2]) {
      trackStates[tracknumber1] = 0;
    }
    // if ir on track 1 or 2 was triggered last by comparing the timers for them.
    // if track 1 timer is greater then make track 2 be available and vice-versa.
    trackStates[trackInUse] = 1;
    Serial.println("internal logic 2");

  }

  void internalLogic1(int track1, int track2, int servoNumber) {  // (track number 1, track number 2, servo number)

    if (servoNumber == 1 || servoNumber == 4 || servoNumber == 6 || servoNumber == 7) {
      if (trackStates[track1] != 1) { // if track one is available
        if (servoPos[servoNumber] == 0) { // if servo position is for track 1 or track 2
          servoPos[servoNumber] = pos[1];
          trackStates[track1] = 1;
        }
      }
      else if (trackStates[track2] != 1) { // if track one is available
        if (servoPos[servoNumber] == 0) { // if servo position is for track 1 or track 2
          servoPos[servoNumber] = pos[0];
          trackStates[track2] = 1;
        }
      }
    }

    else {
      if (track1 != 1) { // if track one is available
        if (servoPos[servoNumber] == 0) { // if servo position is for track 1 or track 2
          servoPos[servoNumber] = pos[0];
          trackStates[track1] = 1;
        }
      }
      else if (2 != 1) { // if track one is available
        if (servoPos[servoNumber] == 0) { // if servo position is for track 1 or track 2
          servoPos[servoNumber] = pos[1];
          trackStates[track2] = 1;
        }
      }
    }
    Serial.println("internal logic 1");
  }


  void readSensors() {
    for (int i = 0; i < 15; i++) {
      sensor_state[i][0] = digitalRead(IRPins[i]);
    }
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
      sensor_state[i][0] = 0;
      sensor_state[i][1] = 0;
      // sets the track numbers of each of the sensors.
      if (i == 1 || i == 2 || i == 3 || i == 8 || i == 9 || i == 10) {
        sensor_state[i][2] = 1;
      }
      else if (i == 4 || i == 5 || i == 6 || i == 7) {
        sensor_state[i][2] = 2;
      }
      else if (i == 15) {
        sensor_state[i][2] = 3;
      }
      else if (i == 11 || i == 12 || i == 13 || i == 14) {
        sensor_state[i][2] = 4;
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
    delay(.5);
    myservo3.write(0);
    myservo4.write(0);
    delay(.5);
    myservo5.write(0);
    myservo6.write(0);
    delay(.5);
    myservo7.write(0);
    myservo8.write(0);
  }

  bool isPaired(int a) {
    for (int i = 0; i < 8; i++) {
      if (a == pairedSensors[i]) {
        return true;
      }
    }
    return false;
  }

  void changeServoPos() {
    myservo.write(servoPos[0]);
    myservo2.write(servoPos[1]);
    myservo3.write(servoPos[2]);
    myservo4.write(servoPos[3]);
    myservo5.write(servoPos[4]);
    myservo6.write(servoPos[5]);
    myservo7.write(servoPos[6]);
    myservo8.write(servoPos[7]);
  }
