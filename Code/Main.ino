/*
   Project Name: Train Collision Avoidance System (TCAS)
   Author(s): Aryan Chavan, Benjamin Ponka
   Date Started: 2025-03-24            Submission Date: 2025-04-17
   Version: V.1.08.a
   Description: Second year Mechatronics Engineering Diploma Project in which a train collision avoidance system is made with basic sensors(IR, sonar), actuators(Servos) and microcontroller(Arduino Mega).
   Licence:
   Links:
   Development Board: Arduino Mega 2560
*/

//Dependancies
#include <Servo.h>

#define LEDPIN 13  // indicator for the ir beams( if beam is broken the light turns on)


// Setting up the Ir Pins
int IRPins[15] = { 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36 };


// Variables related to the IR sensor.
// Try to put this in an array.
int sensor_state[15][3];  // each row corresponds to each sensor. cols: current state, last state, on track number.
int nonPairedSensors[] = { 3, 4, 5, 6, 7, 8, 15 };

// Variables for the servos
int pos[] = { 0, 30, 60, 90, 120, 150, 180 };  // all of the positions we would want the servo to be in.
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 };   // current/live servo positions.

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
int pairedSensors[] = { 1, 2, 9, 10, 11, 12, 13, 14 };
int pairedOuter[] = { 1, 10, 11, 14 };
int pairedInner[] = { 2, 9, 12, 13 };


void setup() {
  // initialize the LED pin as an output:
  pinMode(LEDPIN, OUTPUT);

  IRInit();        // initialize the sensor pins as inputs and setting them to HIGH.
  attachServos();  // attaching the pins of the servos to the object servo in code
  zeroServo();     // zero all of the servo
  zeroIRStates();
  initTrackStatesIO();

  Serial.begin(9600);  // Serial monitor
}


// New Logic
unsigned int Etimer = 1;  //Global timer (running eternally).
int sensorTimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int timerDelay = 1000;   // time delay for neglecting second sensor.
int timerDelay2 = 2500;  // timer for specific case of ir 3(case 2) and 8( case 7)

int trackStates[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  //  state of track numbers ( new markings)
int trackStatesIO[9][2] = {};                       // records if the train on track was incoming or outgoing and at what time did it perform each command. (for cases 2,7, last 4 aswell.) row 0 is for track0, row 1 is for track 1, row 2 is for track 6, row 3 is for track 7.

int count = 0;

void loop() {  // Main Loop

  readSensors();
  for (int i = 0; i < 15; i++) {

    if (sensor_state[i][0] == LOW) {
      if (count == 0 || count == 1) {
        // change the state of the track number of the ir sensor that was just triggered.
        if (trackStates[sensor_state[i][2]] == 0) {
          trackStates[sensor_state[i][2]] = 1;
          count += 1;
        }
      }
    }

    int a = 0;  // timer check variable.
    int servoNum = 0;
    int servoNum2 = 0;
    int IrNum1 = 0;
    int IrNum2 = 0;
    int trackNumber1 = 0;
    int trackNumber2 = 0;
    int trackNumber3 = 0;
    int trackNumber4 = 0;
    int trackToUse = 0;
    int incomingFromTrack = 0;  // for case 2 and 7 // the code uses incomingFromTrack variable to do both incoming and outgoing as those track numbers will be the same for either sub case.
    int currentIrPrevious = 0;  // use to store the time value from previously being tripped so that the new timer can be appeneded immediately.

    switch (i) {
      case 0:  // logic for Ir Sensor 1
        Serial.println("Case0");
        currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
        sensorTimes[i] = Etimer;             // appends the trip time of the current sensor
        a = Etimer - sensorTimes[1];
        servoNum = 0;
        if (a > timerDelay) {
          internalLogic1(4, 2, servoNum);
        }
        Serial.println("Case0 finish");
        break;

      case 1:  // Logic for Ir sensor 2
        Serial.println("Case1");
        currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
        sensorTimes[i] = Etimer;             // appends the trip time of the current sensor
        a = Etimer - sensorTimes[0];
        IrNum1 = 2;
        IrNum2 = 3;
        trackNumber1 = 2;
        trackNumber2 = 4;
        trackToUse = 8;  // change the init function for the ir to have the new track numbers. //sensor_states[i][2]
        if (a > timerDelay) {
          internalLogic2(IrNum1, IrNum2, trackNumber1, trackNumber2, trackToUse);
        }
        Serial.println("Case1 finish");
        break;

      case 2:  // Logic for IR sensor 3
        Serial.println("Case2");
        currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
        sensorTimes[i] = Etimer;             // appends the trip time of the current sensor

        servoNum = 2;  // servo number for this case.
        IrNum1 = 3;    // current ir
        IrNum2 = 7;    // opposing ir
        trackNumber1 = 0;
        trackNumber2 = 1;
        incomingFromTrack = sensor_state[i][2];  // the track number for where the train came in from.

        internalLogic3(servoNum, IrNum1, IrNum2, trackNumber1, trackNumber2, incomingFromTrack, currentIrPrevious);

        Serial.println("Case2 finish");
        break;

      case 3:  // Logic for IR sensor 4
               // For deciding if it should enter the internal loop, when approching from track 2
        Serial.println("Case3");
        currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
        sensorTimes[i] = Etimer;             // appends the trip time of the current sensor
        a = Etimer - sensorTimes[0];

        //IrNum1 = 3; // approching IR unneeded

        trackNumber1 = 5;  // This is to check if something is in the inner loop, track 5
        trackNumber2 = 6;
        trackNumber3 = 7;
        trackNumber4 = 9;       // This is to check if something is approching the junction from track 9, or in the center loop
        incomingFromTrack = 2;  // This means that it was approching from track 2
        servoNum = 4;           // the servo that switches a counter-clockwise train into the inner loop.


        internalLogic4(servoNum, trackNumber1, trackNumber2, trackNumber3, trackNumber4, incomingFromTrack);

        Serial.println("Case3 finish");
        break;

      case 4:  // Logic for IR sensor 5
        Serial.println("Case4");
        currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
        sensorTimes[i] = Etimer;             // appends the trip time of the current sensor
        a = Etimer - sensorTimes[1];
        servoNum = 3;
        servoNum2 = 4;
        Serial.println("Case4 finish");
        break;

        // case 5:   // Logic for IR sensor 6
        //   Serial.println("Case2");
        //   a = Etimer - sensorTimes[0];
        //   IrNum1 = 2;
        //   IrNum2 = 3;
        //   trackNumber1 = 1;
        //   trackNumber2 = 2;
        //   if (a > timerDelay) {
        //     internalLogic2(IrNum1, IrNum2, trackNumber1, trackNumber2);
        //   }
        //   Serial.println("Case2 finish");
        //   break;

        // case 6:   // Logic for IR sensor 7
        //   Serial.println("Case1");
        //   a = Etimer - sensorTimes[1];
        //   servoNum = 0;
        //   if (a > timerDelay) {
        //     internalLogic1(1, 2, servoNum);
        //   }
        //   Serial.println("Case1 finish");
        //   break;

      case 7:  // Logic for IR sensor 8
        Serial.println("Case7");
        currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
        sensorTimes[i] = Etimer;             // appends the trip time of the current sensor
        servoNum = 2;                        // servo number for this case.
        IrNum1 = 7;                          // current ir
        IrNum2 = 3;                          // opposing ir
        trackNumber1 = 0;
        trackNumber2 = 1;
        incomingFromTrack = sensor_state[i][2];  // the track number for where the train came in from.

        if (a > timerDelay) {
          internalLogic3(servoNum, IrNum1, IrNum2, trackNumber1, trackNumber2, incomingFromTrack, currentIrPrevious);
        }
        Serial.println("Case7 finish");
        break;

      case 8:  // Logic for IR sensor 9
        Serial.println("Case8");
        a = Etimer - sensorTimes[1];
        servoNum = 0;
        if (a > timerDelay) {
          internalLogic1(1, 2, servoNum);
        }
        Serial.println("Case8 finish");
        break;

        // case 9:   // Logic for IR sensor 10
        //   Serial.println("Case2");
        //   a = Etimer - sensorTimes[0];
        //   IrNum1 = 2;
        //   IrNum2 = 3;
        //   trackNumber1 = 1;
        //   trackNumber2 = 2;
        //   if (a > timerDelay) {
        //     internalLogic2(IrNum1, IrNum2, trackNumber1, trackNumber2);
        //   }
        //   Serial.println("Case2 finish");
        //   break;

        // case 10:   // Logic for IR sensor 11
        //   Serial.println("Case1");
        //   a = Etimer - sensorTimes[1];
        //   servoNum = 0;
        //   if (a > timerDelay) {
        //     internalLogic1(1, 2, servoNum);
        //   }
        //   Serial.println("Case1 finish");
        //   break;

        // case 11:   // Logic for IR sensor 12
        //   Serial.println("Case2");
        //   a = Etimer - sensorTimes[0];
        //   IrNum1 = 2;
        //   IrNum2 = 3;
        //   trackNumber1 = 1;
        //   trackNumber2 = 2;
        //   if (a > timerDelay) {
        //     internalLogic2(IrNum1, IrNum2, trackNumber1, trackNumber2);
        //   }
        //   Serial.println("Case2 finish");
        //   break;

        // case 12:   // Logic for IR sensor 13
        //   Serial.println("Case1");
        //   a = Etimer - sensorTimes[1];
        //   servoNum = 0;
        //   if (a > timerDelay) {
        //     internalLogic1(1, 2, servoNum);
        //   }
        //   Serial.println("Case1 finish");
        //   break;

        // case 13:   // Logic for IR sensor 14
        //   Serial.println("Case2");
        //   a = Etimer - sensorTimes[0];
        //   IrNum1 = 2;
        //   IrNum2 = 3;
        //   trackNumber1 = 1;
        //   trackNumber2 = 2;
        //   if (a > timerDelay) {
        //     internalLogic2(IrNum1, IrNum2, trackNumber1, trackNumber2);
        //   }
        //   Serial.println("Case2 finish");
        //   break;

      case 14:  // Logic for IR sensor 15
        Serial.println("Case14");
        currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
        sensorTimes[i] = Etimer;             // appends the trip time of the current sensor
        Serial.println("Case14 finish");
        break;
    }
  }

  changeServoPos();
  Etimer += 1;
  Serial.println(Etimer);
}  // end main loop


// Functions

void internalLogic4(int servoNum, int trackNumber1, int trackNumber2, int trackNumber3, int trackNumber4, int incomingFromTrack) {
  if (trackStates[trackNumber1] == 1) {  // this is checking if the inner loop is occupied, so we can know if it's safe to put a train there or not.
    if (servoPos[servoNum] == pos[0]) {
      servoPos[servoNum] = pos[1];
    } else if (trackStates[trackNumber2 == 1] || trackStates[trackNumber3 == 1] || trackStates[trackNumber4 == 1]) {  // if there is an incoming(?) train
      if (servoPos[servoNum] == pos[1]) {
        servoPos[servoNum] = pos[0];    // this is to shift the train into the central loop
        trackStates[trackNumber1] = 1;  //since the train is in the central loop, then set that loop as occupied
        trackStates[incomingFromTrack] = 0;
      } else {

        if (servoPos[servoNum] == pos[0]) {
          servoPos[servoNum] = pos[1];
        }
      }
    }
  }
}

void internalLogic3(int servoNum, int IrNum1, int IrNum2, int trackNumber1, int trackNumber2, int incomingFromTrack, int currentIrPrevious) {
  if (trackStates[trackNumber1] == 1 || trackStates[trackNumber2] == 1) {
    if (Etimer - sensorTimes[IrNum2] < timerDelay2) {
      // train coming in
      if (trackStates[trackNumber1] == 1) {
        // track 0 is being used
        if (servoPos[servoNum] == pos[0]) {
          servoPos[servoNum] = pos[1];
          trackStates[trackNumber2] = 1;
          trackStates[incomingFromTrack] = 0;
        }
      } else if (trackStates[trackNumber2] == 1) {
        // track 1 is being used.
        if (servoPos[servoNum] == pos[1]) {
          servoPos[servoNum] = pos[0];
          trackStates[trackNumber1] = 1;
          trackStates[incomingFromTrack] = 0;
        }
      }
    } else if (Etimer - sensorTimes[IrNum2] > timerDelay2) {
      // train going out.
      if (trackStates[trackNumber1] == 1 && trackStates[trackNumber2] == 1) {
        // both tracks are occupied.
        if (trackStatesIO[trackNumber1][1] > trackStatesIO[trackNumber2][1]) {
          // track 0 time is greater which means this train is not the one going out.
          trackStates[trackNumber2] = 0;
          trackStatesIO[trackNumber2][0] = 0;
          trackStatesIO[trackNumber2][1] = Etimer;
        } else if (trackStatesIO[trackNumber1][1] < trackStatesIO[trackNumber2][1]) {
          // track 1 time is greater which means this train is not the one going out.
          trackStates[trackNumber1] = 0;
          trackStatesIO[trackNumber1][0] = 0;
          trackStatesIO[trackNumber1][1] = Etimer;
        }
      } else if (trackStates[trackNumber1] == 1) {
        // track 0 is being used
        trackStates[trackNumber1] = 0;
      } else if (trackStates[trackNumber2] == 1) {
        // track 1 is being used.
        trackStates[trackNumber2] = 0;
      }
      trackStates[incomingFromTrack] = 1;  // the code uses incomingFromTrack variable to do both incoming and outgoing as those track numbers will be the same for either sub case.
    }

  } else {
    int randomNum = random(trackNumber1, trackNumber2);
    if (randomNum == trackNumber1) {
      if (servoPos[servoNum] == pos[1]) {
        servoPos[servoNum] = pos[0];
        trackStates[trackNumber1] = 1;
      }
    } else if (randomNum == trackNumber2) {
      if (servoPos[servoNum] == pos[0]) {
        servoPos[servoNum] = pos[1];
        trackStates[trackNumber2] = 1;
      }
    }
  }
  Serial.println("internal logic 3");
}


void internalLogic2(int IrNumber1, int IrNumber2, int tracknumber1, int tracknumber2, int trackInUse) {

  if (sensorTimes[IrNumber1] > sensorTimes[IrNumber2]) {
    trackStates[tracknumber2] = 0;
  } else if (sensorTimes[IrNumber1] < sensorTimes[IrNumber2]) {
    trackStates[tracknumber1] = 0;
  }
  // if ir on track 1 or 2 was triggered last by comparing the timers for them.
  // if track 1 timer is greater then make track 2 be available and vice-versa.
  trackStates[trackInUse] = 1;
  Serial.println("internal logic 2");
}

void internalLogic1(int track1, int track2, int servoNumber) {  // (track number 1, track number 2, servo number)

  if (servoNumber == 1) {                //|| servoNumber == 4 || servoNumber == 6 || servoNumber == 7
    if (trackStates[track1] != 1) {      // if track one is available
      if (servoPos[servoNumber] == 0) {  // if servo position is for track 1 or track 2
        servoPos[servoNumber] = pos[1];
        trackStates[track1] = 1;
      }
    } else if (trackStates[track2] != 1) {  // if track one is available
      if (servoPos[servoNumber] == 0) {     // if servo position is for track 1 or track 2
        servoPos[servoNumber] = pos[0];
        trackStates[track2] = 1;
      }
    }
  }

  else {
    if (track1 != 1) {                   // if track one is available
      if (servoPos[servoNumber] == 0) {  // if servo position is for track 1 or track 2
        servoPos[servoNumber] = pos[0];
        trackStates[track1] = 1;
      }
    } else if (2 != 1) {                 // if track one is available
      if (servoPos[servoNumber] == 0) {  // if servo position is for track 1 or track 2
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
    if (i == 0 || i == 1) {
      sensor_state[i][2] = 8;
    } else if (i == 2) {
      sensor_state[i][2] = 2;
    } else if (i == 3 || i == 4 || i == 5 || i == 6) {
      sensor_state[i][2] = 4;
    } else if (i == 7) {
      sensor_state[i][2] = 3;
    } else if (i == 8 || i == 9) {
      sensor_state[i][2] = 9;
    } else if (i == 11 || i == 12 || i == 13 || i == 10) {
      sensor_state[i][2] = 6;
    } else if (i == 14) {
      sensor_state[i][2] = 5;
    }
  }

  for (int i = 0; i < 15; i++) {  // prints the array.
    for (int j = 0; j < 3; j++) {
      Serial.print(sensor_state[i][j]);
      Serial.print(" ");  // Delimiter between elements
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
  myservo2.write(00);
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

void initTrackStatesIO() {  // initialising the trackstateIO array.
  for (int i = 0; i < 9; i++) {
    trackStatesIO[i][0] = 0;
    trackStatesIO[i][1] = 0;
  }
}
