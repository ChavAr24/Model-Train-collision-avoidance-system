/*
   Project Name: Train Collision Avoidance System (TCAS)
   Author(s): Aryan Chavan, Benjamin Ponka
   Date Started: 2025-03-24            Submission Date: 2025-04-__
   Version: V.2.06
   Description: Second year Mechatronics Engineering Diploma Project in which a train collision avoidance system is made with basic sensors(IR, sonar), actuators(Servos) and microcontroller(Arduino Mega).
   Licence:
   Links: https://github.com/ChavAr24/Model-Train-collision-avoidance-system.git
   Development Board: Arduino Mega 2560
*/

//Dependancies
#include <Servo.h>

#define ledpin 13     //for debugging
#define softReset 11  // for resetting the code without having to power it off and being able to test faster. // this pin mimics a push button. when the pin is grounded the button is pressed.

// Setting up the Ir Pins
int IRPins[15] = { 22, 23, 24, 25, 26, 27, 28, 29, A12, A13, A14, A15, 39, 40, 41 };  //not using 38 as it is an timer pin.

// Variables related to the IR sensor.
int sensor_state[15][3];  // each row corresponds to each sensor. cols: current state, last state, on track number.

// Variables for the servos
int pos[] = { 0, 30 };                        // all of the positions we would want the servo to be in.
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // current/live servo positions.

// Creating servo objects
Servo myservo;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
Servo myservo7;
Servo myservo8;

void setup() {
  pinMode(ledpin, OUTPUT);
  pinMode(softReset, INPUT_PULLUP);  // initializes pin 11 to be a soft reset pin.
  IRInit();                          // initialize the sensor pins as inputs and setting them to HIGH.
  attachServos();                    // attaching the pins of the servos to the object servo in code
  zeroServo();                       // zero all of the servo
  zeroIRStates();
  initTrackStatesIO();  // sets the vaules in this array to be 0 in all elements
  Serial.begin(9600);   // Serial monitor init
  delay(1000);
}

// New Logic
unsigned int Etimer = 0;  //Global timer (running eternally).
int sensorTimes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int timerDelay = 7;   // time delay for neglecting second sensor.
int timerDelay2 = 7;  // timer for specific case of ir 3(case 2) and 8( case 7)
int timerDelay3 = 10;

int trackStatesIO[10][2] = {};  // state, time       // records if the train on track was incoming or outgoing and at what time did it perform each command. (for cases 2,7, last 4 aswell.) row 0 is for track0, row 1 is for track 1, row 2 is for track 6, row 3 is for track 7.

void loop() {  // Main Loop

  do {
    readSensors();
    for (int i = 0; i < 15; i++) {

      if (sensor_state[i][0] == LOW) {
        int a = 0;                  // timer check variable.
        int b = 0;                  // timer check variable.
        int servoNum = 0;           // servo number for each case. 0 is servo 1 and 7 is servo 8
        int servoNum2 = 0;          // servo number for a specific case. 0 is servo 1 and 7 is servo 8
        int IrNum1 = 0;             // stores the sensor number
        int IrNum2 = 0;             // stores the sensor number
        int trackNumber1 = 0;       // stores the track numbers for each case.
        int trackNumber2 = 0;       // stores the track numbers for each case.
        int trackNumber3 = 0;       // stores the track numbers for specific case.
        int trackNumber4 = 0;       // stores the track numbers for specific case.
        int trackToUse = 0;         // stores the track number that is about to be changes to not available as in currently being used by a train.
        int incomingFromTrack = 0;  // for case 2 and 7 // the code uses incomingFromTrack variable to do both incoming and outgoing as those track numbers will be the same for either sub case.
        int currentIrPrevious = 0;  // use to store the time value from previously being tripped so that the new timer can be appeneded immediately.

        switch (i) {
          case 0:  // logic for Ir Sensor 1
            Serial.println("Case0");
            currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
            a = Etimer - sensorTimes[1];         // sensor trigger check. if the a value is greater than the set delay then consider the sensor as being triggered but if it is less then dont consider it being hit.
            //b = Etimer - currentIrPrevious; // replaced by DTP
            servoNum = 0;
            incomingFromTrack = 8;
            if (DTP(currentIrPrevious) == 1) {
              if (a > timerDelay) {  // if the a value is greater than the set delay then consider the sensor as being triggered but if it is less then dont consider it being hit.
                Serial.println("a is greater than timer delay");
                sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
                internalLogic1(2, 4, servoNum, incomingFromTrack);
              } else {
                Serial.println("Neglected");
                break;
              }
            } else {
              Serial.println("Case 0 Bounced");
              break;
            }
            // displaytrackstaus();
            Serial.println("Case0 finish");
            break;

          case 1:  // Logic for Ir sensor 2
            Serial.println("Case1");
            currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
            a = Etimer - sensorTimes[0];
            IrNum1 = sensorTimes[2];
            IrNum2 = sensorTimes[3];
            trackNumber1 = 2;
            trackNumber2 = 4;
            trackToUse = 8;  // change the init function for the ir to have the new track numbers. //sensor_states[i][2]
            //if (b > timerDelay3) { //b was not intalized anyways
            if (DTP(currentIrPrevious) == 1) {
              if (a > timerDelay) {
                sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
                internalLogic2(IrNum1, IrNum2, trackNumber1, trackNumber2, trackToUse);
              } else {
                Serial.println("Neglected");
                break;
              }
            } else {
              Serial.println("Case 1 Bounced");
              break;
            }
            Serial.println("Case1 finish");
            break;

          case 2:  // Logic for IR sensor 3
            Serial.println("Case2");
            currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
            a = Etimer - sensorTimes[7];
            servoNum = 1;  // servo number for this case.
            IrNum1 = 2;    // current ir
            IrNum2 = 7;    // opposing ir
            trackNumber1 = 0;
            trackNumber2 = 1;
            trackNumber3 = 2;
            incomingFromTrack = 2;  // the track number for where the train came in from.
            if (DTP(currentIrPrevious) == 1) {
              sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
              internalLogic3(servoNum, IrNum1, IrNum2, trackNumber1, trackNumber2, incomingFromTrack, trackNumber3);

            } else {
              Serial.println("Case 2 Bounced");
              break;
            }
            Serial.println("Case2 finish");
            break;

          case 3:  // Logic for IR sensor 4
            Serial.println("Case3");
            currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
            //IrNum1 = 3; // approching IR unneeded
            trackNumber1 = 5;  // This is to check if something is in the inner loop, track 5
            trackNumber2 = 6;
            trackNumber3 = 7;
            trackNumber4 = 9;             // This is to check if something is approching the junction from track 9, or in the center loop
            incomingFromTrack = 2;        // This means that it was approching from track 2
            servoNum = 3;                 // the servo that switches a counter-clockwise train into the inner loop.
            a = Etimer - sensorTimes[4];  // This takes the time since IR 4 was hit
            if (DTP(currentIrPrevious) == 1) {
              internalLogic4(servoNum, trackNumber1, trackNumber2, trackNumber3, trackNumber4, incomingFromTrack, a);
              //This was disabled as it was causing issues
              sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
              //this is moved here so that it only appends if it didn't bounce.
            } else {
              Serial.println("Case 3 Bounced");
              break;
            }
            Serial.println("Case3 finish");
            break;

          case 4:  // Logic for IR sensor 5
            Serial.println("Case4");
            currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
            sensorTimes[i] = Etimer;             // appends the trip time of the current sensor
            //this does not need to move since we don't care if it bounces
            a = Etimer - sensorTimes[1];
            servoNum = 2;
            servoNum2 = 3;
            Serial.println("Case4 finish");
            break;  // This does not need a double bounce protection, as it does not run anything

          case 5:  // Logic for IR sensor 6
            Serial.println("Case5");
            currentIrPrevious = sensorTimes[i];
            sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
            a = Etimer - sensorTimes[1];
            Serial.println("Case5 finish");
            break;  // This sensor is largely useless, I am not quite sure why it was even wired in, as it is the only sensor to not be mirrored along the central axis (#15 doesn't count, it's along that axis)
          // this also does not have DTP, as it is not needed, since it does not execute any code by itself
          case 6:  // Logic for IR sensor 7
            Serial.println("Case6");
            currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
            trackNumber1 = 5;                    // This is to check if something is in the inner loop, track 5
            trackNumber2 = 6;
            trackNumber3 = 7;
            trackNumber4 = 8;             // This is to check if something is approching the junction from track 8, or in the center loop
            incomingFromTrack = 3;        // This means that it was approching from track 3
            servoNum = 2;                 // the servo that switches a counter-clockwise train into the inner loop.
            a = Etimer - sensorTimes[4];  // This takes the time since IR 4 was hit
            if (DTP(currentIrPrevious) == 1) {
              internalLogic4(servoNum, trackNumber1, trackNumber2, trackNumber3, trackNumber4, incomingFromTrack, a);
              //Disabled for the same reason as in Case 3
              sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
            } else {
              Serial.println("Case 6 Bounced");
              break;
            }
            Serial.println("Case6 finish");
            break;

          case 7:  // Logic for IR sensor 8
            Serial.println("Case7");
            currentIrPrevious = sensorTimes[7];  // stores the previous time of the current sensor.
            a = Etimer - sensorTimes[2];         // This takes the time since IR 3 was hit
            servoNum = 4;                        // servo number for this case.
            IrNum1 = 7;                          // current ir
            IrNum2 = 2;                          // opposing ir
            trackNumber1 = 0;
            trackNumber2 = 1;
            trackNumber3 = 3;
            incomingFromTrack = 3;  // the track number for where the train came in from.
            if (DTP(currentIrPrevious) == 1) {
              Serial.println("DTP Case 7");
              if (a > timerDelay) {
                Serial.println("timer delay Case 7");
                sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
                Serial.println("Timer appended");
                internalLogic3(servoNum, IrNum1, IrNum2, trackNumber1, trackNumber2, incomingFromTrack, trackNumber3);
                Serial.println("logic 3 exit.");
              }else if (a < timerDelay) {
                Serial.println("Neglected");
                break;
              }
            } else {
              Serial.println("Case 7 Bounced");
              break;
            }
            Serial.println("Case7 finish");
            break;

          case 8:  // Logic for IR sensor 9
            Serial.println("Case8");
            currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
            a = Etimer - sensorTimes[9];
            IrNum1 = sensorTimes[7];
            IrNum2 = sensorTimes[6];
            trackNumber1 = 2;
            trackNumber2 = 4;
            trackToUse = 9;  // change the init function for the ir to have the new track numbers. //sensor_states[i][2]
            //if (b > timerDelay3) { //b was not intalized anyways
            if (DTP(currentIrPrevious) == 1) {
              if (a > timerDelay) {
                sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
                internalLogic2(IrNum1, IrNum2, trackNumber1, trackNumber2, trackToUse);
              } else if (a < timerDelay) {
                Serial.println("Neglected");
                break;
              }
            } else {
              Serial.println("Case 8 Bounced");
              break;
            }
            // digitalWrite(ledpin, LOW);
            Serial.println("Case8 finish");
            break;

          case 9:  // Logic for IR sensor 10
            Serial.println("Case9");
            currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
            a = Etimer - sensorTimes[8];         //compares with Sensor 8,
            servoNum = 5;
            incomingFromTrack = 9;
            if (DTP(currentIrPrevious) == 1) {
              if (a > timerDelay) {
                sensorTimes[i] = Etimer;                            // appends the trip time of the current sensor
                internalLogic1(3, 4, servoNum, incomingFromTrack);  // this stays the same when mirrored, as they're both connected to the same 2 tracks
              }else if (a < timerDelay) {
                Serial.println("Neglected");
                break;
              }
            } else {
              Serial.println("Case 9 Bounced");
              break;
            }
            Serial.println("Case9 finish");
            break;

          case 10:  // Logic for IR sensor 11
            Serial.println("Case10");
            currentIrPrevious = sensorTimes[i];
            a = Etimer - sensorTimes[11];  // change the i to the opposite ir numb.
            servoNum = 6;                  // servo number for this case.
            trackNumber1 = 6;
            trackNumber2 = 7;
            incomingFromTrack = 9;  // the track number for where the train came in from.

            if (DTP(currentIrPrevious) == 1) {
              Serial.println("DTP Case 10.");
              if (a > timerDelay) {
                Serial.println("Timer delay Case 10.");
                sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
                Serial.println("timer append Case 10.");
                internalLogic5(servoNum, trackNumber1, trackNumber2, incomingFromTrack);
                Serial.println("logic finish Case 10.");
              }
            } else {
              Serial.println("Case 10 Bounced");
              break;
            }
            Serial.println("Case10 finish");
            break;

          case 11:  // Logic for IR sensor 12
            Serial.println("Case11");
            currentIrPrevious = sensorTimes[i];
            a = Etimer - sensorTimes[10];
            trackNumber1 = 6;
            trackNumber2 = 7;
            incomingFromTrack = 9;

            if (DTP(currentIrPrevious) == 1) {
              if (a > timerDelay) {
                sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
                internalLogic6(incomingFromTrack, trackNumber1, trackNumber2);
              }else if (a < timerDelay) {
                Serial.println("Neglected");
                break;
              }
            } else {
              Serial.println("Case 11 Bounced");
              break;
            }
            Serial.println("Case11 finish");
            break;

          case 12:  // Logic for IR sensor 13
            Serial.println("Case12");
            currentIrPrevious = sensorTimes[12];
            a = Etimer - sensorTimes[13];
            trackNumber1 = 6;
            trackNumber2 = 7;
            incomingFromTrack = 8;
            if (DTP(currentIrPrevious) == 1) {
              if (a > timerDelay) {
                sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
                internalLogic6(incomingFromTrack, trackNumber1, trackNumber2);
              }else if (a < timerDelay) {
                Serial.println("Neglected");
                break;
              }
            } else {
              Serial.println("Case 12 Bounced");
              break;
            }
            Serial.println("Case12 finish");
            break;

          case 13:  // Logic for IR sensor 14
            Serial.println("Case13");
            currentIrPrevious = sensorTimes[i];
            a = Etimer - sensorTimes[12];
            servoNum = 7;  // servo number for this case.
            trackNumber1 = 6;
            trackNumber2 = 7;
            incomingFromTrack = 8;  // the track number for where the train came in from.
            if (DTP(currentIrPrevious) == 1) {
              if (a > timerDelay) {
                sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
                internalLogic5(servoNum, trackNumber1, trackNumber2, incomingFromTrack);
              }else if (a < timerDelay) {
                Serial.println("Neglected");
                break;
              }
            } else {
              Serial.println("Case 13 Bounced");
              break;
            }
            Serial.println("Case13 finish");
            break;

          case 14:  // Logic for IR sensor 15
            Serial.println("Case14");
            currentIrPrevious = sensorTimes[i];  // stores the previous time of the current sensor.
            trackNumber1 = 4;                    //the gauntlet track that connects to the middle loop
            trackNumber2 = 5;                    //the inner loop
            servoNum = 2;
            servoNum2 = 3;  // these are the two servos that connect to the inner loop, they are basically interchangeable
            if (DTP(currentIrPrevious) == 1) {
              //actually connected the logic for this, oops (I wrote it last time, but had forgotten to actually implement it, my bad.)
              sensorTimes[i] = Etimer;  // appends the trip time of the current sensor
              internalLogic15(trackNumber1, trackNumber2, servoNum, servoNum2);
            } else {
              Serial.println("Case 14 Bounced");
              break;
            }
            Serial.println("Case14 finish");
            break;
        }
      }
    }
    changeServoPos();
    displaytrackstates();
    Serial.print("Golbal Timer: ");
    Serial.println(Etimer);
    delay(50);
    Etimer += 1;
  }while (digitalRead(softReset) != LOW);  // end while

  if(digitalRead(softReset) == LOW){
    softRST();
  }
}  // end main loop

// Functions

bool DTP(int currentIrPrevious) {  //Double Tap Prevention
  if ((Etimer - currentIrPrevious) < timerDelay3) {
    return false;  //if the switch was just hit, then don't perform the code
  } else {
    return true;
  }
}

void displaytrackstates() {
  for (int i = 0; i < 10; i++) {
    // Serial.print("")
    Serial.print(trackStatesIO[i][0]);
    Serial.print(" ");
    // Serial.println(trackStatesIO[i][0]);
    // Serial.println(trackStatesIO[i][1]);
  }  //this is for debugging, so we can see which tracks are/aren't active
}

void internalLogic15(int trackNumber1, int trackNumber2, int servoNum1, int servoNum2) {
  if (trackStatesIO[trackNumber1][0] == 0) {  //if the inner track is clear, open both servos, on a delay
    // trackStatesIO[trackNumber2] = 0;       //clears the inner loop
    Serial.print("Updating track state");
    Serial.print(trackNumber2);
    updateTrackStates(trackNumber2, 0);  // the new implementation of this
    // trackStatesIO[trackNumber1] = 1;       // occupies the loop it is headed too
    Serial.print("Updating track state");
    Serial.print(trackNumber1);
    updateTrackStates(trackNumber1, 1);

    // This is opening the inner loop
    if (servoPos[servoNum1] == pos[1] && servoPos[servoNum2] == pos[1]) {
      Serial.println("both servo switch open.");
      servoPos[servoNum1] = pos[0];
      // delay(200);
      //does a brief delay to avoid too many sensor switches THIS MIGHT CAUSE ISSUES
      // This only happens if BOTH need to switch
      servoPos[servoNum2] = pos[0];
    } else if (servoPos[servoNum1] == pos[1]) {
      Serial.println("one servo switch open.");
      servoPos[servoNum1] = pos[0];
    } else if (servoPos[servoNum2] == pos[1]) {
      Serial.println("two servo switch open.");
      servoPos[servoNum2] = pos[0];
    }
    //CLOSING THE LOOP INSTEAD, SAME PROCESS but switched numbers
  } else {
    if (servoPos[servoNum1] == pos[0] && servoPos[servoNum2] == pos[0]) {
      Serial.println("both servo switch close.");
      servoPos[servoNum1] = pos[1];
      // delay(200);
      //does a brief delay to avoid too many sensor switches THIS MIGHT CAUSE ISSUES
      // This only happens if BOTH need to switch
      servoPos[servoNum2] = pos[1];
    }
    if (servoPos[servoNum1] == pos[0]) {
      Serial.println("one servo switch close.");
      servoPos[servoNum1] = pos[1];
    }
    if (servoPos[servoNum2] == pos[0]) {
      Serial.println("two servo switch close.");
      servoPos[servoNum2] = pos[1];
    }
  }
}

void internalLogic6(int incomingFromTrack, int trackNumber1, int trackNumber2) {
  // train is going out.
  if (trackStatesIO[trackNumber1][0] == 1 && trackStatesIO[trackNumber2][0] == 1) {
    //both track are occupied.
    if (trackStatesIO[trackNumber1][1] >= trackStatesIO[trackNumber2][1]) {
      // track 1 was just changed so it is not the train that is going out.
      Serial.print("Updating track state");
      Serial.print(trackNumber1);
      updateTrackStates(trackNumber1, 0);
    } else if (trackStatesIO[trackNumber1][1] < trackStatesIO[trackNumber2][1]) {
      // track 2 was just changed so it is not the train that is going out.
      Serial.print("Updating track state");
      Serial.print(trackNumber2);
      updateTrackStates(trackNumber2, 0);
    }

  } else if (trackStatesIO[trackNumber1][0] == 1 && trackStatesIO[trackNumber2][0] == 0) {
    // track 1 is occupied other is available.
    Serial.print("Updating track state");
    Serial.print(trackNumber1);
    updateTrackStates(trackNumber1, 0);
  } else if (trackStatesIO[trackNumber1][0] == 0 && trackStatesIO[trackNumber2][0] == 1) {
    // track 2 is occupied other is available.
    Serial.print("Updating track state");
    Serial.print(trackNumber2);
    updateTrackStates(trackNumber2, 0);
  }
  Serial.print("Updating track state");
  Serial.print(incomingFromTrack);
  updateTrackStates(incomingFromTrack, 1);
}

// updates both array for track states
void updateTrackStates(int trackNum, int stateTo) {
  if (stateTo == 0) {
    // trackStates[trackNum] = 0;
    trackStatesIO[trackNum][0] = 0;
    trackStatesIO[trackNum][1] = Etimer;
  } else if (stateTo == 1) {
    // trackStates[trackNum] = 1;
    trackStatesIO[trackNum][0] = 1;
    trackStatesIO[trackNum][1] = Etimer;
  }
}

void internalLogic5(int servoNum, int trackNumber1, int trackNumber2, int incomingFromTrack) {
  if (trackStatesIO[trackNumber1][0] == 0 && trackStatesIO[trackNumber2][0] == 0) {
    // if both track are available. choose a random track and put it on that.
    Serial.println("Both tracks are available.");
    int randomNum = random(0, 1);
    Serial.println(randomNum);
    if (randomNum == 0) {
      if (servoPos[servoNum] == pos[0]) {
        servoPos[servoNum] = pos[1];
      }
      Serial.print("Updating track state");
      Serial.print(trackNumber1);
      updateTrackStates(trackNumber1, 1);
    } else if (randomNum == 2) {
      if (servoPos[servoNum] == pos[1]) {
        servoPos[servoNum] = pos[0];
      }
      Serial.print("Updating track state");
      Serial.print(trackNumber2);
      updateTrackStates(trackNumber2, 1);
    }
  } else if (trackStatesIO[trackNumber1][0] == 1 || trackStatesIO[trackNumber2][0] == 1) {
    // if either of the tracks are occupied.
    if (trackStatesIO[trackNumber1][0] == 1) {
      Serial.println("Track 1 is occupied.");
      // if track 1 is occupied then put train on the other one.
      if (servoPos[servoNum] == pos[1]) {
        servoPos[servoNum] = pos[0];
      }
      Serial.print("Updating track state");
      Serial.print(trackNumber2);
      updateTrackStates(trackNumber2, 1);

    } else if (trackStatesIO[trackNumber2][0] == 1) {
      // if track 2 is occupied then put train on track 1
      Serial.println("Track 2 is occupied.");
      if (servoPos[servoNum] == pos[0]) {
        servoPos[servoNum] = pos[1];
      }
      Serial.print("Updating track state");
      Serial.print(trackNumber1);
      updateTrackStates(trackNumber1, 1);
    }
  }
  updateTrackStates(incomingFromTrack, 0);
}

// I am fixing this right now
void internalLogic4(int servoNum, int trackNumber1, int trackNumber2, int trackNumber3, int trackNumber4, int incomingFromTrack, int a) {
  if (a > 4 * timerDelay) {  // TLDR, if sensor 4 was NOT hit recently, do this section, this is multiplied to make up for the gap

    if (trackStatesIO[trackNumber1][0] == 1) {  // this is checking if the inner loop is occupied, so we can know if it's safe to put a train there or not.
      if (servoPos[servoNum] == pos[0]) {
        servoPos[servoNum] = pos[1];
        // Change Track States.
      }
    } else if (trackStatesIO[trackNumber2][0] == 1 || trackStatesIO[trackNumber3][0] == 1 || trackStatesIO[trackNumber4][0] == 1) {  // if there is an incoming(?) train

      if (servoPos[servoNum] == pos[1]) {
        servoPos[servoNum] = pos[0];         // this is to shift the train into the central loop
        updateTrackStates(trackNumber1, 1);  //since the train is in the central loop, then set that loop as occupied
        updateTrackStates(incomingFromTrack, 0);
        ;
      } else {
        if (servoPos[servoNum] == pos[0]) {
          servoPos[servoNum] = pos[1];
        }
      }
    }

  } else {
    Serial.print("Updating track state");
    Serial.print("4");
    updateTrackStates(4, 0);  //Since sensor 4 was just hit, this is leaving, clear the track
  }
}

void internalLogic3(int servoNum, int IrNum1, int IrNum2, int trackNumber1, int trackNumber2, int incomingFromTrack, int trackNumber3) {
  Serial.println("in logic 3.");
  if (trackStatesIO[trackNumber1][0] == 0 && trackStatesIO[trackNumber2][0] == 0) {
    Serial.println("if Statement");
    int randomNum = random(trackNumber1, trackNumber2);
    if (randomNum == trackNumber1) {
      Serial.println("Chose track 1.");
      if (servoPos[servoNum] == pos[1]) {
        servoPos[servoNum] = pos[0];
      }
      Serial.print("Updating track state");
      Serial.print(trackNumber1);
      updateTrackStates(trackNumber1, 1);
    } else if (randomNum == trackNumber2) {
      Serial.println("Chose track 2.");
      if (servoPos[servoNum] == pos[0]) {
        servoPos[servoNum] = pos[1];
      }
      Serial.print("Updating track state");
      Serial.print(trackNumber2);
      updateTrackStates(trackNumber2, 1);
    }
    Serial.print("Updating track state");
    Serial.print(incomingFromTrack);
    updateTrackStates(incomingFromTrack, 0);  // fixed
  } else if (trackStatesIO[trackNumber1][0] == 1 || trackStatesIO[trackNumber2][0] == 1) {
    if (Etimer - sensorTimes[IrNum2] <= timerDelay2) {
      // train coming in
      Serial.println("Train is going in.");
      if (trackStatesIO[trackNumber1][0] == 1) {
        // track 0 is being used
        if (servoPos[servoNum] == pos[0]) {
          servoPos[servoNum] = pos[1];
        }
        Serial.print("Updating track state");
        Serial.print(trackNumber2);
        updateTrackStates(trackNumber2, 1);
      } else if (trackStatesIO[trackNumber2][0] == 1) {
        // track 1 is being used.
        if (servoPos[servoNum] == pos[1]) {
          servoPos[servoNum] = pos[0];
        }
        Serial.print("Updating track state");
        Serial.print(trackNumber1);
        updateTrackStates(trackNumber1, 1);
      }
      Serial.print("Updating track state");
      Serial.print(incomingFromTrack);
      updateTrackStates(incomingFromTrack, 0);

    } else if (Etimer - sensorTimes[IrNum2] > timerDelay2) {
      // train going out.
      Serial.println("Train is going out.");
      if (trackStatesIO[trackNumber1][0] == 1 && trackStatesIO[trackNumber2][0] == 1) {
        // both tracks are occupied.
        if (trackStatesIO[trackNumber1][1] >= trackStatesIO[trackNumber2][1]) {
          // track 0 time is greater which means this train is not the one going out.
          Serial.print("Updating track state");
          Serial.print(trackNumber2);
          updateTrackStates(trackNumber2, 0);
        } else if (trackStatesIO[trackNumber1][1] < trackStatesIO[trackNumber2][1]) {
          // track 1 time is greater which means this train is not the one going out.
          Serial.print("Updating track state");
          Serial.print(trackNumber1);
          updateTrackStates(trackNumber1, 0);
        }
      } else if (trackStatesIO[trackNumber1][0] == 1) {
        // track 0 is being used
        Serial.print("Updating track state");
        Serial.print(trackNumber1);
        updateTrackStates(trackNumber1, 0);
      } else if (trackStatesIO[trackNumber2][0] == 1) {
        // track 1 is being used.
        Serial.print("Updating track state");
        Serial.print(trackNumber2);
        updateTrackStates(trackNumber2, 0);
      }
      Serial.print("Updating track state");
      Serial.print(incomingFromTrack);
      updateTrackStates(incomingFromTrack, 1);  // the code uses incomingFromTrack variable to do both incoming and outgoing as those track numbers will be the same for either sub case.
    }
  }
  Serial.println("internal logic 3");
}

void internalLogic2(int IrNumber1, int IrNumber2, int tracknumber1, int tracknumber2, int trackInUse) { // re write taking in consideration there are 2 tracks the trains can come from.
  Serial.println("In logic 2");
  if (IrNumber1 >= IrNumber2) {
    updateTrackStates(tracknumber1, 0);
    Serial.println("if statement finished");
  } else if (IrNumber1 < IrNumber2) {
    updateTrackStates(tracknumber2, 0);
    Serial.println("else if statement finished");
  }
  // if ir on track 1 or 2 was triggered last by comparing the timers for them.
  // if track 1 timer is greater then make track 2 be available and vice-versa.
  updateTrackStates(trackInUse, 1);
  Serial.println("internal logic 2 finish");
}

void internalLogic1(int track1, int track2, int servoNumber, int incomingFrom) {  // (track number 1, track number 2, servo number)
  Serial.println("In logic 1");
  if (trackStatesIO[track1][0] == 0 && trackStatesIO[track2][0] == 0) {
    Serial.println("none of the tracks are occupied.");
    int randNum = random(0, 1);
    if (randNum == 0) {
      // put on track 1.
      Serial.println("Chose track 1.");
      if (servoPos[servoNumber] == pos[0]) {  // if servo position is for track 1 or track 2
        servoPos[servoNumber] = pos[1];
      }
      updateTrackStates(track1, 1);
    } else if (randNum == 1) {
      // put on track 2.
      Serial.println("Chose track 2.");
      if (servoPos[servoNumber] == pos[1]) {  // if servo position is for track 1 or track 2
        servoPos[servoNumber] = pos[0];
      }
      updateTrackStates(track2, 1);
    }
  } else if (trackStatesIO[track1][0] == 0 || trackStatesIO[track2][0] == 0) {
    Serial.println("either of the tracks are occupied.");
    if (trackStatesIO[track1][0] == 0) {  // if track one is available
      Serial.println("track 1 is open.");
      if (servoPos[servoNumber] == pos[0]) {  // if servo position is for track 1 or track 2
        Serial.println("servoPos is 0");
        servoPos[servoNumber] = pos[1];
      }
      updateTrackStates(track1, 1);
    } else if (trackStatesIO[track2][0] == 0) {  // if track one is available
      Serial.println("track 2 is open.");
      if (servoPos[servoNumber] == pos[1]) {  // if servo position is for track 1 or track 2
        Serial.println("servoPos is 1");
        servoPos[servoNumber] = pos[0];
      }
      updateTrackStates(track2, 1);
    }
  }

  updateTrackStates(incomingFrom, 0);
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
  myservo2.write(0);
  delay(3000);
  myservo3.write(0);
  myservo4.write(0);
  delay(3000);
  myservo5.write(0);
  myservo6.write(0);
  delay(3000);
  myservo7.write(0);
  myservo8.write(0);

  for (int i = 0; i < sizeof(servoPos); i++) {
    servoPos[i] = 0;
  }
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
  for (int i = 0; i < 10; i++) {
    trackStatesIO[i][0] = 0;
    trackStatesIO[i][1] = 0;
  }
}

void softRST() {

  // If Running serial monitor in arduino IDE
  // print 10 empty serial lines.
  for (int i = 0; i < 10; i++) {
    Serial.println("");
  }

  // If running serial monitor in python serial monitor.
  // print

  // common for both
  zeroServo();
  zeroIRStates();
  initTrackStatesIO();
  Etimer = 0;
}
