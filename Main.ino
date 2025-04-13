/*
 * Project Name: Train Collision Avoidance System (TCAS)
 * Author(s): Aryan Chavan, Benjamin Ponka, Krish Patel, Namra Patel
 * Date Started: 2025-03-24            Submission Date: 2025-04-17
 * Version: V.1.00
 * Description: Second year Mechatronics Engineering Diploma Project in which a train collision avoidance system is made with basic sensors(IR, sonar), actuators(Servos) and microcontroller(Arduino Mega). 
 * Licence:
 * Links:
 * Development Board: Arduino Mega 2560
 */

 // Dependancies
#include <Servo.h>


#define LEDPIN 13  // indicator for the ir beams( if beam is broken the light turns on)


// Setting up the Ir Pins
int IRPins[15] = {22,23,24,25,26,27,28,29,30,31,32,33,34,35,36};


// Variables related to the IR sensor.
    // Try to put this in an array.
int sensor_state[15][2]; // each row corresponds to each sensor.


// Creating servo objects
Servo myservo;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
Servo myservo7;
Servo myservo8;



// Initialising the functions:
//void IRInit();                             // initialize the sensor pins as inputs and setting them to HIGH.
//void attachServos();                       // attaching the pins of the servos to the object servo in code
//void zeroServo();                          // zero all of the servo 
//void zeroIRStates();
 
void setup() {
  // initialize the LED pin as an output:
  pinMode(LEDPIN, OUTPUT);
  
  IRInit();                             // initialize the sensor pins as inputs and setting them to HIGH.
  attachServos();                       // attaching the pins of the servos to the object servo in code
  zeroServo();                          // zero all of the servo 
  zeroIRStates();
  
  Serial.begin(9600);                 // Serial monitor
}


// main loop.
void loop(){
  for(int i = 0; i < 15; i++){
//    IRPins[i] = 
  }

  
}


void IRInit(){
  for (int i = 0; i < 15; i++) {
    pinMode(IRPins[i], INPUT_PULLUP);
  }
}

void zeroIRStates(){
  for (int i = 0; i < 15; i++) {
    pinMode(sensor_state[i][0], 0);
    pinMode(sensor_state[i][1], 0);
  }
  
  for (int i = 0; i < 15; i++) {
    for (int j = 0; j < 2; j++) {
      Serial.print(sensor_state[i][j]);
      Serial.print(" "); // Delimiter between elements
    }
  }
}

// attach the servos to the digital pins.
void attachServos(){
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
void zeroServo(){
  myservo.write(0);
  myservo2.write(0);
  myservo3.write(0);
  myservo4.write(0);
  myservo5.write(0);
  myservo6.write(0);
  myservo7.write(0);
  myservo8.write(0);
}





//Initial test code.
//void loop(){
//  // read the state of the pushbutton value:
//  sensorState = digitalRead(SENSORPIN1);
//  sensorState2 = digitalRead(SENSORPIN2);
//  sensorState3 = digitalRead(SENSORPIN3);
//  sensorState4 = digitalRead(SENSORPIN4);
//  sensorState5 = digitalRead(SENSORPIN5);
//  sensorState6 = digitalRead(SENSORPIN6);
//  sensorState7 = digitalRead(SENSORPIN7);
//  sensorState8 = digitalRead(SENSORPIN8);
//  sensorState9 = digitalRead(SENSORPIN9);
//  sensorState10 = digitalRead(SENSORPIN10);
//  sensorState11 = digitalRead(SENSORPIN11);
//  sensorState12 = digitalRead(SENSORPIN12);
//  sensorState13 = digitalRead(SENSORPIN13);
//  sensorState14 = digitalRead(SENSORPIN14);
//  sensorState15 = digitalRead(SENSORPIN15);
//
//  // check if the sensor beam is broken
//  // if it is, the sensorState is LOW:
//  if (sensorState == LOW || sensorState2 == LOW) {     
//    // turn LED on:
//    digitalWrite(LEDPIN, HIGH);  
//    myservo.write(30);
//  } 
//  else {
//    // turn LED off:
//    digitalWrite(LEDPIN, LOW); 
//    myservo.write(0);
//  }
//
//  if (sensorState3 == LOW || sensorState4 == LOW) {     
//    // turn LED on:
//    digitalWrite(LEDPIN, HIGH);  
//    myservo2.write(30);
//  } 
//  else {
//    // turn LED off:
//    digitalWrite(LEDPIN, LOW); 
//    myservo2.write(0);
//  }
//
//  if (sensorState5 == LOW || sensorState6 == LOW) {     
//    // turn LED on:
//    digitalWrite(LEDPIN, HIGH);  
//    myservo3.write(30);
//  } 
//  else {
//    // turn LED off:
//    digitalWrite(LEDPIN, LOW); 
//    myservo3.write(0);
//  }
//
//  if (sensorState7 == LOW || sensorState8 == LOW) {     
//    // turn LED on:
//    digitalWrite(LEDPIN, HIGH);  
//    myservo4.write(30);
//  } 
//  else {
//    // turn LED off:
//    digitalWrite(LEDPIN, LOW); 
//    myservo4.write(0);
//  }
//
//  if (sensorState9 == LOW || sensorState10 == LOW) {     
//    // turn LED on:
//    digitalWrite(LEDPIN, HIGH);  
//    myservo5.write(30);
//  } 
//  else {
//    // turn LED off:
//    digitalWrite(LEDPIN, LOW); 
//    myservo5.write(0);
//  }
//
//  if (sensorState11 == LOW || sensorState12 == LOW) {     
//    // turn LED on:
//    digitalWrite(LEDPIN, HIGH);  
//    myservo6.write(30);
//  } 
//  else {
//    // turn LED off:
//    digitalWrite(LEDPIN, LOW); 
//    myservo6.write(0);
//  }
//
//  if (sensorState13 == LOW || sensorState14 == LOW ) {     
//    // turn LED on:
//    digitalWrite(LEDPIN, HIGH);  
//    myservo7.write(30);
//  } 
//  else {
//    // turn LED off:
//    digitalWrite(LEDPIN, LOW); 
//    myservo7.write(0);
//  }
//
//  if (sensorState15 == LOW) {     
//    // turn LED on:
//    digitalWrite(LEDPIN, HIGH);  
//    myservo8.write(30);
//  } 
//  else {
//    // turn LED off:
//    digitalWrite(LEDPIN, LOW); 
//    myservo8.write(0);
//  }
//
//
//  // serial monitor debug code.
////  if (sensorState && !lastState) {
////    Serial.println("Unbroken");
////  } 
////  if (!sensorState && lastState) {
////    Serial.println("Broken");
////  }
//  lastState = sensorState;
//  lastState2 = sensorState2;
//  lastState3 = sensorState3;
//  lastState4 = sensorState4;
//  lastState5 = sensorState5;
//  lastState6 = sensorState6;
//  lastState7 = sensorState7;
//  lastState8 = sensorState8;
//  lastState9 = sensorState9;
//  lastState10 = sensorState10;
//  lastState11 = sensorState11;
//  lastState12 = sensorState12;
//  lastState13 = sensorState13;
//  lastState14 = sensorState14;
//}
