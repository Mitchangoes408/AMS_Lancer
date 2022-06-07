#include <SoftwareSerial.h>
#include <Arduino_JSON.h>

/*
   AMS for the Lancer using an arduino to control 8 relays, 2 per corner, via bluetooth.

   Using bluetooth, the arduino will communicate to an Android application to either air up or down any or all specified airbags.
*/

// RELAY PINS
const int relay1_Pin = 1;
const int relay2_Pin = 2;
const int relay3_Pin = 3;
const int relay4_Pin = 4;
const int relay5_Pin = 5;
const int relay6_Pin = 6;
const int relay7_Pin = 7;
const int relay8_Pin = 8;

//  HC-05 PINS
const int transOut = 10;
const int recIn = 11;

//  CHECK VARIABLE FOR SYSTEM STARTUP TASK
bool system_init = false;
bool process_complete = false;

//  RIDE PRESETS
const int rideHeight = 50;
const int FD_Ride = 50;
const int FP_Ride = 50;
const int RD_Ride = 50;
const int RP_Ride = 50;

const int cruiseHeight = 35;
const int FD_Cruise = 35;
const int FP_Cruise = 35;
const int RD_Cruise = 35;
const int RP_Cruise = 35;

//  CHECK VARIABLES FOR EACH RELAY PAIR
bool bag_FD_Complete = false;
bool bag_FP_Complete = false;
bool bag_RD_Complete = false;
bool bag_RP_Complete = false;

char input;

int FD_Curr = 0;
int FP_Curr = 0;
int RD_Curr = 0;
int RP_Curr = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //SET PIN MODES
  /*  RELAY PINS  */
  pinMode(relay1_Pin, OUTPUT);
  pinMode(relay2_Pin, OUTPUT);
  pinMode(relay3_Pin, OUTPUT);
  pinMode(relay4_Pin, OUTPUT);
  pinMode(relay5_Pin, OUTPUT);
  pinMode(relay6_Pin, OUTPUT);
  pinMode(relay7_Pin, OUTPUT);
  pinMode(relay8_Pin, OUTPUT);

  /*  USING PORT MANIPULATION */
  DDRD = B11111111;

  /*  HC-05 PINS  */

}

void loop() {
  // put your main code here, to run repeatedly:
  if (!system_init) {
    onStartup();
  }

  //REACT TO INPUT
  while (Serial.available() > 0) {
    input = Serial.read();
    Serial.println(input);
  }

  switch (input) {
    case 'A':
      Serial.println("Air up FrontDriver.");
      break;

    case 'a':
      Serial.println("Air down FrontDriver.");
      break;

    case 'B':
      Serial.println("Air up FrontPassenger.");
      break;

    case 'b':
      Serial.println("Air down FrontPassenger.");
      break;

    case 'C':
      Serial.println("Air up RearDriver.");
      break;

    case 'c':
      Serial.println("Air down RearDriver.");
      break;

    case 'D':
      Serial.println("Air up RearPassenger.");
      break;

    case 'd':
      Serial.println("Air down RearPassenger.");
      break;

    case 'R':
      airTo_Ride();
      break;

    case 'S':
      airTo_Slam();
      break;

    case 'K':
      airTo_Cruise();
      break;
        
    case 'Z':
        onShutDown();
        break;

    default:
      Serial.println("Invalid input received.");
  }

}

void onStartup() {
  Serial.println("Start up procedures IN PROGRESS...");
  /*
      TEST SEQUENCE:
          Iterate through relays
          Hold ON for 0.5 second each, then OFF
          Flash following sequence:
            Odd relays once for 0.5 seconds
            Even relays once for 0.5 seconds
            All relays twice for 0.5 seconds
  */
  for (int i = 1; i < 9; i++) {
    digitalWrite(i, HIGH);
    delay(250);
    digitalWrite(i, LOW);
  }

  PORTD = B01010101;
  delay(250);
  PORTD = B00000000;

  delay(250);
  PORTD = B10101010;
  delay(250);
  PORTD = B00000000;

  for (int i = 0; i < 2; i++) {
    delay(250);
    PORTD = B11111111;
    delay(250);
    PORTD = B00000000;
  }

  system_init = true;
  Serial.println("Start up procedures COMPLETE.");
}

void onShutDown() {
  Serial.println("Shut down procedures IN PROGRESS...");
  airTo_Slam();

  system_init = false;
  Serial.println("Shut down procedure COMPLETE.");
}

void airTo_Cruise() {
  Serial.println("airTo_Cruise process IN PROGRESS...");
  process_complete = false;

  int count = 0;    //replace count with reading from pressure sensors
  while (!process_complete) {
    if(count == cruiseHeight) {
      process_complete = true;
    }

    count++;
  }

  Serial.println("airTo_Cruise process COMPLETE.");
}

void airTo_Ride() {
  Serial.println("airTo_Ride process IN PROGRESS...");
  process_complete = false;

  int count = 0;    //replace count with reading from pressure sensors
  while (!process_complete) {
    if(count == rideHeight) {
      process_complete = true;
    }

    count++;
  }

  Serial.println("airTo_Ride process COMPLETE.");
}

void airTo_Slam() {
  Serial.println("airTo_Slam process IN PROGRESS...");
  process_complete = false;

  /*  ASS TO THE GRASS  */
  while (!process_complete) {
    PORTD = B01010101;

    if (FD_Curr == 0 && FP_Curr == 0 && RD_Curr == 0 && RP_Curr == 0) {
      process_complete = true;
    }
  }

  Serial.println("airTo_Slam process COMPLETE.");
  PORTD = B00000000;
}
