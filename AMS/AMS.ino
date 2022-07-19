#include <SoftwareSerial.h>
#include <Arduino_JSON.h>

/*
   AMS for the Lancer using an arduino to control 8 relays, 2 per corner, via bluetooth.

   Using bluetooth, the arduino will communicate to an Android application to either air up or down any or all specified airbags.
*/

//  FUNCTION DECLARATIONS
void onStartup();
void onShutDown();

void airTo_Cruise();
void airTo_Ride();
void airTo_Slam();

void checkCruise(float, int);
void checkRide(float, int);

float adcTo_V();
float fmap(float, float, float, float, float);

void calibrate_Sensor();
void monitorSystem();



//  RELAY PINS
const int relay1_Pin = 22;
const int relay2_Pin = 23;
const int relay3_Pin = 24;
const int relay4_Pin = 25;
const int relay5_Pin = 26;
const int relay6_Pin = 27;
const int relay7_Pin = 28;
const int relay8_Pin = 29;

//  HC-05 PINS
const int transOut = 10;
const int recIn = 11;

//  AIR PRESSURE SENSOR PINS
const int FD_Pin = A0;
const int FP_Pin = A1;
const int RD_Pin = A2;
const int RP_Pin = A3;
const int tank_Pin = A4;


//  CHECK VARIABLE FOR SYSTEM TASKS
bool system_init = false;
bool process_complete = false;

bool atCruise = false;
bool atRide = false;
bool atSlam = false;

unsigned long t = 0;  // time tracking variable


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

const int tolerance = 5;

//  CHECK VARIABLES FOR EACH RELAY PAIR
bool bag_FD_Complete = false;
bool bag_FP_Complete = false;
bool bag_RD_Complete = false;
bool bag_RP_Complete = false;

bool FD_OutOfRange = false;
bool FP_OutOfRange = false;
bool RD_OutOfRange = false;
bool RP_OutOfRange = false;

//  INPUT CHAR FROM BLUETOOTH
char input;

float FD_Curr = 0;
float FP_Curr = 0;
float RD_Curr = 0;
float RP_Curr = 0;
float tank_Curr = 0;
float corners_Curr [] = { 0, 0, 0, 0};  //in case i want to use an array to store values instead

int sensorValue = 0;
int sensorOut = 0;

//  SENSOR CALIBRATION VALUES
//  FRONT DRIVER
int FD_Min = 1024;
int FD_Max = 0;
//  FRONT PASSENGER
int FP_Min = 1024;
int FP_Max = 0;
//  REAR DRIVER
int RD_Min = 1024;
int RD_Max = 0;
//  REAR PASSENGER
int RP_Min = 1024;
int RP_Max = 0;
//  TANK
int tank_Min = 1024;
int tank_Max = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //SET PIN MODES
  /*  RELAY PINS  */
  pinMode(relay1_Pin, OUTPUT);    //Pair1 UP
  pinMode(relay2_Pin, OUTPUT);    //Pair1 DOWN
  pinMode(relay3_Pin, OUTPUT);    //Pair2 UP
  pinMode(relay4_Pin, OUTPUT);    //Pair2 DOWN
  pinMode(relay5_Pin, OUTPUT);    //Pair3 UP
  pinMode(relay6_Pin, OUTPUT);    //Pair3 DOWN
  pinMode(relay7_Pin, OUTPUT);    //Pair4 UP
  pinMode(relay8_Pin, OUTPUT);    //Pair4 DOWN

  /*  USING PORT MANIPULATION
      - all ports used as outputs
  */
  DDRA = B11111111;

  /*  HC-05 PINS  */

  onStartup();
}

void loop() {
  //  put your main code here, to run repeatedly:
  const int serialPrintInterval = 1500; //increase value to slow down serial print activity

  //  HOLD AND WAIT FOR INPUT
  while (Serial.available() == 0) {
    //  MONITOR SENSOR READINGS
    if (millis() > t + serialPrintInterval) {
      monitorSystem();

      t = millis();
    }

  }

  input = Serial.read();
  Serial.println(input);

  //  USING SWITCH CASE FOR INPUTS
  switch (input) {
    case 'A':
      Serial.println("Air up FrontDriver.");
      digitalWrite(relay1_Pin, HIGH);
      break;

    case 'B':
      Serial.println("Air down FrontDriver.");
      digitalWrite(relay2_Pin, HIGH);
      break;

    case 'C':
      Serial.println("Air up FrontPassenger.");
      digitalWrite(relay3_Pin, HIGH);
      break;

    case 'D':
      Serial.println("Air down FrontPassenger.");
      digitalWrite(relay4_Pin, HIGH);
      break;

    case 'E':
      Serial.println("Air up RearDriver.");
      digitalWrite(relay5_Pin, HIGH);
      break;

    case 'F':
      Serial.println("Air down RearDriver.");
      digitalWrite(relay6_Pin, HIGH);
      break;

    case 'G':
      Serial.println("Air up RearPassenger.");
      digitalWrite(relay7_Pin, HIGH);
      break;

    case 'H':
      Serial.println("Air down RearPassenger.");
      digitalWrite(relay8_Pin, HIGH);
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

    default:
      Serial.println("Invalid input received.");
      PORTA &= B00000000;

  }

  //  USING IF STATEMENTS FOR INPUTS
  if(input == 'A') {
    digitalWrite(relay1_Pin, HIGH);
  }
  if(input == 'a') {
    digitalWrite(relay1_Pin, LOW);
  }

  if(input == 'B') {
    digitalWrite(relay2_Pin, HIGH);
  }
  if(input == 'b') {
    digitalWrite(relay2_Pin, LOW);
  }

  if(input == 'C') {
    digitalWrite(relay3_Pin, HIGH);
  }
  if(input == 'c') {
    digitalWrite(relay3_Pin, LOW);
  }

  if(input == 'D') {
    digitalWrite(relay4_Pin, HIGH);
  }
  if(input == 'd') {
    digitalWrite(relay4_Pin, LOW);
  }

  if(input == 'E') {
    digitalWrite(relay5_Pin, HIGH);
  }
  if(input == 'e') {
    digitalWrite(relay5_Pin, LOW);
  }

  if(input == 'F') {
    digitalWrite(relay6_Pin, HIGH);
  }
  if(input == 'f') {
    digitalWrite(relay6_Pin, LOW);
  }

  if(input == 'G') {
    digitalWrite(relay7_Pin, HIGH);
  }
  if(input == 'g') {
    digitalWrite(relay7_Pin, LOW);
  }

  if(input == 'H') {
    digitalWrite(relay8_Pin, HIGH);
  }
  if(input == 'h') {
    digitalWrite(relay8_Pin, LOW);
  }

  if(input == 'K') {
    airTo_Cruise();
  }

  if(input == 'R') {
    airTo_Ride();
  }

  if(input == 'S') {
    airTo_Slam();
  }

  delay(10);

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
  for (int i = relay1_Pin; i < relay8_Pin; i++) {
    digitalWrite(i, HIGH);
    delay(250);
    digitalWrite(i, LOW);
  }


  PORTA |= B01010101;
  delay(500);
  PORTA &= B00000000;

  delay(500);
  PORTA |= B10101010;
  delay(500);
  PORTA &= B00000000;

  for (int i = 0; i < 2; i++) {
    delay(500);
    PORTA |= B11111111;
    delay(500);
    PORTA &= B00000000;
  }

  //airTo_Slam();    //  ATSLAM SINCE SYSTEM CLOSES ATSLAM
  PORTA |= B11111111;
  system_init = true;
  Serial.println("Start up procedures COMPLETE.");
}

void onShutDown() {
  Serial.println("Shut down procedures IN PROGRESS...");
  airTo_Slam();

  //system_init = false;
  Serial.println("Shut down procedure COMPLETE.");
}

void airTo_Cruise() {
  Serial.println("airTo_Cruise process IN PROGRESS...");
  process_complete = false;
  atRide = false;
  atSlam = false;

  while (!process_complete) {
    //  GET CURRENT CORNER READINGS
    FD_Curr = adcTo_PSI(analogRead(FD_Pin));
    FP_Curr = adcTo_PSI(analogRead(FP_Pin));
    RD_Curr = adcTo_PSI(analogRead(RD_Pin));
    RP_Curr = adcTo_PSI(analogRead(RP_Pin));

    //  CHECK IF THE CORNER READINGS ARE IN RANGE
    checkCruise(FD_Curr, FD_Pin);
    checkCruise(FP_Curr, FP_Pin);
    checkCruise(RD_Curr, RD_Pin);
    checkCruise(RP_Curr, RP_Pin);

    //  START TYING UP AIRING PROCESS IF CORNERS ARE DONE
    if (bag_FD_Complete && bag_FP_Complete && bag_RD_Complete && bag_RP_Complete) {
      //  ENSURE ALL RELAYS OFF
      PORTA &= B00000000;

      bag_FD_Complete = false;
      bag_FP_Complete = false;
      bag_RD_Complete = false;
      bag_RP_Complete = false;

      process_complete = true;
    }

    //  START AIRING PROCESS
    if (atSlam) {
      //  AIR UP FROM SLAM
      PORTA |= B10101010;
    } else if (atRide) {
      //  AIR DOWN FROM RIDE HEIGHT
      PORTA |= B01010101;
    }
  }

  atCruise = true;
  Serial.println("airTo_Cruise process COMPLETE.");
}

void airTo_Ride() {
  Serial.println("airTo_Ride process IN PROGRESS...");
  process_complete = false;
  atCruise = false;
  atSlam = false;

  while (!process_complete) {
    //  GET CURRENT CORNER READINGS
    FD_Curr = adcTo_PSI(analogRead(FD_Pin));
    FP_Curr = adcTo_PSI(analogRead(FP_Pin));
    RD_Curr = adcTo_PSI(analogRead(RD_Pin));
    RP_Curr = adcTo_PSI(analogRead(RP_Pin));

    //  CHECK IF THE CORNER READINGS ARE IN RANGE
    checkRide(FD_Curr, FD_Pin);
    checkRide(FP_Curr, FP_Pin);
    checkRide(RD_Curr, RD_Pin);
    checkRide(RP_Curr, RP_Pin);

    //  START TYING UP AIRING PROCESS IF CORNERS ARE DONE
    if (bag_FD_Complete && bag_FP_Complete && bag_RD_Complete && bag_RP_Complete) {
      //  ENSURE ALL RELAYS OFF
      PORTA &= B00000000;

      bag_FD_Complete = false;
      bag_FP_Complete = false;
      bag_RD_Complete = false;
      bag_RP_Complete = false;

      process_complete = true;
    }

    //AIR UP
    PORTA |= B10101010;
  }

  atRide = true;
  Serial.println("airTo_Ride process COMPLETE.");
}

void airTo_Slam() {
  Serial.println("airTo_Slam process IN PROGRESS...");
  process_complete = false;
  atCruise = false;
  atRide = false;

  /*  ASS TO THE GRASS  */
  while (!process_complete) {
    FD_Curr = adcTo_PSI(analogRead(FD_Pin));
    FP_Curr = adcTo_PSI(analogRead(FP_Pin));
    RD_Curr = adcTo_PSI(analogRead(RD_Pin));
    RP_Curr = adcTo_PSI(analogRead(RP_Pin));

    PORTA |= B01010101;

    if (FD_Curr == 0 && FP_Curr == 0 && RD_Curr == 0 && RP_Curr) {
      process_complete = true;
    }
  }

  PORTA &= B00000000;

  atSlam = true;
  Serial.println("airTo_Slam process COMPLETE.");

}

void checkCruise(float cornerPSI, int cornerPin) {
  switch (cornerPin) {
    case FD_Pin:
      if (cornerPSI >= FD_Cruise + tolerance || cornerPSI <= FD_Cruise - tolerance ) {
        PORTA &= B11111100;
        bag_FD_Complete = true;
        FD_OutOfRange = false;
      } else {
        FD_OutOfRange = true;
      }
      break;
    case FP_Pin:
      if (cornerPSI >= FP_Cruise + tolerance || cornerPSI <= FP_Cruise - tolerance ) {
        PORTA &= B11110011;
        bag_FP_Complete = true;
        FP_OutOfRange = false;
      } else {
        FP_OutOfRange = true;
      }
      break;
    case RD_Pin:
      if (cornerPSI >= RD_Cruise + tolerance || cornerPSI <= RD_Cruise - tolerance ) {
        PORTA &= B11001111;
        bag_RD_Complete = true;
        RD_OutOfRange = false;
      } else {
        RD_OutOfRange = true;
      }
      break;
    case RP_Pin:
      if (cornerPSI >= RP_Cruise + tolerance || cornerPSI <= RP_Cruise - tolerance ) {
        PORTA &= B00111111;
        bag_RP_Complete = true;
        RP_OutOfRange = false;
      } else {
        RP_OutOfRange = true;
      }
      break;
    default:
      Serial.println("Invalid cornerPin entry for checkCruise");
  }
}

void checkRide(float cornerPSI, int cornerPin) {
  //  IF WITHIN TOLERABLE RANGES, TURN ONN RELAY/SOLENOID
  switch (cornerPin) {
    case FD_Pin:
      if (cornerPSI >= FD_Ride + tolerance || cornerPSI <= FD_Ride - tolerance ) {
        PORTA &= B11111100;
        bag_FD_Complete = true;
        FD_OutOfRange = false;
      } else {
        FD_OutOfRange = true;
      }
      break;
    case FP_Pin:
      if (cornerPSI >= FP_Ride + tolerance || cornerPSI <= FP_Ride - tolerance ) {
        PORTA &= B11110011;
        bag_FP_Complete = true;
        FP_OutOfRange = false;
      } else {
        FP_OutOfRange = true;
      }
      break;
    case RD_Pin:
      if (cornerPSI >= RD_Ride + tolerance || cornerPSI <= RD_Ride - tolerance ) {
        PORTA &= B11001111;
        bag_RD_Complete = true;
        RD_OutOfRange = false;
      } else {
        RD_OutOfRange = true;
      }
      break;
    case RP_Pin:
      if (cornerPSI >= RP_Ride + tolerance || cornerPSI <= RP_Ride - tolerance ) {
        PORTA &= B00111111;
        bag_RP_Complete = true;
        RP_OutOfRange = false;
      } else {
        RP_OutOfRange = true;
      }
      break;
    default:
      Serial.println("Invalid cornerPin entry for checkRide");
  }
}



float adcTo_PSI(float analogReading) {
  //ANALOG TO VOLTAGE
  float voltage = (5 * analogReading) / 1023;
  //VOLTAGE TO PSI
  float psi = (voltage - 0.5) * 100 / (4.5 - 0.5);

  return psi;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  float returnVal = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

  return returnVal;
}

void calibrate_Sensor() {
  //  CALIBRATE SENSORS
  Serial.println("Calibrating sensors...");

  while (millis() < 5000) {
    FD_Curr = analogRead(FD_Pin);
    FP_Curr = analogRead(FP_Pin);
    RD_Curr = analogRead(RD_Pin);
    RP_Curr = analogRead(RP_Pin);
    tank_Curr = analogRead(tank_Pin);

    // record the maximum sensor value
    if (FD_Curr > FD_Max) {
      FD_Max = FD_Curr;
    }
    if (FP_Curr > FP_Max) {
      FP_Max = FP_Curr;
    }
    if (RD_Curr > RD_Max) {
      RD_Max = RD_Curr;
    }
    if (RP_Curr > RP_Max) {
      RP_Max = RP_Curr;
    }
    if (tank_Curr > tank_Max) {
      tank_Max = tank_Curr;
    }



    // record minimum sensor value
    if (FD_Curr > FD_Min) {
      FD_Min = FD_Curr;
    }
    if (FP_Curr > FP_Min) {
      FP_Min = FP_Curr;
    }
    if (RD_Curr > RD_Min) {
      RD_Min = RD_Curr;
    }
    if (RP_Curr > RP_Min) {
      RP_Min = RP_Curr;
    }
    if (tank_Curr > tank_Min) {
      tank_Min = tank_Curr;
    }
  }

  Serial.print("FD_Min = ");
  Serial.print(FD_Min);
  Serial.print("  FD_Max = ");
  Serial.println(FD_Max);

  Serial.print("FP_Min = ");
  Serial.print(FP_Min);
  Serial.print("  FP_Max = ");
  Serial.println(FP_Max);

  Serial.print("RD_Min = ");
  Serial.print(RD_Min);
  Serial.print("  RD_Max = ");
  Serial.println(RD_Max);

  Serial.print("RP_Min = ");
  Serial.print(RP_Min);
  Serial.print("  RP_Max = ");
  Serial.println(RP_Max);

  Serial.print("tank_Min = ");
  Serial.print(tank_Min);
  Serial.print("  tank_Max = ");
  Serial.println(tank_Max);

  FD_Curr = constrain(FD_Curr, FD_Min, FD_Max);
  FP_Curr = constrain(FP_Curr, FP_Min, FP_Max);
  RD_Curr = constrain(RD_Curr, RD_Min, RD_Max);
  RP_Curr = constrain(RP_Curr, RP_Min, RP_Max);
  tank_Curr = constrain(tank_Curr, tank_Min, tank_Max);

  Serial.println("Sensors calibrated.");

}

void monitorSystem() {
  FD_Curr = analogRead(FD_Pin);
  FP_Curr = analogRead(FP_Pin);
  RD_Curr = analogRead(RD_Pin);
  RP_Curr = analogRead(RP_Pin);
  tank_Curr = analogRead(tank_Pin);

  Serial.print("FD: ");
  Serial.print(FD_Curr);
  Serial.print("; FP: ");
  Serial.print(FP_Curr);
  Serial.print("; RD: ");
  Serial.print(RD_Curr);
  Serial.print("; RP: ");
  Serial.println(RP_Curr);
  Serial.print("tank_Curr: ");
  Serial.println(tank_Curr);

  delay(10);
}
