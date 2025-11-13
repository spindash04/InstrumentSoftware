// include stepper library
#include <AccelStepper.h>

// define steps per revolution
float theta_degperstep = 0.18;  // 10:1 gearbox
float P_degperstep = 0.018;  // 100:1 gearbox
float A_degperstep = 0.018;  // 100:1 gearbox
float H_mmperstep = 8.0/200.0;
float Xtilt_mmperstep = 8.0/200.0;
float Ytilt_mmperstep = 8.0/200.0;

int speed = 800;  // set max motor sped
int accel = 500;  // set max motor acceleration

long steps;
float positions[2];  // array to hold current arm positions for simultaneous movement 

// define pins
const int theta_enablepin = 2;  // pins associated with incident arm
const int theta_dirpin = 3;
const int theta_steppin = 4;
const int theta_homepin = 32;
const int theta_endpin = 33; 

const int thetaR_enablepin = 5;  // pins associated with reflected arm
const int thetaR_dirpin = 6;
const int thetaR_steppin = 7;
const int thetaR_homepin = 30;
const int thetaR_endpin = 31;  

const int P_enablepin = 8;  // pins associated with polariser
const int P_dirpin = 9;
const int P_steppin = 10;
const int P_homepin = 28;
const int P_endpin = 29; 

const int A_enablepin = 11;  // pins associated with analyser 
const int A_dirpin = 12;
const int A_steppin = 13;
const int A_homepin = 26;
const int A_endpin = 27;  

const int H_enablepin = 40;  // pins associated with height adjust
const int H_dirpin = 41;
const int H_steppin = 42;
const int H_homepin = 34;
const int H_endpin = 35;  

const int Xtilt_enablepin = 43;  // pins associated with x-tilt
const int Xtilt_dirpin = 44;
const int Xtilt_steppin = 45;
const int Xtilt_homepin = 38;
const int Xtilt_endpin = 39;  

const int Ytilt_enablepin = 46;  // pins associated with y-tilt
const int Ytilt_dirpin = 47;
const int Ytilt_steppin = 48;
const int Ytilt_homepin = 36;
const int Ytilt_endpin = 37; 

// initialise motor positions 
float theta_pos = 0.0;
float thetaR_pos = 0.0;
float P_pos = 0.0;
float A_pos = 0.0;
float H_pos = 0.0;
float Xtilt_pos = 0.0;
float Ytilt_pos = 0.0;

// initialise stepper library on motor shield
AccelStepper steppertheta(1, theta_steppin, theta_dirpin);
AccelStepper stepperthetaR(1, thetaR_steppin, thetaR_dirpin);
AccelStepper stepperP(1, P_steppin, P_dirpin);
AccelStepper stepperA(1, A_steppin, A_dirpin);
AccelStepper stepperH(1, H_steppin, H_dirpin);
AccelStepper stepperXtilt(1, Xtilt_steppin, Xtilt_dirpin);
AccelStepper stepperYtilt(1, Ytilt_steppin, Ytilt_dirpin);


// general homing function
float home(AccelStepper stepper, int enablepin, int homepin, float pos) {

  stepper.move(-150000);  // runs motor towards home switch
  digitalWrite(enablepin, HIGH);  // ensure motor is enabled
   
  while (digitalRead(homepin)==HIGH) {  // motor runs until home switch triggered
      stepper.run();
  }

  stepper.setCurrentPosition(0);  // prevents motor turning in wrong direction when new move called

  pos = 0.0;  // position reset when home reached 
  
  Serial.println(pos);  // prints current position to serial monitor 

return pos;

}


// simultaneous homing function
void shome(AccelStepper stepper, AccelStepper stepper2, String instring, int enablepin, int enablepin2, int homepin, int homepin2, float pos, float pos2) {

  stepper.move(-150000);  // runs motors towards home switch
  stepper2.move(-150000);

  digitalWrite(enablepin, HIGH);  // ensure motors are enabled
  digitalWrite(enablepin2, HIGH);

  bool home_pins_check = false;  // boolean variable to check if both home pins enabled 

  while (!home_pins_check) {  // while both home pins are still disabled 
    int home_pin_check = digitalRead(homepin);
    int home_pin2_check = digitalRead(homepin2);

    if (home_pin_check==HIGH) stepper.run();  // run incident arm motor if incident home pin still disabled
    if (home_pin2_check==HIGH) stepper2.run();  // run reflected arm motor if reflected home pin still disabled 

    home_pins_check = (home_pin_check==LOW) && (home_pin2_check==LOW);  // update check with combined home pin logic 
  }

  stepper.setCurrentPosition(0);  // prevents motors turning in wrong direction when new move called
  stepper2.setCurrentPosition(0);

  pos = 0.0;  // positions reset when home reached
  pos2 = 0.0;

  positions[0] = pos;  // saves positions to globally accessible array
  positions[1] = pos2;

  Serial.print(pos);  // prints current positions to serial monitor 
  Serial.print(',');
  Serial.println(pos2); 

}


// end stop function
void end(AccelStepper stepper, int enablepin, int endpin) {

  stepper.move(150000);  // runs motor towards end switch
  digitalWrite(enablepin, HIGH);  // ensure motor is enabled 

  while (digitalRead(endpin) == HIGH) {  // motor runs until end stop switch triggered
      stepper.run();
  }

  stepper.setCurrentPosition(0);  // prevents motor turning in wrong direction when new move called

return Serial.println("Moved to end stop");

}


// general movement function
float move(AccelStepper stepper, String instring, int enablepin, int homepin, int endpin, float pos, float perstep) {
  
  int movestringindex = instring.indexOf(' ');
  String movestring = instring.substring(movestringindex + 1);  // read new position only from communication string
  float new_pos = movestring.toFloat();  // obtain new position and convert to float
  
  long steps = round((new_pos - pos) / perstep);  // calculate steps needed to move to new position

  digitalWrite(enablepin, HIGH);  // ensure motor is enabled

  stepper.move(-steps);  // tell motor how many steps to take

  if (steps > 0) {  // if movement in positive direction
    while (abs(stepper.distanceToGo()) > 0 && digitalRead(endpin) == HIGH) {  // run motor until no more steps remaining or end stop reached
      stepper.run();
    }
  }
  else{  // if movement in negative direction
     while (abs(stepper.distanceToGo()) > 0 && digitalRead(homepin) == HIGH) {  // run motor until no more steps remaining or home stop reached
      stepper.run();
    } 
  }

  if (steps < 0) {  // if movement in negative direction
    pos = round(pos - (abs(steps) - abs(stepper.distanceToGo())) * perstep);  // calculate new position based on steps taken
  }
  else {  // if movement in positive direction
    pos = round(pos + (abs(steps) - abs(stepper.distanceToGo())) * perstep);  // calculate new position based on steps taken
  }

  stepper.setCurrentPosition(0);  // prevents motor turning in wrong direction when new move called 

  Serial.println(pos);  // print current position to serial monitor 

return pos;

}


// simultaneous arm movement function
void smove(AccelStepper stepper, AccelStepper stepper2, String instring, int enablepin, int enablepin2, float pos, float pos2, float perstep) {

   int movestringindex = instring.indexOf(' ');
   String movestring = instring.substring(movestringindex + 1);  // read new position only from communication string

   int movestringindex2 = movestring.indexOf(',');
   String Imovestring = movestring.substring(0, movestringindex2);  // obtain new position of incident arm
   String Rmovestring = movestring.substring(movestringindex2 + 1);  // obtain new position of reflected arm

   float new_pos = Imovestring.toFloat();  // convert new position to float
   float new_pos2 = Rmovestring.toFloat();  // convert new position to float

   long steps = round((new_pos - pos) / perstep);  // calculate steps needed to move incident arm to new position
   long steps2 = round((new_pos2 - pos2) / perstep);  // calculate steps needed to move reflected arm to new position

   digitalWrite(enablepin, HIGH);  // ensure motors are enabled
   digitalWrite(enablepin2, HIGH);

   stepper.move(-steps);  // tell motor for incident arm how many steps to take
   stepper2.move(-steps2);  // tell motor for reflected arm how many steps to take
  
   int steps_complete_check = 1; 
 
   while (steps_complete_check > 0) {  // while both motors still need to make steps
    int steps_remaining = stepper.distanceToGo(); 
    int steps2_remaining = stepper2.distanceToGo(); 

    if (abs(steps_remaining) > 0) stepper.run();  // run incident arm motor if steps still remain
    if (abs(steps2_remaining) > 0) stepper2.run();  // run reflected arm motor if steps still remain 
      
    steps_complete_check = abs(steps_remaining) + abs(steps2_remaining);  // update check with combined remaining steps
   }

   if (steps < 0) {  // if incident arm movement in negative direction
     pos = new_pos + abs(stepper.distanceToGo()) * perstep;  // calculate new position based on steps taken
   }
   else{  // if incident arm movement in positive direction 
    pos = new_pos - abs(stepper.distanceToGo()) * perstep;  // calculate new position based on steps taken
   }

   if (steps2 < 0) {  // if reflected arm movement in negative direction
    pos2 = new_pos2 + abs(stepper2.distanceToGo()) * perstep;  // calculate new position based on steps taken
   }
   else{  // if reflected arm movement in positive direction
    pos2 = new_pos2 - abs(stepper2.distanceToGo()) * perstep;  // calculate new position based on steps taken
   }
  
  positions[0] = pos;  // save current positions to globally accessible array
  positions[1] = pos2;

  stepper.setCurrentPosition(0);  // prevents motors turning in wrong direction when new move called
  stepper2.setCurrentPosition(0);

  Serial.print(pos);  // print current positions to serial monitor 
  Serial.print(',');
  Serial.println(pos2); 

}


// setup function
void setup() {
  
  Serial.begin(115200);

  steppertheta.setMaxSpeed(speed);  // set speed of stepper motors
  stepperthetaR.setMaxSpeed(speed);
  stepperP.setMaxSpeed(speed);
  stepperA.setMaxSpeed(speed);
  stepperH.setMaxSpeed(speed);
  stepperXtilt.setMaxSpeed(speed);
  stepperYtilt.setMaxSpeed(speed);

  steppertheta.setAcceleration(accel);  // set acceleration of stepper motors
  stepperthetaR.setAcceleration(accel);
  stepperP.setAcceleration(accel);
  stepperA.setAcceleration(accel);
  stepperH.setAcceleration(accel);
  stepperXtilt.setAcceleration(accel);
  stepperYtilt.setAcceleration(accel);

  pinMode(theta_homepin, INPUT_PULLUP);   // set switch pin modes
  pinMode(theta_endpin, INPUT_PULLUP);
  pinMode(thetaR_homepin, INPUT_PULLUP);
  pinMode(thetaR_endpin, INPUT_PULLUP);
  pinMode(P_homepin, INPUT_PULLUP);
  pinMode(P_endpin, INPUT_PULLUP);
  pinMode(A_homepin, INPUT_PULLUP);
  pinMode(A_endpin, INPUT_PULLUP);
  pinMode(H_homepin, INPUT_PULLUP);
  pinMode(H_endpin, INPUT_PULLUP);
  pinMode(Xtilt_homepin, INPUT_PULLUP);
  pinMode(Xtilt_endpin, INPUT_PULLUP);
  pinMode(Ytilt_homepin, INPUT_PULLUP);
  pinMode(Ytilt_endpin, INPUT_PULLUP);
  
  pinMode(theta_enablepin, OUTPUT);  // set enable pin modes
  pinMode(thetaR_enablepin, OUTPUT);
  pinMode(P_enablepin, OUTPUT);
  pinMode(A_enablepin, OUTPUT);
  pinMode(H_enablepin, OUTPUT);
  pinMode(Xtilt_enablepin, OUTPUT);
  pinMode(Ytilt_enablepin, OUTPUT);

  digitalWrite(theta_enablepin, HIGH);  // permanently enable stepper motors
  digitalWrite(thetaR_enablepin, HIGH);
  digitalWrite(P_enablepin, HIGH);
  digitalWrite(A_enablepin, HIGH);
  digitalWrite(H_enablepin, HIGH);
  digitalWrite(Xtilt_enablepin, HIGH);
  digitalWrite(Ytilt_enablepin, HIGH);

}


// loop function
void loop() {

  if (Serial.available() > 0) {  // check command is available at serial port

    String incomingstring = Serial.readString();  // read incoming string
    String comstring = incomingstring.substring(0, 5);  // obtain first 5 letters of communication string

    if (comstring == "HOMEI") {
      theta_pos = home(steppertheta, theta_enablepin, theta_homepin, theta_pos);  // home incident arm and update position
    }

    else if (comstring == "HOMER") {
      thetaR_pos = home(stepperthetaR, thetaR_enablepin, thetaR_homepin, thetaR_pos);  // home reflected arm and update position
    }

    else if (comstring == "HOMEP") {
      P_pos = home(stepperP, P_enablepin, P_homepin, P_pos);  // home polariser and update position
    }

    else if (comstring == "HOMEA") {
      A_pos = home(stepperA, A_enablepin, A_homepin, A_pos);  // home analyser and update position
    }

    else if (comstring == "HOMEH") {
      H_pos = home(stepperH, H_enablepin, H_homepin, H_pos);  // home height adjust and update position
    }

    else if (comstring == "HOMEX") {
      Xtilt_pos = home(stepperXtilt, Xtilt_enablepin, Xtilt_homepin, Xtilt_pos);  // home x-tilt and update position
    }

    else if (comstring == "HOMEY") {
      Ytilt_pos = home(stepperYtilt, Ytilt_enablepin, Ytilt_homepin, Ytilt_pos);  // home y-tilt and update position
    }

    else if (comstring == "HOMIR") {
      shome(steppertheta, stepperthetaR, incomingstring, theta_enablepin, thetaR_enablepin, theta_homepin, thetaR_homepin, theta_pos, thetaR_pos);  // home incident and reflected arms
      theta_pos = positions[0];  // update position of incident arm from globally accessible array
      thetaR_pos = positions[1];  // update position of reflected arm from globally accessible array    
    }

    else if (comstring == "ENDSI") {
      end(steppertheta, theta_enablepin, theta_endpin);  // end stop incident arm 
    }

    else if (comstring == "ENDSR") {
      end(stepperthetaR, thetaR_enablepin, thetaR_endpin);  // end stop reflected arm
    }

    else if (comstring == "ENDSP") {
      end(stepperP, P_enablepin, P_endpin);  // end stop polariser 
    }

    else if (comstring == "ENDSA") {
      end(stepperA, A_enablepin, A_endpin);  // end stop analyser
    }

    else if (comstring == "ENDSH") {
      end(stepperH, H_enablepin, H_endpin);  // end stop height adjust
    }

    else if (comstring == "ENDSX") {
      end(stepperXtilt, Xtilt_enablepin, Xtilt_endpin);  // end stop x-tilt
    }

    else if (comstring == "ENDSY") {
      end(stepperYtilt, Ytilt_enablepin, Ytilt_endpin);  // end stop y-tilt
    }

    else if (comstring == "MOVEI") {
      theta_pos = move(steppertheta, incomingstring, theta_enablepin, theta_endpin, theta_homepin, theta_pos, theta_degperstep);  // move incident arm and update position 
    }

    else if (comstring == "MOVER") {
      thetaR_pos = move(stepperthetaR, incomingstring, thetaR_enablepin, thetaR_endpin, thetaR_homepin, thetaR_pos, theta_degperstep);  // move reflected arm and update position
    }

    else if (comstring == "MOVEP") {
      P_pos = move(stepperP, incomingstring, P_enablepin, P_endpin, P_homepin, P_pos, P_degperstep);  // move polariser and update position
    }

    else if (comstring == "MOVEA") {
      A_pos = move(stepperA, incomingstring, A_enablepin, A_endpin, A_homepin, A_pos, A_degperstep);  // move analyser and update position
    } 

    else if (comstring == "MOVEH") {
      H_pos = move(stepperH, incomingstring, H_enablepin, H_endpin, H_homepin, H_pos, H_mmperstep);  // move height adjust and update position
    }

    else if (comstring == "MOVEX") {
      Xtilt_pos = move(stepperXtilt, incomingstring, Xtilt_enablepin, Xtilt_endpin, Xtilt_homepin, Xtilt_pos, Xtilt_mmperstep);  // move x-tilt and update position
    }

    else if (comstring == "MOVEY") {
      Ytilt_pos = move(stepperYtilt, incomingstring, Ytilt_enablepin, Ytilt_endpin, Ytilt_homepin, Ytilt_pos, Ytilt_mmperstep);  // move y-tilt and update position
    }

    else if (comstring == "MOVIR") {
      smove(steppertheta, stepperthetaR, incomingstring, theta_enablepin, thetaR_enablepin, theta_pos, thetaR_pos, theta_degperstep);  // move incident and reflected arms simultaneously
      theta_pos = positions[0];  // update position of incident arm from globally accessible array
      thetaR_pos = positions[1];  // update position of reflected arm from globally accessible array
    }

    else if (comstring == "GETII") {
      Serial.println(theta_pos);  // get incident arm position in degrees
    }

    else if (comstring == "GETRR") {
      Serial.println(thetaR_pos);  // get reflected arm position in degrees
    }

    else if (comstring == "GETPP") {
      Serial.println(P_pos);  // get polariser position in degrees
    }

    else if (comstring == "GETAA") {
      Serial.println(A_pos);  // get analyser position in degrees
    }

    else if (comstring == "GETHH") {
      Serial.println(H_pos);  // get height adjust position in mm
    }

    else if (comstring == "GETXX") {
      Serial.println(Xtilt_pos);  // get x-tilt position in mm
    }

    else if (comstring == "GETYY") {
      Serial.println(Ytilt_pos);  // get y-tilt position in mm 
    }
  }
}


