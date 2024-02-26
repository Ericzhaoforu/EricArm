/*
This progamm is developed by Eric to controll the robot arm
The code took the ST3215 Driver as a reference
The object of this project is to utilize remote control of the arm , including set the initialize position and open loop control
*/


#include "Eric_Interface.h"
void setup() {
  // put your setup code here, to run once:
  Interface_Setup();
  // simpleLinkageIkRad(L1,L2,L2*cos(M_PI/6)+L2*cos(M_PI/3),L1*sin(M_PI/6)+L2*sin(M_PI/3));
  CoordinateCtrl(L1,L2,0);
  goalPosMove();

  lastX = L1;
  lastY=  L2;
  //delay(5000);

  //RoArmM2_allPosAbsBesselCtrl(L1*cos(M_PI/4)+L2*cos(M_PI/6+M_PI/4),L1*sin(M_PI/4)+L2*sin(M_PI/6+M_PI/4),M_PI/4,0.36);
  
  Serial.printf("Goal pos elbow:%f\n",goalpos[ELBOW]);
  Serial.printf("Goal pos wrist:%f\n",goalpos[WRIST_2]);
  Serial.printf("Goal pos wrist spin:%f\n",goalpos[WRIST]);
}

void loop() {
  // put your main code here, to run repeatedly: 
    delay(500);
    //Serial.println(time_us);
}

