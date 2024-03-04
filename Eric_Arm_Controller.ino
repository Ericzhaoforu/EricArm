/*
This progamm is developed by Eric to controll the robot arm
The code took the ST3215 Driver as a reference
The object of this project is to utilize remote control of the arm , including set the initialize position and open loop control
*/

#include "SD.h"

#include "Eric_Interface.h"
void setup() {
  // put your setup code here, to run once:
  Interface_Setup();
  // simpleLinkageIkRad(L1,L2,L1*cos(M_PI/6)+L2*cos(M_PI/3),L1*sin(M_PI/6)+L2*sin(M_PI/3));
  //CoordinateCtrl(L1,L2,0);
  //98.426->231.198
  //CoordinateCtrl(L1*cos(M_PI/6)+L2*cos(M_PI/3),L1*sin(M_PI/6)+L2*sin(M_PI/3),0);
  //Serial.printf("originX:%lf,Y:%lf\n",L1*cos(M_PI/6)+L2*cos(M_PI/3),L1*sin(M_PI/6)+L2*sin(M_PI/3));
  

  // lastX = L1;
  // lastY=  L2;
  delay(5000);
  
  kinematics_update_X_Y_T(1);
  
  delay(3000);
  // RoArmM2_allPosAbsBesselCtrl(113.74543,223.593208,0,0.36);
  // delay(8000);
  // RoArmM2_allPosAbsBesselCtrl(198.5757,115.865,0,0.36);
  // delay(8000);
  // RoArmM2_allPosAbsBesselCtrl(86.49,187.3199,0,0.36);
  // delay(8000);
  // RoArmM2_allPosAbsBesselCtrl(148.5286,162.8530,0,0.36);
  // delay(8000);
  // RoArmM2_allPosAbsBesselCtrl(113.74543,223.593208,0,0.36);

  Serial.printf("Goal pos elbow:%f\n",goalpos[ELBOW]);
  Serial.printf("Goal pos wrist:%f\n",goalpos[WRIST_2]);
  Serial.printf("Goal pos wrist spin:%f\n",goalpos[WRIST]);
}

void loop() {
  // put your main code here, to run repeatedly: 
    delay(500);
    //Serial.println(time_us);
}
