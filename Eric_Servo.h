#include "SCServo.h"
#define ELBOW_ID 13
#define WRIST_UP_DOWN_ID 11
#define WRIST_SPAN_ID 14
#define HAND_ID  15

#define DOF_NUM 4

#define ELBOW 0
#define WRIST 1
#define HAND 2
#define WRIST_2 3

#define DETECTED 1
#define NOT_FOUND 0

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#define MOTOR_MODE 3
#define SERVO_MODE 0
#define ARM_SERVO_POS_RANGE 4096

#define BESSEL_MAX_LOG_NUM 2048
double L1= 222.324;
double L2= 98.426;
// === ST Servo
SMS_STS st;
float ServoDigitalRange_ST  = 4095.0;
float Eric_Range_upper[4] = {2700.0, 4095.0, 4095.0 ,3410.0};
float ServoDigitalMiddle_ST = 2047.0;
float goalpos[4] = {ServoDigitalMiddle_ST,ServoDigitalMiddle_ST,ServoDigitalMiddle_ST,ServoDigitalMiddle_ST};
float ServoAngleRange_ST    = 360.0;

int ERIC_PART_SELECT = ELBOW;
double ELBOW_JOINT_RAD;
double WRIST_JOINT_RAD;
double HAND_ROTATE_JOINT_RAD;
double initX = L1+L2; //
double initY = 0;
double initT = 0;

double goalX = initX;
double goalY = initY;
double goalT = initT;

double lastX = goalX;
double lastY = goalY;
double lastT = goalT;

bool nanIK;
#define ServoInitACC_ST      100
#define ServoMaxSpeed_ST     4000
#define MaxSpeed_X_ST        4000
#define ServoInitSpeed_ST    2000
#define ServoMaxAcc 150

int Eric_Arm_ID[4]  = {ELBOW_ID,WRIST_SPAN_ID,HAND_ID,WRIST_UP_DOWN_ID};
char *Eric_Arm_Name[] ={"ELBOW","WRIST","HAND","WRIST_2"};
bool Eric_Arm_Status[]={NOT_FOUND,NOT_FOUND,NOT_FOUND,NOT_FOUND};

bool Eric_Arm_Torque[4]={TORQUE_ENABLE,TORQUE_ENABLE,TORQUE_ENABLE,TORQUE_ENABLE};
// feed data structure
struct Eric_Servo_FB {
  s16 load;
  s16 speed;
  byte voltage;
  int current;
  s16 pos;
  s16 mode;
  s16 temper;
} Eric_Arm_FB[DOF_NUM];
int64_t start_time;
// Bessel Curve Log structure
struct Bessel {
  double x;
  double y;
  double z;
  int64_t time;
} bessels[BESSEL_MAX_LOG_NUM];

bool searchCmd = false;

s16 Eric_Desire_Speed[4] = {200,200,200,200};
s16 Eric_Desire_Acc[4] = {100,100,100,100};
float Eric_Initial_Pos[4] = {2047.0,2047.0,2047.0,2047.0};
float Eric_AfterGrab_Pos[4] = {2047.0,2047.0,2047.0,2047.0};
float Eric_Land_Pos[4] = {2047,2047,2047,2047};
float Eric_Elbow_limit = 4095;
// the buffer of the bytes read from USB-C and servos. 
int usbRead;
int stsRead;

// set the max ID.
void getFeedBack();
void setMode(byte InputID, byte InputMode);
void servoStop(byte servoID);
void servoTorque(byte servoID, u8 enableCMD);


void simpleLinkageIkRad(double L1, double L2, double X, double Y);
float Rad_To_Pos(double rad);
void Rad_Ctrl_ELBOW(double radInput);
void Rad_Ctrl_WRIST(double radInput);
void Rad_Ctrl_HAND(double radInput);
void goalPosMove();
void RoArmM2_allPosAbsBesselCtrl(double inputX, double inputY, double inputT, double inputSpd);
void RoArmM2_movePosGoalfromLast(float spdInput);
double maxNumInArray();
double besselCtrl(double numStart, double numEnd, double rateInput);
void CoordinateCtrl(double inputX, double inputY, double inputT);
void lastPosUpdate();
int64_t get_Current_time();
void log_msg(int counter);

void getFeedBack(){

    for(int i=0;i<DOF_NUM;i++)
    {
      if(Eric_Arm_Status[i] == DETECTED)
      {
        //download all status to mem buffer
        if(st.FeedBack(Eric_Arm_ID[i])!=-1)
        {
          Eric_Arm_FB[i].pos = st.ReadPos(-1);
          Eric_Arm_FB[i].speed = st.ReadSpeed(-1);
          Eric_Arm_FB[i].load = st.ReadLoad(-1);
          Eric_Arm_FB[i].voltage = st.ReadVoltage(-1);
          Eric_Arm_FB[i].current = st.ReadCurrent(-1);
          Eric_Arm_FB[i].temper = st.ReadTemper(-1);
          Eric_Arm_FB[i].mode = st.ReadMode(Eric_Arm_ID[i]);
        }
      }
    }
}


void setMode(byte InputID, byte InputMode){
  if(InputMode == 0){
      st.unLockEprom(InputID);
      st.writeWord(InputID, 11, 4095);
      st.writeByte(InputID, SMS_STS_MODE, InputMode);
      st.LockEprom(InputID);
    
  }

  else if(InputMode == 3){
      st.unLockEprom(InputID);
      st.writeByte(InputID, SMS_STS_MODE, InputMode);
      st.writeWord(InputID, 11, 0);
      st.LockEprom(InputID);
  }
}



void servoStop(byte servoID){
    st.EnableTorque(servoID, 0);
    delay(10);
    st.EnableTorque(servoID, 1);
}


void servoTorque(byte servoID, u8 enableCMD){
    st.EnableTorque(servoID, enableCMD);
}

// inverse kinematics, Transfer (X,Y) to servo rads
// Simple Linkage IK:
// input the position of the end and return angle.
//   O----O
//  /
// O
// ---------------------------------------------------
// |       /beta           /delta                    |
//        O----L2---------X------                    |
// |     /       omega.   |       \L2                |
//      L1        .                < ----------------|
// |alpha     .          Y          |
//    /psi.                                 |
// | /.   Lambda          |                          |
// O- - - - - X - - - - X -                        |
// ---------------------------------------------------
// alpha, beta > 0 ; delta <= 0 ; X, Y > 0
void simpleLinkageIkRad(double L1, double L2, double X, double Y) {
  double psi, alpha, omega, beta, L2C, LC, Lambda, delta;

  if (fabs(Y) < 1e-6) {
    psi = acos((L1 * L1 + X * X - L2 * L2) / (2 * L1 * X));
    //printf("psi:%f,\n",psi);
    alpha = M_PI / 2.0 - psi;
    //printf("alpha:%f,\n",alpha);
    omega = acos((X * X + L2 * L2 - L1 * L1) / (2 * X * L2));
    //printf("omega:%f,\n",omega);
    beta = psi + omega;
  } else {
    L2C = X * X + Y * Y;
    LC = sqrt(L2C);
    Lambda = atan2(Y, X);
    //printf("lambda:%f,\n",Lambda);
    //printf("%f\n",(L1 * L1 + L2C - L2 * L2) / (2 * L1 * LC));
    psi = acos((L1 * L1 + L2C - L2 * L2) / (2 * L1 * LC));
    //printf("psi:%f,\n",psi);
    alpha = M_PI / 2.0 - Lambda - psi;
    omega = acos((L2 * L2 + L2C - L1 * L1) / (2 * LC * L2));
    //printf("omega:%f,\n",omega);
    beta = psi + omega;
  }

  delta = M_PI / 2.0 - alpha - beta;
  Serial.printf("alpha:%f,beta:%f\n",alpha,beta);
  ELBOW_JOINT_RAD = alpha;
  WRIST_JOINT_RAD = beta;
  HAND_ROTATE_JOINT_RAD = delta;
  nanIK = isnan(alpha) || isnan(beta) || isnan(delta);
}

//mapping between desired angle to position command

float Rad_To_Pos(double rad)
{
  return round((rad / (2 * M_PI)) * ARM_SERVO_POS_RANGE);
}

void Rad_Ctrl_ELBOW(double radInput)
{
  
  radInput = constrain(radInput, 0, M_PI/2);
  radInput = M_PI/2 - radInput;
  float pos_desired = Rad_To_Pos(radInput)+ServoDigitalMiddle_ST;
  goalpos[ELBOW] = pos_desired;
}

void Rad_Ctrl_WRIST(double radInput)
{
  // this - defines the correct motion direction
  radInput = -constrain(radInput, -M_PI/2, M_PI/2);

  float pos_desired = constrain(Rad_To_Pos(radInput)+ServoDigitalMiddle_ST,ARM_SERVO_POS_RANGE-Eric_Range_upper[WRIST_2],Eric_Range_upper[WRIST_2]);
  goalpos[WRIST_2] = pos_desired;
}

void Rad_Ctrl_HAND(double radInput)
{
  radInput = constrain(radInput, -M_PI, M_PI);
  float pos_desired = Rad_To_Pos(radInput)+ServoDigitalMiddle_ST;
  goalpos[WRIST] = pos_desired;
}

void goalPosMove()
{
  Rad_Ctrl_ELBOW(ELBOW_JOINT_RAD);
  Rad_Ctrl_WRIST(WRIST_JOINT_RAD);
  st.WritePosEx(Eric_Arm_ID[ELBOW], goalpos[ELBOW],Eric_Desire_Speed[ELBOW], Eric_Desire_Acc[ELBOW]);
  st.WritePosEx(Eric_Arm_ID[WRIST_2], goalpos[WRIST_2], Eric_Desire_Speed[WRIST_2], Eric_Desire_Acc[WRIST_2]);
  st.WritePosEx(Eric_Arm_ID[WRIST], goalpos[WRIST], Eric_Desire_Speed[WRIST], Eric_Desire_Acc[WRIST]);
}

// ctrl all axis abs position.
// initX = l3+l2B
// initY = 0
// initZ = l2A
// initT = M_PI
// default inputSpd = 0.36
// The meaning of parameter 'inputSpd'
void RoArmM2_allPosAbsBesselCtrl(double inputX, double inputY, double inputT, double inputSpd){
  goalX = inputX;
  goalY = inputY;
  goalT = inputT;
  start_time = get_Current_time();
  RoArmM2_movePosGoalfromLast(inputSpd);
  
}


// use this function to move the end of the arm to the goal position.
void RoArmM2_movePosGoalfromLast(float spdInput){
  double deltaSteps = maxNumInArray();
  //Counter the steps in bessel intepolate
  int counter =0 ;
  double bufferX;
  double bufferY;
  double bufferT;

  static double bufferLastX;
  static double bufferLastY;
  static double bufferLastT;

  for(double i=0;i<=1;i+=(1/(deltaSteps*1))*spdInput){
    
    
    bufferX = besselCtrl(lastX, goalX, i);
    bufferY = besselCtrl(lastY, goalY, i);
    //bufferZ = besselCtrl(lastZ, goalZ, i);
    bufferT = besselCtrl(lastT, goalT, i);
    CoordinateCtrl(bufferX,bufferY,bufferT);
    //RoArmM2_baseCoordinateCtrl(bufferX, bufferY, bufferZ, bufferT);
    if(nanIK){
      // IK failed
      goalX = bufferLastX;
      goalY = bufferLastY;
      //goalZ = bufferLastZ;
      goalT = bufferLastT;
      CoordinateCtrl(goalX,goalY,goalT);
      //RoArmM2_baseCoordinateCtrl(goalX, goalY, goalZ, goalT);
      //RoArmM2_goalPosMove();
      goalPosMove();
      lastPosUpdate();
      return;
    }
    else{
      // IK succeed.
      bufferLastX = bufferX;
      bufferLastY = bufferY;
      bufferLastT = bufferT;
    }
    bessels[counter].x = bufferLastX;
    bessels[counter].y = bufferLastY;
    bessels[counter].z = 0;
    bessels[counter].time = get_Current_time();

    goalPosMove();
    delay(2);
    counter+=1;
  }
  CoordinateCtrl(goalX, goalY, goalT);
  bessels[counter].x=goalX;
  bessels[counter].y=goalY;
  bessels[counter].z=0;
  bessels[counter].time = get_Current_time();
  goalPosMove();
  lastPosUpdate();
  log_msg(counter);
}


// use this function to get the max deltaSteps.
// get the max offset between [goal] and [last] position.
double maxNumInArray(){
    double deltaPos[3] = {abs(goalX - lastX),
                          abs(goalY - lastY),
                          abs(goalT - lastT)*10};
    double maxVal = deltaPos[0];
    for(int i = 0; i < (sizeof(deltaPos) / sizeof(deltaPos[0])); i++){
      maxVal = max(deltaPos[i],maxVal);
    }
    return maxVal;
}

// ctrl the movement in a smooth way.
// |                 ..  <-numEnd
// |             .    |
// |           .    
// |         .        |
// |        .
// |      .           |
// |. . <-numStart
// ----------------------
// 0                  1 rateInput
double besselCtrl(double numStart, double numEnd, double rateInput){
  double numOut;
  numOut = (numEnd - numStart)*((cos(rateInput*M_PI+M_PI)+1)/2) + numStart;
  return numOut;
}

void CoordinateCtrl(double inputX, double inputY, double inputT)
{
  simpleLinkageIkRad(L1, L2, inputX, inputY);
  Rad_Ctrl_HAND(inputT);
  //RoArmM2_handJointCtrlRad(0, inputT, 0, 0);
}

void lastPosUpdate()
{
  lastX = goalX;
  lastY = goalY;
  lastT = goalT;
}

int64_t get_Current_time()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
  return time_us;
}

void log_msg(int counter)
{
  Serial.printf("Start time: %lld",start_time);
  //not for log yet just print out to uart
  for(int i=0;i<=counter;i++)
  {
    Serial.printf("X:%f,Y:%f,T:%lld\n",bessels[i].x,bessels[i].y,bessels[i].time);
  }
}
