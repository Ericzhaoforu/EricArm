#include "SCServo.h"
#define ELBOW_ID 11
#define WRIST_UP_DOWN_ID 13
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
// === ST Servo
SMS_STS st;
float ServoDigitalRange_ST  = 4095.0;
float ServoAngleRange_ST    = 360.0;
float ServoDigitalMiddle_ST = 2047.0;
int ERIC_PART_SELECT = ELBOW;

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


bool searchCmd = false;

s16 Eric_Desire_Speed[4] = {200,200,200,200};
s16 Eric_Desire_Acc[4] = {100,100,100,100};
float Eric_Initial_Pos[4] = {2047.0,2047.0,2047.0,2047.0};


// the buffer of the bytes read from USB-C and servos. 
int usbRead;
int stsRead;

// set the max ID.
void getFeedBack();
void setMode(byte InputID, byte InputMode);
void servoStop(byte servoID);
void servoTorque(byte servoID, u8 enableCMD);

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