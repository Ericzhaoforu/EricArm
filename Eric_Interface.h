#include <SPIFFS.h>
#include <ArduinoNvs.h>
#include "Eric_Servo.h"
#define GENERAL_ROBOT_BOARD 1
#define DRIVER_BOARD 2
#define BOARD_TYPE DRIVER_BOARD

#define CONSOLE_BAUD_RATE 115200
#define SERVO_BAUD_RATE 1000000
// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19
#define OK 1
#define FAIL 0
// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#if BOARD_TYPE==GENERAL_ROBOT_BOARD    
    #define S_SCL 33
    #define S_SDA 32
#elif BOARD_TYPE==DRIVER_BOARD 
    #define S_SCL 22
    #define S_SDA 21
#endif

//ESP-NOW param
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif


#include "Eric_Communicate.h"
#include "Eric_Oled.h"


/*
initialize the Serial, OLED, WIFI, etc;
*/
void Interface_Setup();

/*
Serial initialize
*/
bool Serial_Init();

/*
OLED display Initialize
*/
void OLED_Init();

/*
WIFI Initialize
*/
void WIFI_Init();

/*
WebServer Initialize
*/
void WEB_Init();

/*
Servo Initialize
*/
void SERVO_Init();

/*
Thread Initialize
*/
void Thread_Init();

/*
SPIFFS initialize
*/
void SPIFFS_Init();

/*
NVS initialize
*/
void NVS_Init();


/*
initialize the Serial, OLED, WIFI, etc;
*/
void Interface_Setup()
{
    while (Serial_Init()!=OK){
         Serial.println("Wait for Serial.");
         delay(50);
    }
    Serial.println("Serial OK.");

    NVS_Init();

    SPIFFS_Init();
    
    getMAC();

    InitRGB();

    OLED_Init();
    Serial.println(F("OLED OK."));

    WIFI_Init();
    Serial.println(F("WIFI OK."));
    
    WEB_Init();
    Serial.println(F("WEB OK."));

    SERVO_Init();
    
    Eric_Servo_Check(true);

    Thread_Init();
    Serial.println(F("THREAD OK."));
    matrix.setPixelColor(1,0,255,0);
    Move_To_Land_Pos_Elbow_Wrist();
    //Move_To_Desired_Pos();
}

/*
Serial initialize
*/
bool Serial_Init()
{
    //Initialize Console uart & Servo uart
    Serial.begin(CONSOLE_BAUD_RATE);
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    if(Serial && Serial1)
    {
        return OK;
    }
    return FAIL;
}

/*
OLED display Initialize
*/
void OLED_Init()
{
    Wire.begin(S_SDA, S_SCL);
    InitScreen();
}


void WIFI_Init()
{
    WIFI_MODE = DEFAULT_WIFI_MODE;
    if(WIFI_MODE == 1){setAP();}
    else if(WIFI_MODE == 2){setSTA();}
}

void WEB_Init()
{
    webCtrlServer();
}

void SERVO_Init()
{
    st.pSerial = &Serial1;
    for(int i=0;i<=DOF_NUM;i++)
    {
        servoTorque(Eric_Arm_ID[i],TORQUE_ENABLE);
    }
}


void Thread_Init()
{
    xTaskCreatePinnedToCore(&InfoUpdateThreading, "InfoUpdate", 4000, NULL, 5, &ScreenUpdateHandle, ARDUINO_RUNNING_CORE);
    xTaskCreate(&clientThreading, "Client", 4000, NULL, 5, &ClientCmdHandle);
}


void SPIFFS_Init()
{
    if(!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    else
    {
        Serial.println("SPIFFS file system OK.");
    }
}

void NVS_Init()
{
    if (NVS.begin()==1)
    {
        Serial.println("NVS OK.");
    }
    else
    {
        Serial.println("NVS Err.");
    }
    //To write eprom, uncommand this
    //NVS.setInt("isExist",-1);
    //Check if terms is Exist
    if(NVS.getInt("isExist", -1)==-1)
    {
        //Not exist, set default values
        NVS.setInt("Speed0",Eric_Desire_Speed[0]);
        NVS.setInt("Speed1",Eric_Desire_Speed[1]);
        NVS.setInt("Speed2",Eric_Desire_Speed[2]);
        NVS.setInt("Speed3",Eric_Desire_Speed[3]);

        NVS.setInt("Acc0",Eric_Desire_Acc[0]);
        NVS.setInt("Acc1",Eric_Desire_Acc[1]);
        NVS.setInt("Acc2",Eric_Desire_Acc[2]);
        NVS.setInt("Acc3",Eric_Desire_Acc[3]);

        NVS.setFloat("Pos0",Eric_Initial_Pos[0]);
        NVS.setFloat("Pos1",Eric_Initial_Pos[1]);
        NVS.setFloat("Pos2",Eric_Initial_Pos[2]);
        NVS.setFloat("Pos3",Eric_Initial_Pos[3]);
        
        NVS.setFloat("PosG0",Eric_AfterGrab_Pos[0]);
        NVS.setFloat("PosG1",Eric_AfterGrab_Pos[1]);
        NVS.setFloat("PosG2",Eric_AfterGrab_Pos[2]);
        NVS.setFloat("PosG3",Eric_AfterGrab_Pos[3]);

        NVS.setFloat("PosL0",Eric_Land_Pos[0]);
        NVS.setFloat("PosL1",Eric_Land_Pos[1]);
        NVS.setFloat("PosL2",Eric_Land_Pos[2]);
        NVS.setFloat("PosL3",Eric_Land_Pos[3]);

        NVS.setInt("isExist",1);
        Serial.println("Set default Speed,Acc,Pos values to NVS");
    }
    //load data
    else{
        Serial.println("Acc,Speed, Pos already Saved, loading form NVS");
        Eric_Desire_Speed[0] = s16(NVS.getInt("Speed0"));
        Eric_Desire_Speed[1] = s16(NVS.getInt("Speed1"));
        Eric_Desire_Speed[2] = s16(NVS.getInt("Speed2"));
        Eric_Desire_Speed[3] = s16(NVS.getInt("Speed3"));

        Serial.println("Speed");
        Serial.println(Eric_Desire_Speed[0]);
        Serial.println(Eric_Desire_Speed[1]);
        Serial.println(Eric_Desire_Speed[2]);
        Serial.println(Eric_Desire_Speed[3]);

        Eric_Desire_Acc[0] = s16(NVS.getInt("Acc0"));
        Eric_Desire_Acc[1] = s16(NVS.getInt("Acc1"));
        Eric_Desire_Acc[2] = s16(NVS.getInt("Acc2"));
        Eric_Desire_Acc[3] = s16(NVS.getInt("Acc3"));

        Serial.println("acc");
        Serial.println(Eric_Desire_Acc[0]);
        Serial.println(Eric_Desire_Acc[1]);
        Serial.println(Eric_Desire_Acc[2]);
        Serial.println(Eric_Desire_Acc[3]);

        Eric_Initial_Pos[0] = NVS.getFloat("Pos0");
        Eric_Initial_Pos[1] = NVS.getFloat("Pos1");
        Eric_Initial_Pos[2] = NVS.getFloat("Pos2");
        Eric_Initial_Pos[3] = NVS.getFloat("Pos3");

        Serial.println("Pos");
        Serial.println(Eric_Initial_Pos[0]);
        Serial.println(Eric_Initial_Pos[1]);
        Serial.println(Eric_Initial_Pos[2]);
        Serial.println(Eric_Initial_Pos[3]);

        Eric_AfterGrab_Pos[0] = NVS.getFloat("PosG0");
        Eric_AfterGrab_Pos[1] = NVS.getFloat("PosG1");
        Eric_AfterGrab_Pos[2] = NVS.getFloat("PosG2");
        Eric_AfterGrab_Pos[3] = NVS.getFloat("PosG3");
        Serial.println("PosG");
        Serial.println(Eric_AfterGrab_Pos[0]);
        Serial.println(Eric_AfterGrab_Pos[1]);
        Serial.println(Eric_AfterGrab_Pos[2]);
        Serial.println(Eric_AfterGrab_Pos[3]);

        Eric_Land_Pos[0] = NVS.getFloat("PosL0");
        Eric_Land_Pos[1] = NVS.getFloat("PosL1");
        Eric_Land_Pos[2] = NVS.getFloat("PosL2");
        Eric_Land_Pos[3] = NVS.getFloat("PosL3");
        Serial.println("PosL");
        Serial.println(Eric_Land_Pos[0]);
        Serial.println(Eric_Land_Pos[1]);
        Serial.println(Eric_Land_Pos[2]);
        Serial.println(Eric_Land_Pos[3]);

    }

    Serial.println("NVS loaded.");

}


