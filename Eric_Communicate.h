// https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/
#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>

// set the default role here.
// 0 as normal mode.
// 1 as leader, ctrl other device via ESP-NOW.
// 2 as follower, can be controled via ESP-NOW.
#define DEFAULT_ROLE 0


// set the default wifi mode here.
// 1 as [AP] mode, it will not connect other wifi.
// 2 as [STA] mode, it will connect to know wifi.
#define DEFAULT_WIFI_MODE 1

#define clientInterval    10


// the MAC address of the device you want to ctrl.
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0x93, 0x5F, 0xA8};
TaskHandle_t ClientCmdHandle;
//WIFI information
String MAC_ADDRESS;    //MAC_Address
IPAddress IP_ADDRESS; //IP_ADDRESS
byte   WIFI_MODE;    //Mode of WIFI

int WIFI_RSSI;

// WIFI_AP settings.
const char* AP_SSID = "Eric_Arm";
const char* AP_PWD  = "12345678";

// WIFI_STA settings.
const char* STA_SSID = "OnePlus 8";
const char* STA_PWD  = "40963840";


// Create AsyncWebServer object on port 80
WebServer server(80);

void setAP();
void setSTA();
void getWifiStatus();
void webCtrlServer();
void getIP();
void getMAC();

void handle_ELBOW();
void handle_WRIST();
void handle_HAND();
void handle_WRIST_2();
void handle_PART();
void handle_defaultPos();
void handle_TotalC();

void Individual_ctrl(int ctrltype,int ctrlpart);
void Whole_Ctrl(int ctrltype);

void ID_Rotate(int cmdInput);

void SaveParam_Acc_Speed();
void SaveParam_Pos();
void SaveGrabPos();
void SaveLandPos();
void Move_To_Desired_Pos();
void Move_To_Land_Pos_Elbow_Wrist();
void Move_To_Land_Pos_Wrist_Elbow();
void Move_To_Grab_Pos();
void SetMid(int Index);

void clientThreading(void *pvParameter);
void UART_Ctrl();
void Process_Buffer(char Buffer[],int length);

void setAP(){
  WiFi.softAP(AP_SSID, AP_PWD);
  IPAddress myIP = WiFi.softAPIP();
  IP_ADDRESS = myIP;
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  WIFI_MODE = 1;
}

void setSTA(){
  WIFI_MODE = 3;
  WiFi.begin(STA_SSID, STA_PWD);
}

void getWifiStatus(){
  if(WiFi.status() == WL_CONNECTED){
    WIFI_MODE = 2;
    getIP();
    WIFI_RSSI = WiFi.RSSI();
  }
  else if(WiFi.status() == WL_CONNECTION_LOST && DEFAULT_WIFI_MODE == 2){
    WIFI_MODE = 3;
    // WiFi.disconnect();
    WiFi.reconnect();
  }
}

void webCtrlServer(){
    server.on("/", []()
    {
      File file = SPIFFS.open("/web.html", FILE_READ);
          String html="";
          if(file)
          {
            html = file.readString();
          }
          server.send(200, "text/html", html);
          file.close();
    });
    server.on("/Read_ELBOW",handle_ELBOW);
    server.on("/Read_WRIST",handle_WRIST);
    server.on("/Read_HAND",handle_HAND);
    server.on("/Read_WRIST_2", handle_WRIST_2);
    server.on("/PART",handle_PART);
    server.on("/DefaultPos",handle_defaultPos);
    server.on("/TotalC",handle_TotalC);
    server.on("/grid", [](){
          File file = SPIFFS.open("/grid.png", FILE_READ);
          String img="";
          if(file)
          {
            img = file.readString();
          }
          server.send(200,"image/png",img);
          file.close();
      });
    server.on("/crosshair_ball3", [](){
          File file = SPIFFS.open("/crosshair_ball3.png", FILE_READ);
          String img="";
          if(file)
          {
            img = file.readString();
          }
          server.send(200,"image/png",img);
          file.close();
      });
    server.on("/trigger_meter_fill", [](){
          File file = SPIFFS.open("/trigger_meter_fill.png", FILE_READ);
          String img="";
          if(file)
          {
            img = file.readString();
          }
          server.send(200,"image/png",img);
          file.close();
      });
    server.on("/trigger_meter", [](){
          File file = SPIFFS.open("/trigger_meter.png", FILE_READ);
          String img="";
          if(file)
          {
            img = file.readString();
          }
          server.send(200,"image/png",img);
          file.close();
      });
    server.on("/dpad", [](){
          File file = SPIFFS.open("/dpad.png", FILE_READ);
          String img="";
          if(file)
          {
            img = file.readString();
          }
          server.send(200,"image/png",img);
          file.close();
      });
      server.on("/favicon.ico", [](){
          File file = SPIFFS.open("/favicon.png", FILE_READ);
          String img="";
          if(file)
          {
            img = file.readString();
          }
          server.send(200,"image/png",img);
          file.close();
      });
    server.on("/cmd", [](){
    
    double X = server.arg(0).toDouble();
    double Y = server.arg(1).toDouble(); 
    double T = server.arg(2).toDouble();
    double input_spd = server.arg(3).toDouble();
    //coordinate input
    if(X>=10)
    {
      if(X>0&&Y>0&&input_spd>0)
      {
        // Serial.printf("transmission received %lf\n",get_Current_time());
        RoArmM2_allPosAbsBesselCtrl(X,Y,T,input_spd);
      }
    }
    else
    {
    int cmdT = server.arg(0).toInt();
    int cmdI = server.arg(1).toInt();
    int cmdA = server.arg(2).toInt();
    int cmdB = server.arg(3).toInt();

    switch(cmdT){
        //case 1 for individual control
          case 1:Individual_ctrl(cmdI,cmdA);break;
        //case 2 for whole control
          case 2:Whole_Ctrl(cmdI);break;
          case 6: Move_To_Land_Pos_Wrist_Elbow();break;
          case 7: Move_To_Land_Pos_Elbow_Wrist();break;
          case 8: 
                  //CoordinateCtrl(L1,L2,0); 
                  //goalPosMove();
                  //lastX = L1;
                  //lastY=  L2;

          //Move_To_Grab_Pos();
                  break;
          case 9: //Move_To_Desired_Pos();
          break;
        }
    }
  });

  // Start server
  server.begin();
  Serial.println("Server Starts.");
}

void handle_defaultPos(){
  String cont ="";
  //cont+="E:"+String(Eric_Initial_Pos[0])+" W:"+Eric_Initial_Pos[1]+" H:"+Eric_Initial_Pos[2]+" W2:"+Eric_Initial_Pos[3];
  cont+="X:"+String(showX)+" Y:"+(showY);
  server.send(200, "text/plane", cont);
}
void handle_TotalC(){

  
  float TC=0.0;
  for(int i=0;i<DOF_NUM;i++)
  {
    if(Eric_Arm_Status[i]==DETECTED)
    {
      TC+=Eric_Arm_FB[i].current*0.0065;
    }
  }
  //Serial.println(TC,4);
  server.send(200, "text/plane", String(TC,4));
}
/*
show specific servo status
*/
void handle_ELBOW() {
  String stsValue = "";
  //detected
  if(Eric_Arm_Status[ELBOW]==DETECTED){
    stsValue += "<p><span class=\"tag\">Cur:</span><span class=\"value\">"+String(Eric_Arm_FB[ELBOW].current*0.0065,2)+"</span>";
    stsValue += "  <span class=\"tag\">Pos:</span><span class=\"value\">"+String(Eric_Arm_FB[ELBOW].pos)+"</span></p>";
    stsValue += "<p><span class=\"tag\">Vol:</span><span class=\"value\">"+String(float(Eric_Arm_FB[ELBOW].voltage)/10)+"</span>";
    stsValue += "  <span class=\"tag\">Load:</span><span class=\"value\">"+String(Eric_Arm_FB[ELBOW].load)+"</span></p>";
    stsValue += "<p><span class=\"tag\">Speed:</span><span class=\"value\">"+String(Eric_Arm_FB[ELBOW].speed)+"</span>";
    stsValue += "  <span class=\"tag\">Temper:</span><span class=\"value\">"+String(Eric_Arm_FB[ELBOW].temper)+"</span></p>";
    
    stsValue += "<p><span class=\"tag\">Mode:</span>";
    if(Eric_Arm_FB[ELBOW].mode == 0){
      stsValue += "<span class=\"value\">  Servo(M0)</span>";
    }
    else if(Eric_Arm_FB[ELBOW].mode == 3){
      stsValue += "<span class=\"value\">  Motor(M3)</span>";
    }
    if(Eric_Arm_Torque[ELBOW]){
      stsValue += "  <span class=\"tag\">Torque:</span><span class=\"value\"> On</span>";
    }
    else{
      stsValue += "  <span class=\"tag\">Torque:</span><span class=\"value\"> Off</span>";
    }
    stsValue+="</p>";
    stsValue+="<p><span class=\"tag\">Des speed: </span><span class=\"value\">"+String(Eric_Desire_Speed[ELBOW])+"</span></p>";
    stsValue+="<p><span class=\"tag\">Des acc: </span><span class=\"value\">"+String(Eric_Desire_Acc[ELBOW])+"</span></p>";
  }
  else{
    stsValue += "<p><span class=\"tag\">FeedBack err</span></p>";
  }
  server.send(200, "text/plane", stsValue); //Send ADC value only to client ajax request
}

void handle_WRIST() {
  String stsValue = "";
  //detected
  if(Eric_Arm_Status[WRIST]==DETECTED){
    stsValue += "<p><span class=\"tag\">Cur:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST].current*0.0065,2)+"</span>";
    stsValue += "  <span class=\"tag\">Pos:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST].pos)+"</span></p>";
    stsValue += "<p><span class=\"tag\">Vol:</span><span class=\"value\">"+String(float(Eric_Arm_FB[WRIST].voltage)/10)+"</span>";
    stsValue += "  <span class=\"tag\">Load:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST].load)+"</span></p>";
    stsValue += "<p><span class=\"tag\">Speed:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST].speed)+"</span>";
    stsValue += "  <span class=\"tag\">Temper:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST].temper)+"</span></p>";
    
    stsValue += "<p><span class=\"tag\">Mode:</span>";
    if(Eric_Arm_FB[WRIST].mode == 0){
      stsValue += "<span class=\"value\">  Servo(M0)</span>";
    }
    else if(Eric_Arm_FB[WRIST].mode == 3){
      stsValue += "<span class=\"value\">  Motor(M3)</span>";
    }
    if(Eric_Arm_Torque[WRIST]){
      stsValue += "  <span class=\"tag\">Torque:</span><span class=\"value\"> On</span>";
    }
    else{
      stsValue += "  <span class=\"tag\">Torque:</span><span class=\"value\"> Off</span>";
    }
    stsValue+="</p>";
    stsValue+="<p><span class=\"tag\">Des speed: </span><span class=\"value\">"+String(Eric_Desire_Speed[WRIST])+"</span></p>";
    stsValue+="<p><span class=\"tag\">Des acc: </span><span class=\"value\">"+String(Eric_Desire_Acc[WRIST])+"</span></p>";
  }
  else{
    stsValue += "<p><span class=\"tag\">FeedBack err</span></p>";
  }
  server.send(200, "text/plane", stsValue); //Send ADC value only to client ajax request
}

void handle_HAND() {
  String stsValue = "";
  //detected
  if(Eric_Arm_Status[HAND]==DETECTED){
    stsValue += "<p><span class=\"tag\">Cur:</span><span class=\"value\">"+String(Eric_Arm_FB[HAND].current*0.0065,2)+"</span>";
    stsValue += "  <span class=\"tag\">Pos:</span><span class=\"value\">"+String(Eric_Arm_FB[HAND].pos)+"</span></p>";
    stsValue += "<p><span class=\"tag\">Vol:</span><span class=\"value\">"+String(float(Eric_Arm_FB[HAND].voltage)/10)+"</span>";
    stsValue += "  <span class=\"tag\">Load:</span><span class=\"value\">"+String(Eric_Arm_FB[HAND].load)+"</span></p>";
    stsValue += "<p><span class=\"tag\">Speed:</span><span class=\"value\">"+String(Eric_Arm_FB[HAND].speed)+"</span>";
    stsValue += "  <span class=\"tag\">Temper:</span><span class=\"value\">"+String(Eric_Arm_FB[HAND].temper)+"</span></p>";
    
    stsValue += "<p><span class=\"tag\">Mode:</span>";
    if(Eric_Arm_FB[HAND].mode == 0){
      stsValue += "<span class=\"value\">  Servo(M0)</span>";
    }
    else if(Eric_Arm_FB[HAND].mode == 3){
      stsValue += "<span class=\"value\">  Motor(M3)</span>";
    }
    if(Eric_Arm_Torque[HAND]){
      stsValue += "  <span class=\"tag\">Torque:</span><span class=\"value\"> On</span>";
    }
    else{
      stsValue += "  <span class=\"tag\">Torque:</span><span class=\"value\"> Off</span>";
    }
    stsValue+="</p>";
    stsValue+="<p><span class=\"tag\">Des speed: </span><span class=\"value\">"+String(Eric_Desire_Speed[HAND])+"</span></p>";
    stsValue+="<p><span class=\"tag\">Des acc: </span><span class=\"value\">"+String(Eric_Desire_Acc[HAND])+"</span></p>";
  }
  else{
    stsValue += "<p><span class=\"tag\">FeedBack err</span></p>";
  }
  server.send(200, "text/plane", stsValue); //Send ADC value only to client ajax request
}

void handle_WRIST_2() {
  String stsValue = "";
  //detected
  if(Eric_Arm_Status[WRIST_2]==DETECTED){
    stsValue += "<p><span class=\"tag\">Cur:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST_2].current*0.0065,2)+"</span>";
    stsValue += "  <span class=\"tag\">Pos:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST_2].pos)+"</span></p>";
    stsValue += "<p><span class=\"tag\">Vol:</span><span class=\"value\">"+String(float(Eric_Arm_FB[WRIST_2].voltage)/10)+"</span>";
    stsValue += "  <span class=\"tag\">Load:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST_2].load)+"</span></p>";
    stsValue += "<p><span class=\"tag\">Speed:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST_2].speed)+"</span>";
    stsValue += "  <span class=\"tag\">Temper:</span><span class=\"value\">"+String(Eric_Arm_FB[WRIST_2].temper)+"</span></p>";

    stsValue += "<p><span class=\"tag\">Mode:</span>";
    if(Eric_Arm_FB[WRIST_2].mode == 0){
      stsValue += "<span class=\"value\">  Servo(M0)</span>";
    }
    else if(Eric_Arm_FB[WRIST_2].mode == 3){
      stsValue += "<span class=\"value\">  Motor(M3)</span>";
    }
    if(Eric_Arm_Torque[WRIST_2]){
      stsValue += "  <span class=\"tag\">Torque:</span><span class=\"value\"> On</span>";
    }
    else{
      stsValue += "  <span class=\"tag\">Torque:</span><span class=\"value\"> Off</span>";
    }
    stsValue+="</p>";
    stsValue+="<p><span class=\"tag\">Des speed: </span><span class=\"value\">"+String(Eric_Desire_Speed[WRIST_2])+"</span></p>";
    stsValue+="<p><span class=\"tag\">Des acc: </span><span class=\"value\">"+String(Eric_Desire_Acc[WRIST_2])+"</span></p>";
  }
  else{
    stsValue += "<p><span class=\"tag\">FeedBack err</span></p>";
  }
  server.send(200, "text/plane", stsValue); //Send ADC value only to client ajax request
}

void handle_PART()
{
    String stsValue = Eric_Arm_Name[ERIC_PART_SELECT];
    server.send(200, "text/plane", stsValue);
}

void ID_Rotate(int cmdInput){
  ERIC_PART_SELECT += cmdInput;
  if(ERIC_PART_SELECT >= DOF_NUM){ 
    ERIC_PART_SELECT = 0;
  }
  else if(ERIC_PART_SELECT < 0){
    ERIC_PART_SELECT = DOF_NUM-1;
  }
}

void SetMid(int Index)
{
  st.CalibrationOfs(Eric_Arm_ID[Index]);
}

void Whole_Ctrl(int ctrltype)
{
  switch (ctrltype)
  {
    //(2,1,0,0) to torque on 
    case 1:
      for(int i=0;i<DOF_NUM;i++)
      {
        if(Eric_Arm_Status[i]==DETECTED)
        {
          servoTorque(Eric_Arm_ID[i],TORQUE_ENABLE);
          Eric_Arm_Torque[i]=TORQUE_ENABLE;
        }
      }
      break;
    //(2,2,0,0) to torque off
    case 2:
      for(int i=0;i<DOF_NUM;i++)
      {
        if(Eric_Arm_Status[i]==DETECTED)
        {
          servoTorque(Eric_Arm_ID[i],TORQUE_DISABLE);
          Eric_Arm_Torque[i]=TORQUE_DISABLE;
        }
      }
      break;
    //(2,3,0,0) Display ID+
    case 3:
      ID_Rotate(1);
      break;
    //(2,4,0,0) Display ID-
    case 4:
      ID_Rotate(-1);
      break;
    //(2,5,0,0) Search for servo
    case 5:
      searchCmd = true;
      break;
    //(2,6,0,0) Save param
    case 6:
      SaveParam_Acc_Speed();
      break;
    //(2,7,0,0) Save default position
    case 7:
      SaveParam_Pos();
      break;
    //(2,8,0,0) Set Cur position As Mid
    case 8:
      SetMid(ERIC_PART_SELECT);
      break;
    case 9:
      SaveGrabPos();
      break;
    case 10:
      SaveLandPos();
  }
}

void Individual_ctrl(int ctrltype,int ctrlpart)
{
  switch(ctrltype)
  {
    //(1,2,x,0) to stop the servo
    case 2:
      servoStop(Eric_Arm_ID[ctrlpart]);
      break;
    //(1,5,x,0) pos+
    case 5:
      //Servo Mode
      if(Eric_Arm_FB[ctrlpart].mode==0)
      {
        //pos limited
        //caution limit wrist up down when elbow is high
        float range_upper = Eric_Range_upper[ctrlpart] -1;
        if(ctrlpart ==3)
        {
          //if elbow high
          Serial.printf("Controlling wrist 2\n");
          if(Eric_Arm_FB[0].pos>2700.0)
          {
            Serial.printf("change upper\n");
            range_upper =2048.0;
          }
        }
        st.WritePosEx(Eric_Arm_ID[ctrlpart], range_upper, Eric_Desire_Speed[ctrlpart], Eric_Desire_Acc[ctrlpart]);
        
      }
      //Motor Mode
      if(Eric_Arm_FB[ctrlpart].mode==3)
      {
        st.WritePosEx(Eric_Arm_ID[ctrlpart], 30000, Eric_Desire_Speed[ctrlpart], Eric_Desire_Acc[ctrlpart]);
      }
      break;
    //(1,6,x,0) pos-
    case 6:
      //Servo Mode
      if(Eric_Arm_FB[ctrlpart].mode==0)
      {
        float lower=0.0;
        //pos limited
        if(ctrlpart==0)
        {
            lower=2048-1;
        }
        if(ctrlpart==3)
        {
          lower = 4096 - Eric_Range_upper[ctrlpart] -1;
        }
        st.WritePosEx(Eric_Arm_ID[ctrlpart], lower, Eric_Desire_Speed[ctrlpart], Eric_Desire_Acc[ctrlpart]);
      }
      //Motor Mode
      if(Eric_Arm_FB[ctrlpart].mode==3)
      {
        st.WritePosEx(Eric_Arm_ID[ctrlpart], -30000, Eric_Desire_Speed[ctrlpart], Eric_Desire_Acc[ctrlpart]);
      }
      break;
    //(1,7,x,0) desire speed +
    case 7:
      Eric_Desire_Speed[ctrlpart] += 10;
      //if larger than max speed 4000,then set to 4000
      if (Eric_Desire_Speed[ctrlpart]>ServoMaxSpeed_ST)
      {
        Eric_Desire_Speed[ctrlpart]=ServoMaxSpeed_ST;
      }
      break;
    //(1,8,x,0) desired speed - 
    case 8:
      Eric_Desire_Speed[ctrlpart]+=-10;
      //if smaller than 0, set to 0
      if(Eric_Desire_Speed[ctrlpart]<0)
      {
        Eric_Desire_Speed[ctrlpart]=0;
      }
      break;
    //(1,9,x,0) desire acc +
    case 9:
      Eric_Desire_Acc[ctrlpart]+=5;
      //if larger than maximum, set to maximum
      if (Eric_Desire_Acc[ctrlpart]>=ServoMaxAcc)
      {
        Eric_Desire_Acc[ctrlpart]=ServoMaxAcc;
      }
      break;
    //(1,10,x,0) desire acc -
    case 10:
      Eric_Desire_Acc[ctrlpart]+=-5;
      //if larger than maximum, set to maximum
      if (Eric_Desire_Acc[ctrlpart]<=0)
      {
        Eric_Desire_Acc[ctrlpart]=0;
      }
      break;
    //(1,12,x,0) Set Servo mode
    case 12:
      setMode(Eric_Arm_ID[ctrlpart],SERVO_MODE);
      break;
    //(1,13,x,0) Set Motor Mode
    case 13:
      setMode(Eric_Arm_ID[ctrlpart],MOTOR_MODE);
      break;
  }
}


void clientThreading(void *pvParameter){
  while(1){
    server.handleClient();
    UART_Ctrl();
    delay(clientInterval);
  }
}

void UART_Ctrl(){
  
    int length=Serial.available();
    char Rec_Buffer[10];
    if(Serial.available()<=10 &&length>0)
    {
      Serial.readBytes(Rec_Buffer,length);
      for (int i=0;i<length;i++){
      Serial.print(Rec_Buffer[i]);Serial.print(" ");
      Process_Buffer(Rec_Buffer,length);
    } 
    }
     
}
void Process_Buffer(char Buffer[],int length)
{
  if(Buffer[0]=='9'&&Buffer[1]=='0')
  {
      Move_To_Desired_Pos();
  }
}
void getIP(){
  IP_ADDRESS = WiFi.localIP();
}

void getMAC(){
  WiFi.mode(WIFI_AP_STA);
  MAC_ADDRESS = WiFi.macAddress();
  Serial.print("MAC:");
  Serial.println(WiFi.macAddress());
}

void SaveParam_Acc_Speed(){
  //Not exist, set default values
        NVS.setInt("Speed0",Eric_Desire_Speed[0]);
        NVS.setInt("Speed1",Eric_Desire_Speed[1]);
        NVS.setInt("Speed2",Eric_Desire_Speed[2]);
        NVS.setInt("Speed3",Eric_Desire_Speed[3]);

        NVS.setInt("Acc0",Eric_Desire_Acc[0]);
        NVS.setInt("Acc1",Eric_Desire_Acc[1]);
        NVS.setInt("Acc2",Eric_Desire_Acc[2]);
        NVS.setInt("Acc3",Eric_Desire_Acc[3]);
  Serial.println("Save Acc Speed param from Web.");
}

void SaveParam_Pos()
{
  //Read Cur Feed back pos
  for(int i=0;i<DOF_NUM;i++)
    {
      if(Eric_Arm_Status[i] == DETECTED)
      {
          Eric_Initial_Pos[i]= float(Eric_Arm_FB[i].pos);
      }
    }
    //Save
    NVS.setFloat("Pos0",Eric_Initial_Pos[0]);
    NVS.setFloat("Pos1",Eric_Initial_Pos[1]);
    NVS.setFloat("Pos2",Eric_Initial_Pos[2]);
    NVS.setFloat("Pos3",Eric_Initial_Pos[3]);
    Serial.println("Save Acc Speed param from Web.");
}

void SaveGrabPos()
{
  for(int i=0;i<DOF_NUM;i++)
    {
      if(Eric_Arm_Status[i] == DETECTED)
      {   
          Eric_AfterGrab_Pos[i] = float(Eric_Arm_FB[i].pos);
      }
    }
    //Save
    NVS.setFloat("PosG0",Eric_AfterGrab_Pos[0]);
    NVS.setFloat("PosG1",Eric_AfterGrab_Pos[1]);
    NVS.setFloat("PosG2",Eric_AfterGrab_Pos[2]);
    NVS.setFloat("PosG3",Eric_AfterGrab_Pos[3]);
    //Serial.println("Save Acc Speed param from Web.");
}

void SaveLandPos()
{
  for(int i=0;i<DOF_NUM;i++)
    {
      if(Eric_Arm_Status[i] == DETECTED)
      {   
          Eric_Land_Pos[i] = float(Eric_Arm_FB[i].pos);
      }
    }
    //Save
    NVS.setFloat("PosL0",Eric_Land_Pos[0]);
    NVS.setFloat("PosL1",Eric_Land_Pos[1]);
    NVS.setFloat("PosL2",Eric_Land_Pos[2]);
    NVS.setFloat("PosL3",Eric_Land_Pos[3]);
}

void Move_To_Grab_Pos()
{
  st.WritePosEx(Eric_Arm_ID[WRIST_2],Eric_AfterGrab_Pos[WRIST_2],Eric_Desire_Speed[WRIST_2]);
  vTaskDelay(3000/portTICK_PERIOD_MS);
  st.WritePosEx(Eric_Arm_ID[ELBOW],Eric_AfterGrab_Pos[ELBOW],Eric_Desire_Speed[ELBOW]);

}

void Move_To_Land_Pos_Elbow_Wrist()
{
  st.WritePosEx(Eric_Arm_ID[ELBOW],Eric_Land_Pos[ELBOW],Eric_Desire_Speed[ELBOW],Eric_Desire_Acc[ELBOW]);
  st.WritePosEx(Eric_Arm_ID[WRIST],Eric_Land_Pos[WRIST],Eric_Desire_Speed[WRIST],Eric_Desire_Acc[WRIST]);
  st.WritePosEx(Eric_Arm_ID[HAND],Eric_Land_Pos[HAND],Eric_Desire_Speed[HAND],Eric_Desire_Acc[HAND]);
  vTaskDelay(3000/portTICK_PERIOD_MS);
  st.WritePosEx(Eric_Arm_ID[WRIST_2],Eric_Land_Pos[WRIST_2],Eric_Desire_Speed[WRIST_2],Eric_Desire_Acc[WRIST_2]);
  
}

void Move_To_Land_Pos_Wrist_Elbow()
{
  st.WritePosEx(Eric_Arm_ID[WRIST],Eric_Land_Pos[WRIST],Eric_Desire_Speed[WRIST],Eric_Desire_Acc[WRIST]);
  st.WritePosEx(Eric_Arm_ID[HAND],Eric_Land_Pos[HAND],Eric_Desire_Speed[HAND],Eric_Desire_Acc[HAND]);
  st.WritePosEx(Eric_Arm_ID[WRIST_2],Eric_Land_Pos[WRIST_2],Eric_Desire_Speed[WRIST_2],Eric_Desire_Acc[WRIST_2]);
  vTaskDelay(3000/portTICK_PERIOD_MS);
  st.WritePosEx(Eric_Arm_ID[ELBOW],Eric_Land_Pos[ELBOW],Eric_Desire_Speed[ELBOW],Eric_Desire_Acc[ELBOW]);
}
//Move to Set Pos
void Move_To_Desired_Pos()
{
    u8 idnum = DOF_NUM;
    u8 ID[4];
    s16 Pos[4];
    u16 Speed[4];
    u8 Acc[4];
    for(int i=0;i<=4;i++)
    {
      ID[i]=Eric_Arm_ID[i];
      Pos[i]=s16(Eric_Initial_Pos[i]);
      Speed[i]=Eric_Desire_Speed[i];
      Acc[i]=Eric_Desire_Acc[i];
    }
    st.SyncWritePosEx(ID,idnum,Pos,Speed,Acc);
}