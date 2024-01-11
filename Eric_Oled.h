#include <Wire.h>
// SSD1306: 0x3C
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels, 32 as default.
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
// set the interval of the threading.
#define threadingInterval 600

void InitScreen();
void screenUpdate();
void InfoUpdateThreading(void *pvParameter);
void Eric_Servo_Check(bool searchCommand);
// Creat Handler
TaskHandle_t ScreenUpdateHandle;
//Creat SSD1306 object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*
Initialzie OLED
*/
void InitScreen(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}

void screenUpdate(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  // Row1.
//   display.print(F("MAC:"));display.println(MAC_ADDRESS);
 display.println(Eric_Arm_Name[ERIC_PART_SELECT]);
  // Row2.
  display.print(F("V:"));display.print(float(Eric_Arm_FB[ERIC_PART_SELECT].voltage)/10);display.print(F(" "));display.println(IP_ADDRESS);
  // Row3.
  if(WIFI_MODE == 1){display.print(F(" AP "));display.println(AP_SSID);}
  else if(WIFI_MODE == 2){display.print(F(" STA "));display.print(F("RSSI"));display.println(WIFI_RSSI);}
  else if(WIFI_MODE == 3){display.print(F(" TRY:"));display.print(STA_SSID);display.println(F(""));}

  // Row4.
  if(Eric_Arm_Status[ERIC_PART_SELECT]){
    display.print(F(" ID:"));display.print(Eric_Arm_ID[ERIC_PART_SELECT]);
    display.print(F(" M"));display.print(Eric_Arm_FB[ERIC_PART_SELECT].mode);
    display.print(F(" P"));display.println(Eric_Arm_FB[ERIC_PART_SELECT].pos);
  }
  else{
    display.println(F("NOT DETECTED SERVO"));
  }
  display.display();
}

void InfoUpdateThreading(void *pvParameter){
  while(1){

    getFeedBack();
    getWifiStatus();
    screenUpdate();
    delay(threadingInterval);
    Eric_Servo_Check(searchCmd);
  }
}
//Cheak all the Servo have been detected and worked well
void Eric_Servo_Check(bool searchCommand)
{
    if (searchCommand)
    {
        int PingStatus;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,0);
        for(int i =0; i<DOF_NUM;i++)
        {
            PingStatus=st.Ping(Eric_Arm_ID[i]);
            //Not Detected
            if(PingStatus ==-1)
            {
                Eric_Arm_Status[i] = NOT_FOUND;
            }
            else{
                Eric_Arm_Status[i]=DETECTED;
            }
            delay(1);
        }
        for(int i =0; i<DOF_NUM;i++)
        {
            switch (Eric_Arm_Status[i])
            {
                case NOT_FOUND:
                    display.print(Eric_Arm_Name[i]);display.println("  NOT FOUND"); 
                    break;
            
                case DETECTED:
                    display.print(Eric_Arm_Name[i]);display.println("  DETECTED");
                    break;
            }
            display.display();
            delay(500);
        }
        delay(1000);
    }
    searchCmd = false;
}


