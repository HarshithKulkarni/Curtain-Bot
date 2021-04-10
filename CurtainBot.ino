#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include "SPIFFS.h"
#include "DHT.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>


#define DHTPIN 32 
#define DHTTYPE DHT11


DHT dht(DHTPIN, DHTTYPE);
BluetoothSerial SerialBT;


int bt_read;
bool already_open = false;
bool already_closed = false;
bool open_flag = false;
bool close_flag = false;
bool LDR_open_flag = false;
bool LDR_close_flag = false;
bool accel_open_flag = false;
bool accel_close_flag = false;
bool rtl = false;
bool ltr = false;
bool rtl_caliberation_enable = false;
bool rtl_caliberation_disable = false;
bool ltr_caliberation_enable = false;
bool ltr_caliberation_disable = false;
bool LDR_enable = false;
bool accel_enable = false;
bool temp_enable = true;
bool temp_open_flag = false;
bool temp_close_flag = false;
const int LDR_sensorPin = 2;
const int xpin = A0; // x-axis of the accelerometer
const int ypin = A3; // y-axis
const int zpin = A4; // z-axis
int start_time;
int curr_time;
int end_time;
int motor_clockwise = 15;
int motor_anticlockwise = 4;
int ltr_count = 0;
int rtl_count = 0;
String calib_time;
String calib_dir;


void setup() 
{
  Serial.begin(115200); 
  SerialBT.begin("Curtain Bot");
  dht.begin();
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(motor_clockwise, OUTPUT);
  pinMode(motor_anticlockwise, OUTPUT);
  
  if(!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  else
  {
    Serial.println("File System Mounted Successfully!!");
  }
  File file = SPIFFS.open("/time.txt");
  if(!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  while(file.available())
  {
        calib_time += file.readString();
  }
  file.close();
  calib_dir = calib_time.substring(calib_time.length()-3);
  calib_time = calib_time.substring(0,calib_time.length()-3);

  if(calib_time=="" || calib_time.toInt()==0)
  {
    Serial.println("Device is not Caliberated");
    calib_time = "";
  }
  else
  {
    Serial.print("Device is caliberated with : ");
    Serial.println(calib_time);
    if(calib_dir=="rtl")
    {
      rtl = true;
      ltr = false;
      Serial.println("Right-To-Left Mode Set!");
    }
    else if(calib_dir=="ltr")
    {
      ltr = true;
      rtl = false;
      Serial.println("Left-To-Right Mode Set!");    
    }
  }
   
}

int get_LDR_value()
{
  int lightVal = analogRead(LDR_sensorPin);
  return lightVal;
}

int get_accel_x_value()
{
  int x = analogRead(xpin); //read from xpin
  x = ((float)x - 331.5)/65*9.8;
  return x;
}

void open_curtain()
{
  if(ltr)
  {
    digitalWrite(motor_clockwise,HIGH);
    delay(calib_time.toInt());
    digitalWrite(motor_clockwise,LOW);
    Serial.println("Curtain Opened");
  }
  else if(rtl)
  {
    digitalWrite(motor_anticlockwise,HIGH);
    delay(calib_time.toInt());
    digitalWrite(motor_anticlockwise,LOW);
    Serial.println("Curtain Opened");
  }
}

void close_curtain()
{
  if(ltr)
  {
    digitalWrite(motor_anticlockwise,HIGH);
    delay(calib_time.toInt());
    digitalWrite(motor_anticlockwise,LOW);
    Serial.println("Curtain Closed");
  }
  else if(rtl)
  {
    digitalWrite(motor_clockwise,HIGH);
    delay(calib_time.toInt());
    digitalWrite(motor_clockwise,LOW);
    Serial.println("Curtain Closed");
  }
}

int get_sensor_temp()
{
  int t = dht.readTemperature();
  return t;
}
void loop() 
{
  
  if (SerialBT.available()) 
  {
    bt_read = SerialBT.parseInt();
  }
  if(LDR_enable)
  {
    int ldr_val = get_LDR_value();
//    Serial.println(ldr_val);
    if(ldr_val>=0 && ldr_val<=1000)
    {
      if(!already_open)
      {
        Serial.println("It's Day");
      }
      LDR_open_flag = true;
      LDR_close_flag = false;
    }
    else
    {
      if(!already_closed)
      {
       Serial.println("It's Night"); 
      }
      LDR_open_flag = false;
      LDR_close_flag = true;      
    }
  }
  if(temp_enable)
  {
    int t = get_sensor_temp();
    SerialBT.println(t);
  }

  if(accel_enable)
  {
    int x = get_accel_x_value();
//    Serial.println(x);
    if(x>270)
    {
      if(!already_open)
      {
        Serial.println("Accelerometer Moved towards opening the Curtain");
      }
      accel_open_flag = true;
      accel_close_flag = false;
    }
    else if(x<200)
    {
      if(!already_closed)
      {
       Serial.println("Accelerometer Moved towards closing the Curtain"); 
      }
      accel_open_flag = false;
      accel_close_flag = true;      
    }
  }

  if(rtl_caliberation_enable)
  {
    if(bt_read==15)
    {
      digitalWrite(motor_anticlockwise,HIGH);
      if(rtl_count==0)
      {
        start_time = millis();
      }
      else
      {
        curr_time = millis();
      }
      rtl_count++;
    }
  }
  if(rtl_caliberation_disable)
  {
    digitalWrite(motor_anticlockwise,LOW);
    bool formatted = SPIFFS.format();
    if (formatted)
    {
      Serial.println("SPIFFS formatted successfully");
    } 
    else 
    {
      Serial.println("Error formatting");
    }
    File file = SPIFFS.open("/time.txt", FILE_WRITE);
 
    if(!file)
    {
        Serial.println("There was an error opening the file for writing");
        return;
    }
 
    if(file.print(curr_time-start_time) && file.print("rtl"))
    {
        Serial.print("Caliberation Time Updated with : ");
        Serial.println(curr_time-start_time);
    } 
    else 
    {
        Serial.println("Caliberation Time Failed to Update");
    }
 
    file.close();
    rtl_count = 0;
    rtl_caliberation_disable = false;
  }

  if(ltr_caliberation_enable)
  {
    if(bt_read==20)
    {
      
      digitalWrite(motor_clockwise,HIGH);
      if(ltr_count==0)
      {
        start_time = millis();
      }
      else
      {
        curr_time = millis();
      }
      ltr_count++;
    }
  }
  if(ltr_caliberation_disable)
  {
    digitalWrite(motor_clockwise,LOW);
    bool formatted = SPIFFS.format();
    if (formatted)
    {
      Serial.println("SPIFFS formatted successfully");
    } 
    else 
    {
      Serial.println("Error formatting");
    }
    File file = SPIFFS.open("/time.txt", FILE_WRITE);
 
    if(!file)
    {
        Serial.println("There was an error opening the file for writing");
        return;
    }
 
    if(file.print(curr_time-start_time)&& file.print("ltr"))
    {
        Serial.print("Caliberation Time Updated : ");
        Serial.println(curr_time-start_time);
    } 
    else 
    {
        Serial.println("Caliberation Time Failed to Update");
    }
 
    file.close();
    ltr_count = 0;
    ltr_caliberation_disable = false;
  }


  
  //1 for open
  if(bt_read==1)
  {
    if(!already_open && calib_time!="")
    {
      Serial.println("Opening Window curtain");
      open_flag = true;
      close_flag = false; 
    }
    else
   {
      Serial.println("Please Caliberate the device first!");
      open_flag = false;
      close_flag = false;
      bt_read = 0;
   }
  }
  //2 for close
  if(bt_read==2)
  {
    if(!already_closed && calib_time!="")
    {
      Serial.println("Closing Window curtain");
      close_flag = true;
      open_flag = false;
    }
    else
   {
      Serial.println("Please Caliberate the device first!");
      open_flag = false;
      close_flag = false;
      bt_read = 0;
   }    
  }
  //3 for LDR_enable
  if(bt_read==3)
  {
    Serial.println("Enabling Light Sensor");
    LDR_enable = true;
  }
  //4 for LDR disable
  if(bt_read==4)
  {
    Serial.println("Disabling Light Sensor");
    LDR_enable = false;
  }
  //5 for accelerometer enable
  if(bt_read==5)
  {
    Serial.println("Enabling Accelerometer Sensor");
    accel_enable = true;
  }
  //6 for accel disable
  if(bt_read==6)
  {
    Serial.println("Disabling Accelerometer Sensor");
    accel_enable = false;
  }
  //7 for right-to-left caliberation enable
  if(bt_read==7)
  {
    Serial.println("Right-To-Left caliberation mode enabled");
    rtl_caliberation_enable = true;
    rtl_caliberation_disable = false;
    bt_read = -1;
  }

  //8 for right-to-left caliberation disable
  if(bt_read==8)
  {
    Serial.println("Right-To-Left caliberation mode disabled");
    rtl_caliberation_disable = true;
    rtl_caliberation_enable = false;
  }

  //9 for left-to-right caliberation enable
  if(bt_read==9)
  {
    Serial.println("Left-To-Right caliberation mode enabled");
    ltr_caliberation_enable = true;
    ltr_caliberation_disable = false;
    bt_read = -1;
  }

  //10 for left-to-right caliberation disabled
  if(bt_read==10)
  {
    Serial.println("Left-To-Right caliberation mode disabled");
    ltr_caliberation_disable = true;
    ltr_caliberation_enable = false;
  }
  // 11 for temperature sensor enable
  if(bt_read==11)
  {
    Serial.println("Temperature Sensor enabled!");
    temp_enable = true;       
  }

  // 12 for temperature sensor disable
  if(bt_read==12)
  {
    Serial.println("Temperature Sensor disabled!");
    temp_enable = false;
  }

  if(open_flag || LDR_open_flag || accel_open_flag || temp_open_flag)
  {
    if(!already_open)
    {
      open_curtain();
      open_flag = false;
      LDR_open_flag = false;
      accel_open_flag = false;
      temp_open_flag = false;
      already_open = true;
      already_closed = false;
    }
  }
  if(close_flag || LDR_close_flag || accel_close_flag || temp_close_flag)
  {
    if(!already_closed)
    {
      close_curtain();
      close_flag = false;
      LDR_close_flag = false;
      temp_close_flag = false;
      accel_close_flag = false;
      already_open = false;
      already_closed = true;

    }
  }
}
