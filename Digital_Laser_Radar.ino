#include "AdafruitIO_Ethernet.h"
#include <SPI.h>
#include <Ethernet.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_Fingerprint.h>
#include <Servo.h>
#include <EEPROM.h>

/************************ Adafruit IO Config *******************************/
// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "Maker_Gavin"
#define IO_KEY "aio_*******"
AdafruitIO_Ethernet io(IO_USERNAME, IO_KEY);

/************************ Example Starts Here *******************************/
// holds the current count value for our sketch
int count = 0;
// holds the boolean (true/false) state of the light
bool is_on = false;

// track time of last published messages and limit feed->save events to once
// every IO_LOOP_DELAY milliseconds
#define IO_LOOP_DELAY 15000
unsigned long lastUpdate;

// set up the 'counter' feed
AdafruitIO_Feed *tof_feed = io.feed("3D Laser Radar Tower");
int num_count;

// start reading from the first byte (address 0) of the EEPROM
byte value;

//#include "TouchyTouch.h"
//const int touch_threshold_adjust = 300;
//const int touch_pins[] = {14,2,3,6,7,8,9,10,11,12,15,13};
//const int touch_count = sizeof(touch_pins) / sizeof(int);
//TouchyTouch touches[touch_count];

SoftwareSerial DT(2, 3); //RX--2  TX--3

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SERVO_PIN1 28
Servo servo1;
#define SERVO_PIN2 27
Servo servo2;

int Key_num = 0;
int lock_status = 0;
int change_password_flag = 0;
uint32_t time_hold = 0;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
//byte mac[] = {
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
//};
IPAddress ip(10, 0, 1, 226);

//////////////////////////////////// Instruction Set ///////////////////////////////////////////////
const byte SYSCMD[4][8] = {
  {0x01, 0x06, 0x00, 0x20, 0x00, 0x8C, 0x89, 0xA5},// 01 06 00 20 00 8C 89 A5 Set offset calibration distance to 140mm
  {0x01, 0x06, 0x00, 0x21, 0x00, 0x64, 0xD8, 0x2B},// 01 06 00 21 00 64 D8 2B Set xtalk calibration distance to 100mm
  {0x01, 0x06, 0x00, 0x06, 0x00, 0x01, 0xA8, 0x0B},// 01 06 00 06 00 01 A8 0B Load calibration
  {0x01, 0x06, 0x00, 0x01, 0x10, 0x00, 0xD5, 0xCA},// 01 06 00 01 10 00 D5 CA Restart distance measuring module
};
const byte distanceCMD[2][8] = {
  {0x01, 0x06, 0x00, 0x04, 0x00, 0x00, 0xC8, 0x0B},// 01 06 00 04 00 00 C8 0B Set TOF400 measuring range to 1.3m / Set TOF200 measuring range to default
  {0x01, 0x06, 0x00, 0x04, 0x00, 0x01, 0x09, 0xCB},// 01 06 00 04 00 01 09 CB Set TOF400 measuring range to 4.0m / Set TOF200 measuring range to high precision
};
//const byte timeCMD[1][8] = {
//  {0x01, 0x06, 0x00, 0x05, 0x01, 0xF4, 0x99, 0xDC},// 01 06 00 05 01 F4 99 DC Set continuous output and output time interval to 500MS
//};
const byte timeCMD[1][8] = {
  {0x01, 0x06, 0x00, 0x05, 0x00, 0x32, 0x18, 0x1E},// 01 06 00 05 00 32 18 1E Set continuous output and output time interval to 50MS
};
//const byte timeCMD[1][8] = {
//  {0x01, 0x06, 0x00, 0x05, 0x00, 0x10, 0x98, 0x07},//01 06 00 05 01 F4 99 DC Set continuous output and output time interval to 15MS
//};

//*********************TOF SYS System Settings*************************//
//*********Distance Module Restart Function
void modRST()// Distance module restart
{
  for (int i = 0; i < 8; i++)
    DT.write(SYSCMD[3][i]);
}

//*********Distance Output Time Interval Function
void outputTIME500MS()// Output time interval 500ms
{
  for (int i = 0; i < 8; i++)
    DT.write(timeCMD[0][i]);
}

//*************Mode Selection****************************//
void shortdistance()// Short distance mode
{
  for (int i = 0; i < 8; i++)
    DT.write(distanceCMD[0][i]);
}

void longdistance()// Long distance mode
{
  for (int i = 0; i < 8; i++)
    DT.write(distanceCMD[1][i]);
}

long TOF_DATA[72][54];
char TOF_DATA_Char[72][54];
String TOF_DATA_String;

// Define thresholds and corresponding characters
const int thresholds[] = {600, 400, 350, 300, 250, 200, 150, 100, 50};
const char characters[] = {' ', '.', ':', '-', '=', '+', '*', '#', '%'};

void setup() {
  DT.begin(115200);
  Serial.begin(115200);
  while(! Serial);
  delay(1000);
  Serial.println("Connecting to Adafruit IO");
  Ethernet.init(17);  // WIZnet W5100S-EVB-Pico
  // connect to io.adafruit.com
  io.connect();

  // attach message handler for the counter feed.
  tof_feed->onMessage(handleCount);

  while(io.status() < AIO_CONNECTED) {
    Serial.print("please check Ethernet Hardware.");
    delay(500);
  }

  servo1.attach(SERVO_PIN1);
  servo1.write(118); // horizontal center position // 76<===>118
  servo2.attach(SERVO_PIN2);    
  servo2.write(130); //vertical center position // 130
  
  shortdistance();
  //longdistance();
  delay(2000);
  
  outputTIME500MS();//Output timeout 500MS
  delay(2000);
  
  modRST();//Reset TOF module
  delay(2000);
  
  servo1.write(36); // Locked position // 76<===>118
  servo2.write(72); // Locked position    
  delay(200);
  // make sure all feeds get their current values right away
  tof_feed->get();
}

void loop() {    

  // process messages and keep connection alive
  io.run();
  tof_feed->save("3D Laser Rader(Scan mode)");
  // Check for touch button input
  // Add your touch button logic here
  //  Touch_handling();  
  
    TOF_DATA_String= "";      
    for(int i =36; i<180; i=i+4 )
    {
      servo1.write(i); 
      delay(5);
      Serial.print("position:"); 
      Serial.println((i-36)/2);
      for(int m =72; m<180; m=m+2)
      {    
          while(DT.available() < 6)
          {
          }
          char a = DT.read();
          if(a != 0x01)
          {
            m--;
            continue;
          }
          byte Buf[6];
          DT.readBytes(Buf, 6);
          if (Buf[2] == 0xFF)
          {
            //Serial.println("Distance:");
            //Serial.println("invalid");
            //m--;
            //continue;
            TOF_DATA[(i-36)/2][(m-72)/2] = 4000;
          }
          else
          {
            TOF_DATA[(i-36)/2][(m-72)/2] = Buf[2] * 256 + Buf[3];
          }
          servo2.write(m+2);          
      }
      servo1.write(i+2);
      delay(5);
      Serial.print("position:"); 
      Serial.println((i-34)/2);
      for(int n=180; n>72; n=n-2)
      {        
          while(DT.available() < 6)
          {
          }        
          //Serial.print("CMD: ");
          char a = DT.read();
          if(a != 0x01)
          {
            n--;
            continue;
          }
          byte Buf[6];
          DT.readBytes(Buf, 6);
          if (Buf[2] == 0xFF)
          {
            //Serial.println("Distance:");
            //Serial.println("invalid");
            //m--;
            //continue;
            TOF_DATA[(i-34)/2][(n-72)/2-1] = 4000;
          }
          else
          {
            TOF_DATA[(i-34)/2][(n-72)/2-1] = Buf[2] * 256 + Buf[3];
          }
//          Serial.print((n-72)/2-1);
//          Serial.print(" "); 
//          Serial.print(TOF_DATA[(i-34)/2][(n-72)/2-1]);
//          Serial.println("mm");           
          servo2.write(n-1);
      }
    }

    for(int q = 0; q<54; q++)
    {
       for(int p = 0; p<72; p++)
       {
          // Find the appropriate character and print it
          char toPrint = '@'; // Default character
          for (int i = 0; i < sizeof(thresholds) / sizeof(thresholds[0]); i++) {
            if (TOF_DATA[71-p][q] > thresholds[i]) {
              toPrint = characters[i];
              break;
            }
          }          
          TOF_DATA_String += toPrint;
      }
      tof_feed->save((TOF_DATA_String.substring(q*72, (q+1)*72-1)));
      delay(2000);
      Serial.println("");
    }
    Serial.println(TOF_DATA_String);
}

//    for(int q = 0; q<54; q++)
//    {
//       for(int p = 0; p<72; p++)
//       {          
//          Serial.print(TOF_DATA[71-p][q]);
//          Serial.print("mm ");
//       }
//       Serial.println("");
//    }    

void display_logo(uint16_t x,uint16_t y)
{  
  uint8_t cloud_pixel[5*8]=
  {
    0b00110010,0b01001001,0b01001001,0b01001001,0b00100110, // S
    0b00000010,0b00010101,0b00010101,0b00010101,0b00001111, // a
    0b00001000,0b00111111,0b01001000,0b01001000,0b00100000, // f
    0b00001110,0b00010101,0b00010101,0b00010101,0b00001100, // e
    0b00000000,0b00000000,0b00000000,0b00000000,0b00000000, // space
    0b01111111,0b01001001,0b01001001,0b01001001,0b00110110, // B
    0b00001110,0b00010001,0b00010001,0b00010001,0b00001110, // o
    0b00010001,0b00001010,0b00000100,0b00001010,0b00010001, // x
  };
  uint16_t _x = x - 40;
  uint16_t _y = y - 10;
  for(uint8_t i=0;i<8;i++)
  {
    for(uint8_t m=0;m<5;m++)
    {
      _y = y - 8;
      for(uint8_t n=0;n<8;n++)
      {
        if((cloud_pixel[i*5+m]>>(7-n))&0x01)
        {
            display.fillRect(_x+1, _y+1, 1, 1, SH110X_WHITE);            
        }
        _y += 2;
      }
      _x = _x + 2;
    }  
  }
  display.setTextSize(2);
  display.fillRect(21, 20, 89, 17, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setCursor(45, 21);
  display.println("Lock");
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.println("*:Clear       #:Enter");
  display.display();
}


// you can also attach multiple feeds to the same
// meesage handler function. both counter and counter-two
// are attached to this callback function, and messages
// for both will be received by this function.
void handleCount(AdafruitIO_Data *data) {

  Serial.print("received <- ");

  // since we are using the same function to handle
  // messages for two feeds, we can use feedName() in
  // order to find out which feed the message came from.
  Serial.print(data->feedName());
  Serial.print(" ");

  // print out the received count or counter-two value
  Serial.println(data->value());

}
