#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "math.h"

SoftwareSerial serialLTS = SoftwareSerial(12,13);

#define OLED_RESET 4
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

typedef struct
{
  uint8_t BYTE[3]; 
}int24_t;

struct TAG_FRAME0
{ 
  int24_t pos[3];
  int24_t vel[3];
  int24_t dis[8];
  float g[3];
  float acc[3];
  float res1[3];
  
  int16_t ang[3];
  float q[4];
  uint32_t res2;
  
  uint32_t local_time;
  uint32_t system_time;
  uint8_t res3;
  
  uint8_t eop[3];
  uint16_t voltage;
  uint8_t res4[5];
};

TAG_FRAME0 tf0;
const unsigned char LTS_HEADER[] = { 0x55, 0x01, 0x00, 0x02};
float coordinatex[11], coordinatey[11], coordinate[2], distance;

void setup() 
{
  Serial.begin(115200);
  serialLTS.begin(115200);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))// Address 0x3C for 128x32
  { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.display(); 
}

void loop()
{
    static int timesforvariance = 0;
    
    if (processLTS())
    {
      screen_display();
      int calibration = 0 ;
      calibration = coordinate_cal(timesforvariance);
      
      timesforvariance++;
      timesforvariance = timesforvariance - calibration;
      
      if (timesforvariance == 5)
      { 
        variance_cal();
        Serial.print("Coo:");Serial.print(coordinate[0]);Serial.print(", ");Serial.print(coordinate[1]);Serial.println();
        timesforvariance = 0;
       /*Serial.print("lx");Serial.print(lx);Serial.print(" ");Serial.print("ly");Serial.print(ly);Serial.println();*/
       /*Serial.print(direction_angle);Serial.println();*/
      }
    }
}

bool processLTS() 
{
  static int fpos = 0;
  const int payloadSize = sizeof(tf0);
  while ( serialLTS.available() ) 
  {
    byte c = serialLTS.read();
    if ( fpos < 4 ) 
    {
      if ( c == LTS_HEADER[fpos] )
      {
        fpos++;
      }       
      else
        fpos = 0;
    }
    else 
    {
      if ( fpos >= 4 )
      {
        ((unsigned char*)(&tf0))[fpos-4] = c;
        /*Serial.print("data");Serial.print(fpos-3);Serial.print(": ");Serial.print(((unsigned char*)(&tf0))[fpos-4], HEX);Serial.println();*/
        fpos++;
        if ( fpos - 4 >= payloadSize ) 
        {
          fpos = 0;
          /*Serial.print(tf0.pos[0].BYTE[0], HEX);Serial.print(" ");Serial.print(tf0.pos[0].BYTE[1], HEX);Serial.print(" ");Serial.print(tf0.pos[0].BYTE[2], HEX);Serial.print(" ");*/
          return true;
        }
      }
    }
    return false;
  }
}  

int coordinate_cal(int a)
{
  uint8_t cx[3], cy[3], cz[3];
  int32_t cxx, cyy, czz;
  float px, py, pz;
  for(int i = 0; i < 3; i++)
  {
    cx[i] = tf0.pos[0].BYTE[i];
    cy[i] = tf0.pos[1].BYTE[i];
    cz[i] = tf0.pos[2].BYTE[i];
  }
  
  cxx = (((cx[0] << 8) & 0xffff) | (cx[1] * 0x10000) | (cx[2] * 0x1000000)) / 0x100;
  cyy = (((cy[0] << 8) & 0xffff) | (cy[1] * 0x10000) | (cy[2] * 0x1000000)) / 0x100;
  czz = (((cz[0] << 8) & 0xffff) | (cz[1] * 0x10000) | (cz[2] * 0x1000000)) / 0x100; 

  px = cxx / 1000.0f;
  py = cyy / 1000.0f;
  pz = czz / 1000.0f;
  
  Serial.print(cx[0], HEX);Serial.print(" ");Serial.print(cx[1], HEX);Serial.print(" ");Serial.print(cx[2], HEX);Serial.print(" ");Serial.println();
  Serial.print(cy[0], HEX);Serial.print(" ");Serial.print(cy[1], HEX);Serial.print(" ");Serial.print(cy[2], HEX);Serial.print(" ");Serial.println();
  Serial.print(cz[0], HEX);Serial.print(" ");Serial.print(cz[1], HEX);Serial.print(" ");Serial.print(cz[2], HEX);Serial.print(" ");Serial.println();
  Serial.print(px);Serial.print(", ");Serial.print(py);Serial.print(", ");Serial.print(pz);Serial.println();
  
  if (px > 80 || py > 80 || px < -80 || py < -80)
  {
    return 1;
  }
  else
  {
    coordinatex[a] = px;
    coordinatey[a] = py;
    return 0;
  }
}

void variance_cal()
{ 
  float coo[2];
  float variancex = 0;
  float variancey = 0;
  float sumx = 0;
  float sumy = 0;

  for (int a = 0; a < 5 ; a++)
  {
    sumx += coordinatex[a];
    sumy += coordinatey[a];
  }
  coo[0] = sumx / 5;
  coo[1] = sumy / 5;
  
  for (int j = 0; j < 5; j++)
  {
    variancex += pow(coordinatex[j] - coo[0], 2) / 5;
    variancey += pow(coordinatey[j] - coo[1], 2) / 5;
  }
  /*Serial.print("v: ");Serial.print(variancex, 4);Serial.print(", ");Serial.print(variancey, 4);Serial.println();*/
  if (variancex < 0.002 && variancey < 0.002)
  {
    coordinate[0] = coo[0];
    coordinate[1] = coo[1];
  } 
  coordinate[0] = coo[0];
  coordinate[1] = coo[1];
}

void screen_display() 
{ 
    display.print("lat: "); display.print(coordinate[0]); display.print( '\n');
    display.print("lon: "); display.print(coordinate[1]); display.print('\n');
}
