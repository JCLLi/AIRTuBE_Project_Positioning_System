#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "math.h"

SoftwareSerial serialGPS = SoftwareSerial(10,11);//9，10 gps
SoftwareSerial serialLTS = SoftwareSerial(12,13);//4，3 lts

#define pinLEFT 2
#define pinRIGHT 3
#define OLED_RESET 4
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define RADTODEG 57.295779513082320876f
#define LSB_M_TO_LAT_LONG 8.993216059e-6

typedef struct
{
  uint8_t BYTE[3]; =
}int24_t;

struct NAV_POSLLH
{
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

NAV_POSLLH posllh;

struct GEOLOC 
{
  double orglat;
  double orglon;
  double curlat;
  double curlon;
};

GEOLOC GPSlocation;

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

float coordinatex[5], coordinatey[5], coordinate[2], DistanceLEFT, DistanceRIGHT;
float direction_angle, rotation_angle, disforward, disbackward, dislast;
float ang, lx, ly, lt;

int StateLEFT, LastStateLEFT,StateRIGHT, LastStateRIGHT;

static int modejudge = 0;
static int statee = 0;
static long counterLEFT, counterRIGHT; 
static float startpositionx, startpositiony;

const unsigned char GPS_HEADER[] = { 0xB5, 0x62 };
const unsigned char LTS_HEADER[] = { 0x55, 0x01, 0x00, 0x02};

void setup() 
{
  Serial.begin(115200);
  serialGPS.begin(19200);
  serialLTS.begin(115200);
  pinMode (2, INPUT);
  pinMode (3, INPUT);
  pinMode (A8, INPUT_PULLUP);
  pinMode (A9, INPUT_PULLUP);
  pinMode (A10, INPUT_PULLUP);
  pinMode (A11, INPUT_PULLUP);
  pinMode (A12, INPUT_PULLUP);
  StateLEFT = digitalRead(pinLEFT); 
  StateRIGHT = digitalRead(pinRIGHT);
  attachInterrupt(0, distance_calLEFT, CHANGE);  
  attachInterrupt(1, distance_calRIGHT, CHANGE);  

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
  serialGPS.listen();
  if (serialGPS.isListening())
  {
    int read_check = 0;
    while(read_check != 1)
    {
      if (processGPS())
      {
        read_check = original_geolocation_cal();
      }
    }
    delay(2000);
  }
  serialGPS.end();
  
  while(1)
  {
    serialLTS.listen();
    static int timesforvariance = 0;
    static int mode = 0;
    int state;
    
    if (processLTS())
    {
      modejudge = mode_judge();
      screen_display();
      int calibration = 0 ;
      calibration = coordinate_cal(timesforvariance);
      
      timesforvariance++;
      timesforvariance = timesforvariance - calibration;
      
      if (timesforvariance == 5)
      { 
        variance_cal();
        current_geolocation_cal();
        Serial.print("Coo:");Serial.print(coordinate[0]);Serial.print(", ");Serial.print(coordinate[1]);Serial.println();
        timesforvariance = 0;
       /*Serial.print("lx");Serial.print(lx);Serial.print(" ");Serial.print("ly");Serial.print(ly);Serial.println();*/
       /*Serial.print(direction_angle);Serial.println();*/
      }
    }
    
      lx = coordinate[0] - startpositionx;;
      ly = coordinate[1] - startpositiony;;
      cal(lx, ly);
    
      if (mode - modejudge != 0)
      {
        /*Serial.print("START:");Serial.print(startpositionx);Serial.print(", ");Serial.print(startpositiony);Serial.println();*/
        mode = modejudge;
        startpositionx = coordinate[0];
        startpositiony = coordinate[1]; 
        counterRIGHT = 0;
        counterLEFT = 0;
        rotation_angle = 0;
        disforward = 0;
        disbackward = 0;
        if(mode == 5) DistanceRIGHT = 0;
        dislast = DistanceRIGHT;
      } 
  }
}

/*******************************************************GPS RELEATED FUNCTION*********************************************************/
void calGPS_checksum(unsigned char* CK) 
{
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) 
  {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() 
{
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  while ( serialGPS.available() ) {
    byte c = serialGPS.read();
    if ( fpos < 2 ) 
    {
      if ( c == GPS_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else 
    {
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&posllh))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) 
      {
        calGPS_checksum(checksum);
      }
      else if ( fpos == (payloadSize+3) )
      {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) )
      {
        fpos = 0;
        if ( c == checksum[1] ) 
        {
          delay(2000);
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) 
      {
        fpos = 0;
      }
    }
  }
  return false;
}

int original_geolocation_cal()
{
  GPSlocation.orglat = 0;
  GPSlocation.orglon = 0;
  for(int i = 0; i < 1000; i++)
  {
    processGPS();
    GPSlocation.orglat = GPSlocation.orglat + posllh.lat * 0.0000001;
    GPSlocation.orglon = GPSlocation.orglon + posllh.lon * 0.0000001;
  }
  
  GPSlocation.orglat = GPSlocation.orglat / 1000;
  GPSlocation.orglon = GPSlocation.orglon / 1000;
  /*Serial.print( GPSlocation.orglat, 10 );Serial.print(" "); Serial.print( GPSlocation.orglon, 10 ); Serial.println();*/
  return 1;
}

void current_geolocation_cal()
{
  GPSlocation.curlat = GPSlocation.orglat + LSB_M_TO_LAT_LONG * coordinate[1];
  GPSlocation.curlon = GPSlocation.orglon + LSB_M_TO_LAT_LONG * coordinate[0];
  /*Serial.print( GPSlocation.curlat, 10 );Serial.print(" "); Serial.print( GPSlocation.curlon, 10 ); Serial.println();*/
}
/*******************************************************GPS RELEATED FUNCTION******************************************************8***/


/*******************************************************LTS(INDOOR SYSTEM / BEACONS) RELEATED FUNCTION******************************************************8***/
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
        /*Serial.print("data");Serial.print(fpos-3);Serial.print(": ");
        Serial.print(((unsigned char*)(&tf0))[fpos-4], HEX);Serial.println();*/
        fpos++;
        if ( fpos - 4 >= payloadSize ) 
        {
          fpos = 0;
          /*Serial.print(tf0.pos[0].BYTE[0], HEX);Serial.print(" ");Serial.print(tf0.pos[0].BYTE[1], HEX);
          Serial.print(" ");Serial.print(tf0.pos[0].BYTE[2], HEX);Serial.print(" ");*/
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
  
  /*Serial.print(cx[0], HEX);Serial.print(" ");Serial.print(cx[1], HEX);Serial.print(" ");
  Serial.print(cx[2], HEX);Serial.print(" ");Serial.println();
  Serial.print(cy[0], HEX);Serial.print(" ");Serial.print(cy[1], HEX);Serial.print(" ");
  Serial.print(cy[2], HEX);Serial.print(" ");Serial.println();
  Serial.print(cz[0], HEX);Serial.print(" ");Serial.print(cz[1], HEX);Serial.print(" ");
  Serial.print(cz[2], HEX);Serial.print(" ");Serial.println();
  Serial.print(px);Serial.print(", ");Serial.print(py);Serial.print(", ");Serial.print(pz);
  Serial.println();*/
  
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

void cal(float x, float y )
{
    lt = pow(pow(x, 2)+ pow(y, 2), 0.5);
    if (lt < 0) lt = fabs(lt);
    ang = asin(fabs(y) / lt);
    
    if (x < 0)
    {
      if (y < 0)
      {
        ang = ang + 180;
      }
      else if (y == 0)
      {
        ang = 180;
      }
      else if (y > 0)
      {
        ang = 180 - ang;
      }
    }
    else if (x == 0)
    {
      if (y < 0)
      {
        ang = 270;
      }
      else if (y > 0)
      {
        ang = 90;
      }
    }
    else if (x > 0)
    {
      if (y < 0)
      {
        ang = 360 - ang;
      }
      else if (y == 0)
      {
        ang = 0;
      }
      else if (y > 0)
      {
        ang = ang;
      }
    }
    direction_angle = ang;
}
/*******************************************************LTS(INDOOR SYSTEM / BEACONS) RELEATED FUNCTION******************************************************8***/

/*******************************************************ROTARY ENCODER RELEATED FUNCTION******************************************************8***/
void distance_calLEFT()
{
  float meter = 0;
  float distick = 10.4772115;
  float dismov = 0;
  
  StateLEFT = digitalRead(pinLEFT);
  
  if (StateLEFT != LastStateLEFT)
  {     
    counterLEFT ++;
  }
  else
  {
    counterLEFT = counterLEFT;
  }
  
  dismov = counterLEFT * distick /10 /2;
  
  LastStateLEFT = StateLEFT; 
  
  if (modejudge == 4)
  {
    DistanceLEFT = dismov;
    angle_cal();
  }
  /*Serial.print("Ticks= ");Serial.print(counter);Serial.print(" | ");Serial.print("Distance moved= ");
  Serial.print(dismov);Serial.println(" cm");*/
}

void distance_calRIGHT()
{
  float meter = 0;
  float distick = 10.4772115;
  float dismov = 0;
  
  StateRIGHT = digitalRead(pinRIGHT);
  
  if (StateRIGHT != LastStateRIGHT)
  {     
    counterRIGHT ++;
  }
  else
  {
    counterRIGHT = counterRIGHT;
  }
  
  dismov = counterRIGHT * distick /10 /2;
  
  LastStateRIGHT = StateRIGHT; 
  
  if (modejudge == 1 || modejudge == 2)
  {
    if (modejudge == 1)
    {
      disforward = dismov;
      /*Serial.print("disforward ");Serial.print(disforward);Serial.println();*/
    }
    else if (modejudge == 2)
    {
      disbackward = 0 - dismov;
      /*Serial.print("disbackward ");Serial.print(disbackward);Serial.println();Serial.println();*/
    }
    DistanceRIGHT = dislast + disforward + disbackward;
  }
  else if (modejudge == 3)
  {
    DistanceRIGHT = dismov;
    /*Serial.print(DistanceRIGHT);Serial.println();*/
    angle_cal();
  }
  
  /*Serial.print("Ticksr= ");Serial.print(counter);Serial.print(" | ");
  Serial.print("Distance movedright= ");Serial.print(dismov);Serial.println(" cm");*/
}

void angle_cal()
{
  float a = 0;
  if(modejudge == 3)  rotation_angle = DistanceRIGHT / 13.5;
  if(modejudge == 4)  rotation_angle = DistanceRIGHT / 13.5;
  if (rotation_angle  >= 6.2831) rotation_angle = rotation_angle - 6.2831;
  if (rotation_angle  >= 12.5662) rotation_angle = rotation_angle - 12.5662;
  /*Serial.print("RT"); Serial.print(rotation_angle * RADTODEG);Serial.println();*/
}

/*******************************************************ROTARY ENCODER RELEATED FUNCTION******************************************************8***/

int mode_judge()
{
    int switchstate1 = 1;
    int switchstate2 = 1;
    int switchstate3 = 1;
    int switchstate4 = 1;
    int switchstate5 = 1;
    
    switchstate1 = digitalRead(A8);
    switchstate2 = digitalRead(A9);
    switchstate3 = digitalRead(A10);
    switchstate4 = digitalRead(A11);
    switchstate5 = digitalRead(A12);
    if (switchstate1 == LOW)
    {
      statee = 1;
      return statee;// go ahead
       
    }
    else if (switchstate2 == LOW)
    {
      statee = 2;
      return statee;// go back
    }
    else if (switchstate3 == LOW)
    {
      return 3;// turn left
    }
    else if (switchstate4 == LOW)
    {
      return 4;//turn right
    }
    else if (switchstate5 == LOW)
    {
      return 5;//turn right
    }
    else
    {
      return statee;  
    }
}

void screen_display() 
{ 
  display.clearDisplay();

  display.setTextSize(0.1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  if (modejudge == 1 || modejudge == 2)
  {
    /*display.print("lat: "); display.print(coordinate[0]); display.print('\n');
    display.print("lon: "); display.print(coordinate[1]); display.print('\n');*/
    display.print("lat: "); display.print(GPSlocation.curlat, 10); display.print( '\n');
    display.print("lon: "); display.print(GPSlocation.curlon, 10); display.print('\n');
    display.print("distance: "); display.print(DistanceRIGHT); display.print('\n');
    display.print("direction: "); display.print(direction_angle);
    display.display();
  }
  if (modejudge == 3)
  {
    display.print("lat: "); display.print(GPSlocation.curlat, 10); display.print('\n');
    display.print("lon: "); display.print(GPSlocation.curlon, 10); display.print('\n');
    display.print("rotation angle: "); display.print(rotation_angle * RADTODEG); display.print('\n'); 
    display.print("   turning left   "); display.print('\n');
    display.display();
  }
  if (modejudge == 4)
  {
    display.print("lat: "); display.print(GPSlocation.curlat, 10); display.print('\n');
    display.print("lon: "); display.print(GPSlocation.curlon, 10); display.print('\n');
    display.print("   turning right   "); display.print('\n');
    display.print("rotation angle: "); display.print(rotation_angle * RADTODEG); display.print('\n'); 
    display.display();
  }
}
