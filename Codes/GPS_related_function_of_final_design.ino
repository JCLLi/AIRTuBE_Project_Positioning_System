#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>

SoftwareSerial serialGPS = SoftwareSerial(10,11);

#define LSB_M_TO_LAT_LONG 8.993216059e-6



const unsigned char GPS_HEADER[] = { 0xB5, 0x62 };

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

void setup() 
{
  Serial.begin(115200);
  serialGPS.begin(19200);
}

void loop()
{
  if (processGPS())
  {
    original_geolocation_cal();
  }
}

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

void original_geolocation_cal()
{
  float original_lat[10];
  float original_lon[10];
  GPSlocation.orglat = 0;
  GPSlocation.orglon = 0;
  for(int i = 0; i < 1000; i++)
  {
    processGPS();
    GPSlocation.orglat = GPSlocation.orglat + posllh.lat * 0.000000001;
    GPSlocation.orglon = GPSlocation.orglon + posllh.lon * 0.000000001;
  }
  
  GPSlocation.orglat = GPSlocation.orglat / 10;
  GPSlocation.orglon = GPSlocation.orglon / 10;
  Serial.print( GPSlocation.orglat, 10 );Serial.print(" "); Serial.print( GPSlocation.orglon, 10 ); Serial.println();
}

void current_geolocation_cal()
{
  GPSlocation.curlat = GPSlocation.orglat + LSB_M_TO_LAT_LONG ;
  GPSlocation.curlon = GPSlocation.orglon + LSB_M_TO_LAT_LONG;
}
