
#define pinA 2
#define pinB 3

int StateLEFT, LastStateLEFT, StateRIGHT, LastStateRIGHT;
  
void setup() 
{ 
   pinMode (2, INPUT);
   pinMode (3, INPUT);
   Serial.begin (115200);
   
   StateLEFT = digitalRead(pinA); 
   StateRIGHT = digitalRead(pinA);
   attachInterrupt(0, distance_calLEFT, CHANGE);  
   attachInterrupt(1, distance_calRIGHT, CHANGE);  
   
 } 

void loop() 
{ 
  
}

void distance_calLEFT()
{
  static long counter = 0; 
  float meter = 0;
  float moving_distance = 0;
  float distick = 10.4772115;
  float dismov = 0;
  
  StateLEFT = digitalRead(pinA);
  
  if (StateLEFT != LastStateLEFT)
  {     
    counter ++;
  }
  else
  {
    counter = counter;
  }
  
  dismov = counter * distick /10 /2;
  
  Serial.print("Ticks= ");Serial.print(counter);Serial.print(" | ");Serial.print("Distance moved= ");Serial.print(dismov);Serial.println(" cm");
  
  LastStateLEFT = StateLEFT; 
}

void distance_calRIGHT()
{
  static long counter = 0; 
  float meter = 0;
  float moving_distance = 0;
  float distick = 10.4772115;
  float dismov = 0;
  
  StateRIGHT = digitalRead(pinB);
  
  if (StateRIGHT != LastStateRIGHT)
  {     
    counter ++;
  }
  else
  {
    counter = counter;
  }
  
  dismov = counter * distick /10 /2;
  
  Serial.print("Ticksr= ");Serial.print(counter);Serial.print(" | ");Serial.print("Distance movedright= ");Serial.print(dismov);Serial.println(" cm");
  
  LastStateRIGHT = StateRIGHT; 
}
