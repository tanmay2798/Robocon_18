#include <Sabertooth.h>

//initializing all the variables                      
#define encodPinA1      2                       
#define encodPinB1      3                                          
#define LOOPTIME        100                     
#define NUMREADINGS     10

int readings[NUMREADINGS];
unsigned long lastMilli = 0;                   
unsigned long lastMilliPrint = 0;              
int speed_req = 30;                            // Set Point
int speed_act = 0;                              //actual value
int st_val = 0;                                                              
volatile long count = 0; // revolution counter
volatile int count1 = 0; // revolution counter for position
float Kp = 0.4;          //setting Kp  
float Kd = 1;            //setting Kd

Sabertooth ST(128, Serial2);

//------------------------------------ SETUP ------------------------------------------------------------

void setup() {  
  Serial.begin(115600);             // laptop
  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  ST.autobaud();
 //setting encoder pins as input
  pinMode(encodPinA1, INPUT);   
  pinMode(encodPinB1, INPUT); 
 //internal pullup for encoders
  digitalWrite(encodPinA1, HIGH);                      
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(1, read_encoder, FALLING);  //set pin no 2 as interrupt pin (INTERRUPT 1) 
  for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;
}

//------------------------------------ LOOP ---------------------------------------------------------

void loop() {
  getParam();                                                                 // check keyboard
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                                 // enter timed loop
    lastMilli = millis();
    getMotorData();                                                          // calculate speed
    st_val= updatePid(st_val, speed_req, speed_act);         // compute PWM value
    // Write st_val to sabertooth
    ST.motor(1, st_val);
  }
  printMotorInfo();                                                           // display data
}

///////////////****************************************************//////////////////
void getMotorData()  {                                       // calculate speed
  static long count_last = 0;                                                    // last count
  //Calculating the speed using encoder count
  speed_act = ((count - count_last)*(60*(1000/LOOPTIME)))/(30);           
  count_last = count;                                           //setting count value to last count
}

int updatePid(int command, int targetValue, int currentValue)   {      // compute PWM value
  float pidTerm = 0;                                                           // PID correction
  int error=0;                                  
  static int last_error=0;                             
  error = abs(targetValue) - abs(currentValue); 
  pidTerm = (Kp * error) + (Kd * (error - last_error));                            
  last_error = error;
  return constrain(command + int(pidTerm), -127, 127);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void printMotorInfo()  {                                                     // display data
  if((millis()-lastMilliPrint) >= 150)   
  {                     
    lastMilliPrint = millis();
    int st_value = st_val;
    st_value = map(st_value, 0, 255, 0, 100);
    Serial.print("SP:");                Serial.print(speed_req);  
    Serial.print("  RPM:");           Serial.print(speed_act);
    Serial.print("  PWM:");          Serial.print(st_val);   
    Serial.print("  enc_count:");   Serial.print(count);
    Serial.print("  kp:");              Serial.print(Kp);
    Serial.print("  kd:");              Serial.println(Kd);            
}}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void read_encoder()  {                                    //Read Encoder
  if (digitalRead(encodPinB1)==HIGH)    
  count--;                // if encoder pin 2 = HIGH then count --
  else                     
  count++;                // if encoder pin 2 = LOW then count ++
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

int getParam()  
{
char param, cmd;
  if(!Serial.available())    return 0;
  delay(10);                  
  param = Serial.read();                              // get parameter byte
  if(!Serial.available())    return 0;
  cmd = Serial.read();                                // get command byte
  Serial.flush();                                     // clean serial buffer
  
switch (param) 
  {  
    case 's':                                         // adjust speed more
    if(cmd=='+')  
      {
        speed_req += 100;
        if(speed_req>400)   speed_req=400;
      }
      if(cmd=='-')    
      {
        speed_req -= 100;
        if(speed_req<0)   speed_req=0;
      }
      break;


/////////////////////////////////////////////  
    case 'r':                                         // adjust speed slowly 
    if(cmd=='+')  
      {
        speed_req += 20;
        if(speed_req>400)   speed_req=400;
      }
    if(cmd=='-')    
      {
        speed_req -= 20;
        if(speed_req<0)   speed_req=0;
      }
      break;
  ////////////////////////////////////////////    
    case 'a':                                        // adjust direction
    if(cmd=='+')
      {
//        digitalWrite(InA1, LOW);
//        digitalWrite(InB1, HIGH);
      }
    if(cmd=='-')   
      {
//        digitalWrite(InA1, HIGH);
//        digitalWrite(InB1, LOW);
      }
    break;
  ////////////////////////////////////////////   
    case 'o':                                        // type "oo" to stop motor
    ST.motor(1, 0);
    speed_req = 0;
    break;
  ////////////////////////////////////////////
    case 'p':                                       //adjust proportional gain
    if(cmd=='+')
      {
       Kp += 0.1;
      }
    if(cmd=='-')   {
       Kp -= 0.1;
      }
    break;
  ///////////////////////////////////////////
    case 'd':                                     //adjust derivative gain
    if(cmd=='+')
      {
       Kd += 0.1;
      }
    if(cmd=='-')   
      {
       Kd -= 0.1;
      }
    break;
  ///////////////////////////////////////////
    default: 
      Serial.println("ERROR");
    }}
/////////////////////////////////////////////////THE END//////////////////////////////////////////////

