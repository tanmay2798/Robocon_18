float duration;
#include <Sabertooth.h>

/////////////// Sabertooth //////////////////////////////
Sabertooth golden_rack(128, Serial3); // Use SWSerial as the serial port.

int L_01 = 1, L_23 = 1;                                  // LSA Directions: +1 or -1 
int W_1 = -1, W_2 = -1, W_3 = 1;                          // Wheel Directions: +1 or -1

int sol_pin = 26, mag_pin = 32,ls_sol = 24, ls_mag = 25;  // TZ1 TZ2
int sol_pin_r = 34, mag_pin_r = 30, ls_sol_r = 35, ls_mag_r = 29; 
int gold_rack_LS = 23;
int LS_1 = 37, LS_2 = 33, LS_3 = 23, LS_4 = 31; 
int LS_5 = 27;


int LSA_0 = A0, LSA_1 = A1, LSA_2 = A2, LSA_3 = A3;       // LSA Pins
int LSA_0_i = 40, LSA_1_i = 47, LSA_2_i = 53, LSA_3_i = 51;       // LSA Interrupt Pins

int LED_manual = 10, LED_transfer = 6, LED_shoot = 5;

int limit_golden = 22;

char cmd;


int US_ball_detect = 46;
int US_manual_echo = 36, US_manual_trig = 46;
//int US_ball_detect_ping = 46;
//int US_manual_ping = 42;
int US_manual_reverse_trig = 48 , US_manual_reverse_echo = 48;

int golden_rack_w = 120;

boolean debug_enable = true;

int debounceTime=75;
boolean prevVal=false;
int lastDebounceTime=0;

void Sabertooth_Init()
{
  Serial3.begin(9600);            // Rack Motor
}


int throw_cnt_tz3=0;
void setup() {
  Sabertooth_Init();
  pin_init();
  Serial.begin(19200);
  pinMode(34, OUTPUT);
  pinMode(US_manual_trig, OUTPUT);
  pinMode(US_manual_echo, INPUT);
}

void loop() {
  cmd = ' ';
  if (Serial.available())
    cmd = Serial.read();
  if (cmd == 'q'){
    digitalWrite(sol_pin, HIGH);
  }else if (cmd == 'a'){
    digitalWrite(sol_pin, LOW);
  }else if (cmd == 'w'){
    digitalWrite(mag_pin, HIGH);
  }else if (cmd == 's'){
    digitalWrite(mag_pin, LOW);
  }else if (cmd == 'e'){
    digitalWrite(sol_pin_r, HIGH);
  }else if (cmd == 'd'){
    digitalWrite(sol_pin_r, LOW);
  }else if (cmd == 'r'){
    digitalWrite(mag_pin_r, HIGH);
  }else if (cmd == 'f'){
    digitalWrite(mag_pin_r, LOW);
  }else if (cmd == 'z'){
    golden_rack.motor(1,golden_rack_w);
    delay(100);
    golden_rack.motor(1,0);
  }else if (cmd == 'z'){
    golden_rack.motor(1,golden_rack_w);
    delay(100);
    golden_rack.motor(1,0);
  }else if (cmd == 'x'){
    golden_rack.motor(1,-golden_rack_w);
    delay(100);
    golden_rack.motor(1,0);
  }else if (cmd == 't'){
    digitalWrite(34, HIGH);
  }else if (cmd == 'g'){
    digitalWrite(34, LOW);
  }else if (cmd == 'p'){
    
  golden_rack.motor(2,0);
  }else if (cmd == ';'){
    throw_cnt_tz3 = 0;
    gold_2();
  }else if (cmd == '['){
    throw_cnt_tz3 = 1;
    gold_2();
  }else if (cmd == ']'){
    throw_cnt_tz3 = 2;
    gold_2();
  }
//  Serial.println(analogRead(A5));
//  digitalWrite(mag_pin, HIGH);
//  digitalWrite(sol_pin, HIGH);
//  Serial.print("Manua Distance: ");
//  Serial.println(manual_ultrasonic_read());
//  Serial.print(" Transfer Distance: ");
//  Serial.print(ball_detect_ultrasonic_read());
//  Serial.print(" Manual Reverse Distance: ");
//  Serial.print(manual_reverse_ultrasonic_read());
//  Serial.print(analogRead(A5));
//  Serial.print("    LSA_0: ");
//  Serial.print(analogRead(LSA_0));
//  delay(10);
//  Serial.print("    LSA_1: ");
//  Serial.print(analogRead(LSA_1));
//  Serial.print("    LSA_2: ");
//  Serial.print(analogRead(LSA_2));
//  Serial.print("    LSA_3: ");
//  Serial.println(analogRead(LSA_3));
//  Serial.print("    LSA_4: ");
//  Serial.println(analogRead(LSA_4));
//  delay(10);
//   Serial.print(" LSA_0_i: ");
//  Serial.print(digitalRead(LSA_0_i));
//  Serial.print(" LSA_1_i: ");
//  Serial.print(digitalRead(LSA_1_i));
//  Serial.print(" LSA_2_i: ");
//  Serial.print(digitalRead(LSA_2_i));
//  Serial.print(" LSA_3_i: ");
//  Serial.print(digitalRead(LSA_3_i));
//  Serial.print(" Sol_LS_1: ");
//  Serial.print(digitalRead(ls_sol));
//  Serial.print(" Mag_LS_1: ");
//  Serial.print(digitalRead(ls_mag));
//  Serial.print(" Sol_LS_2: ");
//  Serial.print(digitalRead(ls_sol_r));
//  Serial.print(" Mag_LS_2: ");
//  Serial.print(digitalRead(ls_mag_r));
//  Serial.print(" LS_1: ");
//  Serial.print(digitalRead(LS_1));
//  Serial.print(" LS_2: ");
//  Serial.print(digitalRead(LS_2));
//  Serial.print(" LS_3: ");
//  Serial.print(digitalRead(LS_3));
//  Serial.print(" LS_4: ");
//  Serial.print(digitalRead(LS_4));
//  Serial.print(" LS_5: ");
//  Serial.print(digitalRead(46));
//  Serial.print(" 46 and 48 ");
  Serial.println(digitalRead(limit_golden));
//Serial.println(digitalRead(36));
}

void pin_init(){
  pinMode(LSA_0_i, INPUT);
  pinMode(LSA_2_i, INPUT);
  pinMode(LSA_3_i, INPUT);
  
  pinMode(LS_1, INPUT);
  pinMode(LS_2, INPUT);
  pinMode(LS_3, INPUT);
  pinMode(LS_4, INPUT);
  pinMode(LS_5, INPUT);
  pinMode(gold_rack_LS, INPUT);
  
  pinMode(ls_mag, INPUT);
  pinMode(ls_sol, INPUT);
  pinMode(sol_pin, OUTPUT);
  pinMode(mag_pin, OUTPUT);
  
  pinMode(ls_mag_r, INPUT);
  pinMode(ls_sol_r, INPUT);
  pinMode(sol_pin_r, OUTPUT);
  pinMode(mag_pin_r, OUTPUT);

  pinMode(LED_manual, OUTPUT);
  pinMode(LED_transfer, OUTPUT);
  pinMode(LED_shoot, OUTPUT);

  digitalWrite(LED_manual,LOW);
  digitalWrite(LED_transfer,LOW);
  digitalWrite(LED_shoot,LOW);
  
  digitalWrite(sol_pin,LOW);
  digitalWrite(mag_pin,LOW);
  digitalWrite(sol_pin_r,LOW);
  digitalWrite(mag_pin_r,LOW);
  
  pinMode(US_manual_reverse_trig,OUTPUT);
  pinMode(US_manual_reverse_echo,INPUT);
}


//float ultra_dist(int ultrasonic_Pin)
//{
//  long a; 
//  duration = 0;
//  float d = 0.0 ;
//  pinModeFast(ultrasonic_Pin, OUTPUT);
//  delayMicroseconds(2);
//  digitalWriteFast(ultrasonic_Pin, HIGH);
//  delayMicroseconds(5);
//  digitalWriteFast(ultrasonic_Pin, LOW);
//  delayMicroseconds(2);
//  pinModeFast(ultrasonic_Pin, INPUT);
//  delayMicroseconds(2);
//  while (!digitalReadFast(ultrasonic_Pin));
//  a = micros();
//  while (digitalReadFast(ultrasonic_Pin) && (micros() - a) < 4500); // 5 millisecond timeout
//  duration = micros() - a;
//  d = duration / 58.2;
//  delayMicroseconds(200); // 16/4 -- for four sensors
//  return d;
//}

int ball_detect_ultrasonic_read()
{
  pinMode(US_ball_detect, OUTPUT);
  digitalWrite(US_ball_detect, LOW);
  delayMicroseconds(2);
  digitalWrite(US_ball_detect, HIGH);
  delay(5);
  digitalWrite(US_ball_detect, LOW);

  pinMode(US_ball_detect, INPUT);
  int duration = pulseIn(US_ball_detect, HIGH);
  int cm = microsecondsToCentimeters(duration); 
//  Serial.println(cm);
  return cm;
}

int manual_reverse_ultrasonic_read(){
  digitalWrite(US_manual_reverse_trig,LOW);
  delayMicroseconds(2);
  digitalWrite(US_manual_reverse_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_manual_reverse_trig,LOW);
  long duration = pulseIn(US_manual_reverse_echo,HIGH);
  int dis = duration/58.2;
  return dis;
}

int manual_ultrasonic_read()
{
//  pinMode(US_manual_trig, OUTPUT);
  digitalWrite(US_manual_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(US_manual_trig, HIGH);
  delay(10);
  digitalWrite(US_manual_trig, LOW);

//  pinMode(US_manual_echo, INPUT);
  int duration = pulseIn(US_manual_echo, HIGH);
  int cm = microsecondsToCentimeters(duration);
//  Serial.println(cm); 
  return cm;
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

void gold_transfer(){
  int gold_w = golden_rack_w;
  if (debug_enable){
    Serial.println("Transferring Gold");
  }
  golden_rack.motor(2,gold_w);
  delay(200);
  while (digitalRead(limit_golden) == LOW){
    delay(1);
  }
  golden_rack.motor(2,0);
  golden_rack.motor(2,gold_w);
  delay(300);
  golden_rack.motor(2,0);
  golden_rack.motor(2,-gold_w);
  while(true){
    while (digitalRead(limit_golden) == LOW){
    delay(1);
    }
    golden_rack.motor(2,0);
    delay(200);
    if (digitalRead(limit_golden) == HIGH){
      break;
    }else{
      gold_w = -gold_w/2;
      golden_rack.motor(2,gold_w);
      delay(3);
    }
  }
  if (debug_enable){
    Serial.println("Gold Transferred");
  }
}

void gold_3(int cnt){
  throw_cnt_tz3 = cnt;
  int gold_w = -golden_rack_w;

  golden_rack.motor(2,gold_w);
//  delay(400);
  for(int i = 0; i < throw_cnt_tz3+2; i++)
  {
    Serial.print(i);
    Serial.println(" high ");
    while (digitalRead(limit_golden) == HIGH)
    {
      delay(5);
    }
    Serial.print(i);
    Serial.println(" low ");
    while (digitalRead(limit_golden) == LOW)
    {
      delay(5);
    }
  }

//  delay(200);
  golden_rack.motor(2,0);
  Serial.print(throw_cnt_tz3+1);
  Serial.println("loaded");
  delay(2000);
//  delay(200);
  gold_w = -gold_w;
  golden_rack.motor(2,gold_w);
  while (digitalRead(limit_golden) == LOW)
  {
    delay(5);
  }
  for(int i = 0; i < throw_cnt_tz3+2; i++)
  {
    while (digitalRead(limit_golden) == HIGH)
    {
      delay(5);
    }
    while (digitalRead(limit_golden) == LOW)
    {
      delay(5);
    }
  }
  golden_rack.motor(2,0);
  delay(200);
  Serial.print(throw_cnt_tz3+1);
  Serial.println("returned");
  delay(2000);
  
  if (digitalRead(limit_golden) == LOW)
  {
    gold_w = -gold_w/2;
    golden_rack.motor(2,gold_w);
    
    while(true){
      while (digitalRead(limit_golden) == LOW){
      delay(1);
      }
      golden_rack.motor(2,0);
      delay(100);
      if (digitalRead(limit_golden) == HIGH){
        break;
      }else{
        gold_w = -gold_w/2;
        golden_rack.motor(2,gold_w);
      }
    }
  }
  Serial.println("original state achieved");
  throw_cnt_tz3 = (throw_cnt_tz3 + 1) % 4;
  Serial.println(throw_cnt_tz3);
//  delay(4000);
}

void gold_2(){
  int gold_w = -golden_rack_w;

  golden_rack.motor(2,gold_w/4);
//  while(1){
//    if (debounce(HIGH)){
//      golden_rack.motor(2,0);
//      break;
//    }
//  }
//  delay(400);
  
  golden_rack.motor(2,gold_w);
  for(int i = 0; i < throw_cnt_tz3+2; i++)
  {
    while (1)
    {
      if(debounce(LOW))
        break;
    }
    while (1)
    {
      if(debounce(HIGH))
        break;
    }
  }

//  delay(200);
  golden_rack.motor(2,0);
  delay(100);
  if (debug_enable){
  Serial.print(throw_cnt_tz3+1);
  Serial.println("loaded");
  }
//  delay(200);
  if (throw_cnt_tz3 != 3){
    gold_w = -gold_w;
    golden_rack.motor(2,gold_w);
    while (1)
    {
      if(debounce(HIGH))
        break;
    }
    for(int i = 0; i < throw_cnt_tz3+1; i++)
    {
      while (1)
    {
      if(debounce(LOW))
        break;
    }
      while (1)
    {
      if(debounce(HIGH))
        break;
    }
    }
  }
  golden_rack.motor(2,0);
  delay(100);
  if(debounce(LOW))
  {
    gold_w = -gold_w/2;
    golden_rack.motor(2,gold_w);
    
    while(true){
      while (digitalRead(limit_golden)==LOW){
      delay(1);
      }
      golden_rack.motor(2,0);
      delay(100);
      while (digitalRead(limit_golden)==HIGH);
      if (debounce(HIGH)){
        break;
      }else{
        gold_w = -gold_w/2;
        golden_rack.motor(2,gold_w);
      }
    }
  }
//  delay(4000);
}

boolean debounce(boolean test_type) {
  boolean condition=digitalRead(limit_golden);
  if (condition != prevVal)
  {
    lastDebounceTime = millis();
    while(millis()-lastDebounceTime < debounceTime)
    {
      condition==digitalRead(limit_golden);
      if (condition == prevVal)
      {
        return false;
      }
    }
    prevVal = condition; 
    if (condition == test_type)
    return true;
    else
    return false;
  }
  else 
  {
    return false;
  }
}

