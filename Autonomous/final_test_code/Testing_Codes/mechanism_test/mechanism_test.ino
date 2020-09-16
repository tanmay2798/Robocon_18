//#include <digitalWriteFast.h>
float duration;


int L_01 = 1, L_23 = 1;                                  // LSA Directions: +1 or -1 
int W_1 = -1, W_2 = -1, W_3 = 1;                          // Wheel Directions: +1 or -1

int LSA_0 = A2, LSA_1 = A1, LSA_2 = A0, LSA_3 = A3;       // LSA Pins
int LSA_0_i = 53, LSA_1_i = 51, LSA_2_i = 49, LSA_3_i = 47;       // LSA Interrupt Pins

int sol_pin = 30, mag_pin = 32,ls_sol = 23, ls_mag = 25;  // TZ1 TZ2
int sol_pin_r = 28, mag_pin_r = 26, ls_sol_r = 27, ls_mag_r = 29;   // TZ3

int LS_1 = 33, LS_2 = 31, LS_3 = 22, LS_4 = 24;       // Drive Limit Switcher   // Check


int LED_manual = 10, LED_transfer = 6, LED_shoot = 5;

char cmd;


int US_ball_detect = 46;
int US_manual = 42;
//int US_ball_detect_ping = 46;
//int US_manual_ping = 42;
int US_manual_reverse_trig = A5 , US_manual_reverse_echo = 41;
void setup() {
  pin_init();
  Serial.begin(19200);
}

void loop() {
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
  }
//  digitalWrite(mag_pin, HIGH);
//  digitalWrite(sol_pin, HIGH);
  Serial.print("Manua Distance: ");
  Serial.print(manual_ultrasonic_read());
  Serial.print(" Transfer Distance: ");
  Serial.print(ball_detect_ultrasonic_read());
  Serial.print(" Manual Reverse Distance: ");
  Serial.print(manual_reverse_ultrasonic_read());
  Serial.print(" Sol_LS_1: ");
  Serial.print(digitalRead(ls_sol));
  Serial.print(" Mag_LS_1: ");
  Serial.print(digitalRead(ls_mag));
  Serial.print(" Sol_LS_2: ");
  Serial.print(digitalRead(ls_sol_r));
  Serial.print(" Mag_LS_2: ");
  Serial.println(digitalRead(ls_mag_r));
}

void pin_init(){
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
  digitalWrite(US_manual_reverse_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_manual_reverse_trig,LOW);
  long duration = pulseIn(US_manual_reverse_echo,HIGH);
  int dis = duration/58.2;
  return dis;
}

int manual_ultrasonic_read()
{
  pinMode(US_manual, OUTPUT);
  digitalWrite(US_manual, LOW);
  delayMicroseconds(2);
  digitalWrite(US_manual, HIGH);
  delay(5);
  digitalWrite(US_manual, LOW);

  pinMode(US_manual, INPUT);
  int duration = pulseIn(US_manual, HIGH);
  int cm = microsecondsToCentimeters(duration);
//  Serial.println(cm); 
  return cm;
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

