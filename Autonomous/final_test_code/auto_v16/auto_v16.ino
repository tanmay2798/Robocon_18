/*
    Y               0_________70
    ^               |__LSA_0__|
    |                   /\
    |                  /  \
    |                 /    \
    |                /      \
    |             - / wheel1 \ +
    |   70 __      /          \      __ 70
    |     |L |    /            \    | L|
    |     |S |   /              \   | S|
    |     |A |  /                \  | A|
    |     |2_| /                  \ |_3|
    |    0    /                    \    0
    |       +/w                    w\-
    |       /  h                  h  \
    |      /     e               e    \                                  
    |     /       e             e      \
    |    /         l           l        \
    |   /___________3_________2__________\
    |               -         +
    |               0_________70
    |               |__LSA_1__|
    |___________________________________________________________>X
*/
#include <Kangaroo.h>
#include <Sabertooth.h>
#include <Encoder.h>
#include <digitalWriteFast.h>

////////////////// Connections & Pins //////////////////////////////////
#define KANGAROO_12_SERIAL Serial1
#define KANGAROO_3_SERIAL Serial2

Encoder yEnc(2, 3);
Encoder xEnc(35, 37);

int L_01 = 1, L_23 = 1;                                  // LSA Directions: +1 or -1 
int W_1 = -1, W_2 = -1, W_3 = 1;                          // Wheel Directions: +1 or -1

int LSA_0 = A2, LSA_1 = A1, LSA_2 = A0, LSA_3 = A3;       // LSA Pins
int LSA_0_i = 53, LSA_1_i = 51, LSA_2_i = 49, LSA_3_i = 47;       // LSA Interrupt Pins

int sol_pin = 30, mag_pin = 32,ls_sol = 23, ls_mag = 25;  // TZ1 TZ2
int sol_pin_r = 28, mag_pin_r = 34, ls_sol_r = 29, ls_mag_r = 27;   // TZ3

int LS_1 = 33, LS_2 = 31, LS_3 = 22, LS_4 = 24;       // Drive Limit Switcher   // Check


int LED_manual = 10, LED_transfer = 6, LED_shoot = 5;
/////////////// Kangaroo ////////////////////////////////
boolean kang_enable = true;
KangarooSerial K1(Serial1);
KangarooSerial K2(Serial2);
KangarooChannel k2(K1, '1');
KangarooChannel k3(K1, '2');
KangarooChannel k1(K2, '2');

////////////////// PID Variables ////////////////////////
float Kp1_x = 0.40, Kd1_x = 0.25;
float Kp2_x = 0.2, Kd2_x = 0.15;
float Kp1_y = 0.40, Kd1_y = 0.15;
float Kp2_y = 0.24, Kd2_y = 0.2;
float Kp_s = 0.05, Kd_s = 0.03;


float Kp1_curr, Kd1_curr;
float Kp2_curr, Kd2_curr;

float lin_diff = 0, lin_pdiff = 0;
float ang_diff = 0, ang_pdiff = 0;
float diff, pdiff;
float ang_control, lin_control;
float lin_differential, ang_differential;

//    acceleration = -40;
//    min_base = -1450;
//    max_base = -300;
//    base = 0;
////////////////// Motion ///////////////////////////////
///////////////////                   0      1       2       3     4     5       6      7      8    9     10    11     12     13     14     15      16
long min_base_val[17] =           {   0,     0,      0,      0,    0,    0,  -1450,   -840,    0,   0,  -100,    0,  -700,     0,     0,   -1200,   0};
long max_base_val[17] =           { 200,  1700,   1000,      0,  100,    0,      0,     0,     0,   0,     0,    0,     0,     0,     0,     0,     0};
long acceleration_val[17] =       {   3,     4,     10,      0,    50,    0,    -6,    10,    12,   0,     -50,    0,    -6,     0,     0,     -6,    0};
long deacceleration_val[17] =     {   0,   -14,    -10,      0,    0,     0,    8,   -10,   -10,   0,      0,    0,     0,     0,     0,    6,      0};
int dis_2_slow[17] =              {   0, 15000,   6000,      0,    0,    0,   4000,  6000,  6000,   0,     0,    0,  3000,     0,     0, 16000,     0};
int time_2_slow[17] =             {   0,  3500,   1000,      0,    0,    0,    1000,  1500,  1000,   0,    0,    0,   800,     0,     0,  2300,     0};

long min_base = 0, max_base = 0, acceleration = 0, deceleration = 0, base = 0;
int v1, v2, v3;

////////////////// Misc /////////////////////////////////
int flag = 0, state = 0, valread_y = 0, junc_R = 0, detect = 2, junc_time_cnt = 0, neg_vel = 0;
long int posx = 0, velx  = 0, posy = 0, vely  = 0, prev = 0, junc_pos_x = 0, junc_pos_y = 0;
int val = 0, val1 = 0, val2 = 0, pval1 = 0, pval2 = 0, ang = 0, i = 0;

/////////////////////////// States //////////////////////
long state_time_cnt = 0;
bool junc = false, shaft = true, final_align_pos = false, dummy_state = false;
int start_read_1 = 1;
boolean state_3_flag, state_9_flag;

//////////////////////////// LSA /////////////////////////
int lsa_star_val = 850;
int target = 490;

/////////////////////////// Ultrasonic //////////////////
float duration, cm;
int US_manual = 42, US_manual_reverse = 99, US_ball_detect = 46;
int manual_dis_min = 5, manual_dis_max = 65, manual_reverse_dis_min = 20, manual_reverse_dis_max = 70, ball_dis_min = 10, ball_dis_max = 16;

/////////////////////////// Throw ///////////////////////
boolean throw_done_tz1 = false, throw_done_tz2 = false, throw_done_tz3 = false;
boolean manual_detect = false, load_position = false, ball_detect = false, manual_reverse_detect = false;
long x_finalpos_tz1, x_finalpos_tz2, x_finalpos_tz3;
int no_of_throws = 0;

///////////////////////////// Small PID ////////////////////////
int other_lsa_a = 0, other_lsa_perror = 0, other_lsa_error = 0, other_lsa_diff = 0;
float other_control = 0;

boolean debug_enable = true;
int time_2_align = 1000; 
int time_cnt = 0;
int time_2_transfer = 10000;
int omega = 200;
int junc_counter;
int time_2_detach = 2000;
int TZ1_dis_2_move = 2000, TZ2_dis_2_move = 500, dis_margin =50, encoder_control = 0;

void junc_move_x();
void junc_move_y();
void state_change();
void junction();
void junction_R();
void stop_bot();

void setup()
{
  pinMode(13,OUTPUT);       // Debug
  pin_init();
  delay(2500);
  Kangaroo_Init();

  pval1 = analogRead(LSA_2);
  pval2 = analogRead(LSA_3);
  
  if (debug_enable){
    Serial.begin(19200);
    Serial.println("1");
  }

  /// Set for State 0 ////
  shaft = true;
  reset_LSA();
  state=0;
  set_K();
  max_base = max_base_val[state];
  min_base = min_base_val[state];
  base = 0;
  acceleration = acceleration_val[state];
  junc = false;
  state_time_cnt = millis();
}

void loop()
{
  base = constrain(base + acceleration, min_base, max_base);
  update_PID();
  if (state == 0){
    if (!junc){
      update_vel();         
      write_vel();
    }else{
      junc_align();         // short loop
      state_change();
    }
  }
  else if(state == 1){
    if (!junc){
      if (val2 < lsa_star_val && start_read_1 == 0){
        start_read_1 = 1;
        acceleration = 15;
        set_K();
        pval2 = analogRead(LSA_1);
      }
      if ((absolute(yEnc.read() - junc_pos_y) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state]))         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        min_base = 625;
        max_base = 1650;
      }
      update_vel();
      write_vel();
    }else{
      junc_align();
      state_change();
    }
  }
  else if(state == 2){
    if (!junc){
      if (((absolute(xEnc.read() - junc_pos_x) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state])))         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        min_base = 300;
      }
      else if (absolute(xEnc.read() - junc_pos_x) > 1500){
        acceleration = 0;
      }
      update_vel();
      write_vel();
    }else{
      state_change();
    }
  }
  else if(state == 3){
    if (state_3_flag == 0){
      if (millis()- state_time_cnt < time_2_align){
        update_vel();
        write_vel();
      }else{
        junc_move_x(170);
        while(absolute(xEnc.read() - junc_pos_x) <  TZ1_dis_2_move){
        }
        stop_bot();
        state_3_flag = 1;
        time_cnt = millis();
      }
    }else{      
        x_finalpos_tz1 = junc_pos_x + TZ1_dis_2_move;
        state_change();
    }
  }
  else if(state == 4){
  ///////////////////// ALIGN WITH WALL ////////////////////////////////////////////
    if ((digitalRead(LS_2) == LOW) && (digitalRead(LS_1) == LOW))
    {
      junc_move_y(base);
      write_vel();
    }
    else if ((digitalRead(LS_2) == HIGH) && (digitalRead(LS_1) == LOW))
    {
      v1 = 0;
      v2 = -base / 5;
      v3 = -base / 5;
      write_vel();
    }
    else if ((digitalRead(LS_2) == LOW) && (digitalRead(LS_1) == HIGH))
    {
      v1 = 0;
      v2 = +base / 5;
      v3 = +base / 5;
      write_vel();
    }
    else if (absolute(xEnc.read() - x_finalpos_tz1) > dis_margin)
    {
      while(absolute(xEnc.read() - x_finalpos_tz1) > dis_margin){
        encoder_control = (x_finalpos_tz1 - xEnc.read())/8;
        junc_move_x(encoder_control);
      }
      stop_bot();
    }
    ////////////////// THROW WHEN READY ////////////////////////////////////
    else
    {
      stop_bot();          
      if(check_manual())                           // Uncomment when detecting manual
      {
        digitalWrite(sol_pin,HIGH);
        digitalWrite(mag_pin,HIGH);
        while(digitalRead(ls_mag)==LOW){}
        delay(500);
        digitalWrite(sol_pin,LOW);
        while(digitalRead(ls_sol)==LOW){}
        delay(10000);
        while(check_ball_transfer==false){
          delay(1000);}
        delay(10000);
        digitalWrite(mag_pin,LOW);
        throw_done_tz1=true;
        delay(10000);
      }
      else if(throw_done_tz1)
      {
        state_change();
      }
//      stop_bot();          
//      check_manual();                           // Uncomment when detecting manual
//      if (manual_detect){
//        if(!load_position){
//          if (debug_enable)
//            Serial.println("Start_throw");            
//          load_ball();                      // Time to transfer/ Transfer Detect
//          load_position = true;
//        }
//        if (load_position){
//          while(ball_detect==false)
//          {
//            check_ball_transfer(); 
//          }
//          if (debug_enable)
//            {
//              Serial.println("ball_detection : ");
//              Serial.println(ball_detect);
//            }
//          if (ball_detect){
//            shoot_ball();
//            throw_done_tz1 = true;
//            load_position = false;
//            ball_detect = false;
//            digitalWrite(LED_transfer, LOW);
//            if (debug_enable)
//            {
//              Serial.println("Throw done");
//            }
//          }
//        }
//      }
//      else{
//        if (throw_done_tz1){
//          state_change();
//        }
//      }
//    }
  }
  }
  else if(state == 5){
    junc_align();
    state_change();
  }
  else if(state == 6){
    if (!junc){
      if (((absolute(xEnc.read() - junc_pos_x) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state])))         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        max_base = -300;
      }
      update_vel();
      write_vel(); 
    }else{
      junc_align();
      state_change();
    }
  }
  else if(state == 7){
    if (!junc){
      if ((absolute(yEnc.read() - junc_pos_y) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state]))         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        min_base = 500;
      }
      update_vel();
      write_vel();
    }else{
      junc_align();
      state_change();
    }
  }
  else if(state == 8){
    if (!junc){
      if (((absolute(xEnc.read() - junc_pos_x) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state])))         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        min_base = 300;
      }
      else if (absolute(xEnc.read() - junc_pos_x) > 1500){
        acceleration = 0;
      }
      update_vel();
      write_vel();
    }else{
      state_change();
    }
  }
  else if(state == 9){
    if (state_9_flag == 0){
      if (millis()- state_time_cnt < time_2_align){
        update_vel();
        write_vel();
      }else{
       junc_move_x(170);
        while(absolute(xEnc.read() - junc_pos_x) <  TZ1_dis_2_move){
        }
        stop_bot();
        delay(100);
        state_9_flag = 1;
        time_cnt = millis();
      }
    }else{
//      if (millis()- time_cnt < time_2_align){
//        update_vel();
//        write_vel();
//      }else{
        x_finalpos_tz2 = junc_pos_x + TZ2_dis_2_move;
        state_change();
      }
    
  }
  else if(state == 10){
    ///////////////////// ALIGN WITH WALL ////////////////////////////////////////////
    if ((digitalRead(LS_4) == LOW) && (digitalRead(LS_3) == LOW))
    {
      junc_move_y(base);
      write_vel();
    }
    else if ((digitalRead(LS_4) == HIGH) && (digitalRead(LS_3) == LOW))
    {
      v1 = 0;
      v2 = -base / 5;
      v3 = -base / 5;
      write_vel();
    }
    else if ((digitalRead(LS_4) == LOW) && (digitalRead(LS_3) == HIGH))
    {
      v1 = 0;
      v2 = +base / 5;
      v3 = +base / 5;
      write_vel();
    }
     else if (absolute(xEnc.read() - x_finalpos_tz2) > dis_margin)
    {
        encoder_control = (x_finalpos_tz2 - xEnc.read())/8;
        junc_move_x(encoder_control);
    }
    //////////////////// THROW WHEN READY //////////////////////////////////////////////
    else
    {
      stop_bot();          
      if(check_manual())                           // Uncomment when detecting manual
      {
        digitalWrite(sol_pin,HIGH);
        digitalWrite(mag_pin,HIGH);
        while(digitalRead(ls_mag)==LOW){}
        delay(500);
        digitalWrite(sol_pin,LOW);
        while(digitalRead(ls_sol)==LOW){}
        delay(10000);
        while(check_ball_transfer==false){
          delay(1000);}
        delay(10000);
        digitalWrite(mag_pin,LOW);
        throw_done_tz2=true;
        delay(10000);
      }
      else if(throw_done_tz2)
      {
        state_change();
      }
//      if (manual_detect){
//        if(!load_position){
//          if (debug_enable)
//          Serial.println("Start_throw");            
//          load_ball();                      // Time to transfer/ Transfer Detect
//          load_position = true;
//        }
//        if (load_position){
//          while(ball_detect==false)
//          {
//            check_ball_transfer(); 
//          }
//          if (debug_enable)
//            {
//              Serial.println("ball_detection : ");
//              Serial.println(ball_detect);
//            }
//          if (ball_detect){
//            shoot_ball();
//            throw_done_tz2 = true;
//            load_position = false;
//            ball_detect = false;
//            digitalWrite(LED_transfer, LOW);
//            if (debug_enable)
//            {
//              Serial.println("Throw done");
//            }
//          }
//        }
//      }
//      else{
//        if (throw_done_tz2){
//          unload_ball();
//          load_position = false;
//          state_change();
//        }
//      }
    }
  }
  else if(state == 11){
    junc_align();
    state_change();
  }
  else if(state == 12){
    if (((absolute(xEnc.read() - junc_pos_x) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state])))         // Slow Distance/Time
    {
      state_change();
    }
    update_vel();
    write_vel(); 
  }
  else if(state == 13){          // rotate bot
    rotate_bot();
    state_change();
  }
  else if(state == 14){          // Rack 2 transfer
//    while (!manual_reverse_detect)      // Uncomment when ultrasonic attached
//      {check_manual_reverse();}
//    while(manual_reverse_detect){
//      check_manual_reverse();
//    }
    delay(10000);
//    if (millis() - state_time_cnt < 5000)                  // Comment when ultrasonic attached
//    {
//      update_vel();
//      write_vel();
//    }
    delay(time_2_detach);
    state_change();
  }
  else if(state == 15){
    if (!junc){
      if (((absolute(xEnc.read() - junc_pos_x) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state])))         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        max_base = -400;
      }
      update_vel();
      write_vel();
    }else{
      if (junc_counter < 5){
        junc_counter++;
        junc=false;
      }else{
        junc_align();
        state_change();
      }
    }
  }
  else if(state == 16){              // Golden Ball Throw
    if ((lin_diff > 50) || (ang_diff > 50))
    {
    update_vel();
    write_vel();    // Debug
    }
    else if (analogRead(LSA_0) > 600|| analogRead(LSA_0) < 400)
    {
      while(analogRead(LSA_0) > 600|| analogRead(LSA_0) < 400)
      {
        small_PID(LSA_0);
        junc_move_x(other_control);
      }
      stop_bot();
    }
    else         // Slow Distance/Time
    { 
      digitalWrite(13,HIGH); 
      stop_bot();           
//      check_manual(); // Uncomment when detecting manual
      if (debug_enable)
      Serial.println("Start_throw");            
      load_gold_ball();
      gold_transfer();
      shoot_gold_ball();
      if (debug_enable)
      Serial.println("Throw done");
      digitalWrite(13,LOW);
      state_time_cnt = 0;                       // To re-align bot
    }
  }
  if(debug_enable){
    Serial.print(state);
    Serial.print("   ");
    Serial.println(junc);
  }
}
