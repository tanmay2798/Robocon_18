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
#include <SabertoothSimplified.h>

////////////////// Connections & Pins //////////////////////////////////
#define KANGAROO_12_SERIAL Serial1
#define KANGAROO_3_SERIAL Serial2

Encoder yEnc(2, 3);
Encoder xEnc(43, 45);

int retry_b1 = 3, retry_b2 = 4, retry_b3 = 5;

int L_01 = 1, L_23 = 1;                                  // LSA Directions: +1 or -1 
int W_1 = -1, W_2 = -1, W_3 = 1;                          // Wheel Directions: +1 or -1

//int LSA_0 = A0, LSA_1 = A2, LSA_2 = A4, LSA_3 = A1;       // LSA Pins
//int LSA_0_i = 40, LSA_1_i = 53, LSA_2_i = 49, LSA_3_i = 47;       // LSA Interrupt Pins

int LSA_0 = A0, LSA_1 = A1, LSA_2 = A2, LSA_3 = A3;       // LSA Pins
int LSA_0_i = 40, LSA_1_i = 47, LSA_2_i = 53, LSA_3_i = 51;       // LSA Interrupt Pins

int sol_pin = 26, mag_pin = 32,ls_sol = 24, ls_mag = 25;  // TZ1 TZ2
int sol_pin_r = 34, mag_pin_r = 30, ls_sol_r = 35, ls_mag_r = 29;   // TZ3

int LS_1 = 37, LS_2 = 33, LS_3 = 31, LS_4 = 23;       // Drive Limit Switcher   // Check

int LED_manual = 8, LED_transfer = 9, LED_shoot = 10, LED_shoot_2 = 11;
int LED_junc = 6, LED_rs2 = 10, LED_rs3 = 11;

int limit_golden = 22;
/////////////// Debug Enable ////////////////////////////
boolean debug_enable = true;

/////////////// Kangaroo ////////////////////////////////
boolean kang_enable = true;

KangarooSerial K1(Serial1);
KangarooSerial K2(Serial2);
KangarooChannel k2(K1, '1');
KangarooChannel k3(K1, '2');
KangarooChannel k1(K2, '2');

/////////////// Sabertooth //////////////////////////////
Sabertooth golden_rack(128, Serial3); // Use SWSerial as the serial port.

////////////////// PID Variables ////////////////////////
float Kp1_x = 0.40, Kd1_x = 0.25;
float Kp2_x = 0.2, Kd2_x = 0.15;
float Kp1_y = 0.40, Kd1_y = 0.15;
float Kp2_y = 0.24, Kd2_y = 0.2;
float Kp_s = 0.05, Kd_s = 0.03;
float Kp1_align = 0.2, Kd1_align = 0.12;
float Kp2_align = 0.1, Kd2_align = 0.07;

float Kp1_curr, Kd1_curr;
float Kp2_curr, Kd2_curr;

float lin_diff = 0, lin_pdiff = 0;
float ang_diff = 0, ang_pdiff = 0;
float diff, pdiff;
float ang_control, lin_control;
float lin_differential, ang_differential;
boolean align_state = false;

//    acceleration = -40;
//    min_base = -1450;
//    max_base = -300;
//    base = 0;
////////////////// Motion ///////////////////////////////
///////////////////                   0      1       2       3     4     5       6      7      8    9     10    11     12     13     14     15      16
long min_base_val[40] =           {   0,     0,      0,      0,    0,    0,  -1450,   -840,    0,   0,  -300,    0,  -700,     0,     0, -1800,     0};
long max_base_val[40] =           { 500,  2000,   1000,      0,  300,    0,      0,   1000, 1450,   0,     0,    0,     0,     0,     0,   400,   300};
long acceleration_val[40] =       {   4,     4,     10,      0,   15,    0,    -11,    10,    11,   0,   -15,    0,    -6,     0,     0,    -8,    15};
long deacceleration_val[40] =     {   0,   -14,    -10,      0,    0,     0,    10,   -10,   -10,   0,     0,    0,     0,     0,     0,     9,     0};
int dis_2_slow[40] =              {   0, 15000,   6000,      0,    0,    0,   4000,  6000,  6000,   0,     0,    0,  3000,     0,     0, 16000,     0};
int time_2_slow[40] =             {   0,  2900,   1300,      0,    0,    0,    1000,  1500,  1000,   0,    0,    0,  1000,     0,     0,  2300,     0};

long min_base = 0, max_base = 0, acceleration = 0, deceleration = 0, base = 0;
int v1, v2, v3;

////////////////// Misc /////////////////////////////////
int flag = 0, state = 0, valread_y = 0, junc_R = 0, detect = 2, junc_time_cnt = 0, neg_vel = 0;
long int posx = 0, velx  = 0, posy = 0, vely  = 0, prev = 0, junc_pos_x = 0, junc_pos_y = 0;
int val = 0, val1 = 0, val2 = 0, pval1 = 0, pval2 = 0, ang = 0, i = 0;

/////////////////////////// States //////////////////////
long state_time_cnt = 0;
bool junc = false, shaft = true, final_align_pos = false, dummy_state = false;
int start_read_0 = 1, start_read_1 = 1;
boolean state_3_flag, state_9_flag;
int retry_state = 0; 

//////////////////////////// LSA /////////////////////////
int lsa_star_val = 800;
int target = 335;

/////////////////////////// Ultrasonic //////////////////
float duration, cm;
int US_manual = 36, manual_reverse_detect_pin = 36, US_ball_detect = 46, US_manual_reverse_trig = 48,  US_manual_reverse_echo = 48;
int manual_dis_min = 5, manual_dis_max = 100, manual_reverse_dis_min_1 = 1, manual_reverse_dis_max_1 = 50, ball_dis_min = 7, ball_dis_max = 22;
int manual_reverse_dis_min, manual_reverse_dis_max, manual_reverse_dis_min_2= 1, manual_reverse_dis_max_2 = 50;
/////////////////////////// Throw //////////////////////
boolean throw_done_tz1 = false, throw_done_tz2 = false, throw_done_tz3 = false;
boolean manual_detect = false, load_position = false, ball_detect = false, manual_reverse_detect = false;
long x_finalpos_tz1, x_finalpos_tz2, x_finalpos_tz3;
int no_of_throws = 0;
int golden_rack_w=40,adjust_rack_w = 40;

///////////////////////////// Small PID ////////////////////////
int other_lsa_a = 0, other_lsa_perror = 0, other_lsa_error = 0, other_lsa_diff = 0;
float other_control = 0;

int time_2_align = 1000; 
int time_cnt = 0;
int time_2_transfer = 3000;
int omega = 200;
int junc_counter;
int time_2_detach = 2000;
int TZ1_dis_2_move = 1700, TZ2_dis_2_move = 1100, dis_margin =200, encoder_control = 0;

void junc_move_x();
void junc_move_y();
void state_change();
void junction();
void junction_R();
void stop_bot();

void setup()
{
  pin_init();
  if (debug_enable){
    Serial.begin(19200);
    Serial.println(debug_enable);
  }
  
  delay(500);
  Kangaroo_Init();
  Sabertooth_Init();

  pval1 = analogRead(LSA_2);
  pval2 = analogRead(LSA_3);
  
  state=0;
  set_reset_state();      // retry
//  state = 12;
//  state_change();
  /// Set for State 0/30 ////
  shaft = true;
  reset_LSA();
  set_K();
  max_base = max_base_val[state];
  min_base = min_base_val[state];
  base = 0;
  acceleration = acceleration_val[state];
  digitalWrite(LED_junc, LOW);
  junc = false;
  state_time_cnt = millis();
  ///////////////// Debug //////////////////////
  if (debug_enable){
    Serial.println("Setup Completed");
    Serial.print("\t Retry State: ");
    Serial.println(retry_state);
  }
}

void loop()
{
//  Serial.print(xEnc.read());
//  Serial.print(" xenc");
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
      if (analogRead(LSA_1) < lsa_star_val && start_read_1 == 0){
        Serial.print(" val2 ");
        Serial.print(val2);
        Serial.print(" LSA1 ");
        Serial.println(analogRead(LSA_1));
        start_read_1 = 1;
        acceleration = 8;
        set_K();
        pval2 = analogRead(LSA_1);
        if (debug_enable){
          Serial.print("Speed Up");
        }
      }
      if ((absolute(yEnc.read() - junc_pos_y) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state]))         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        min_base = 900;
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
        min_base = 600;
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
    if (millis()- state_time_cnt < time_2_align){
        update_vel();
        write_vel();
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
        encoder_control = (x_finalpos_tz1 - xEnc.read())/5;
        Serial.print("enc control");
        Serial.print(encoder_control);
        junc_move_x(encoder_control);
        Serial.println("set encoder");
      }
      stop_bot();
    }
    ////////////////// THROW WHEN READY ////////////////////////////////////
    else
    {
      stop_bot();          
      check_manual();                           // Uncomment when detecting manual
      if (manual_detect){
        if(!load_position){
          if (debug_enable)
            Serial.println("Start_throw");            
          load_ball();                      // Time to transfer/ Transfer Detect
          load_position = true;
        }
        if (load_position){
          check_ball_transfer();
          if (debug_enable)
            {
              Serial.println("ball_detection : ");
              Serial.println(ball_detect);
            }
          if (ball_detect){
            shoot_ball();
            throw_done_tz1 = true;
            load_position = false;
            ball_detect = false;
//            digitalWrite(LED_transfer, LOW);
            if (debug_enable)
            {
              Serial.println("Throw done");
            }
          }
        }
      }
      else{
        if (throw_done_tz1){
          state_change();
        }
      }
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
        max_base = -600;
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
        min_base = 600;
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
        min_base = 600;
      }
      else if (absolute(xEnc.read() - junc_pos_x) > 1500){
        acceleration = 0;
      }
      update_vel();
      write_vel();
    }else{
      // Add junc align
      state_change();
    }
  }
  else if(state == 9){
    if (millis()- state_time_cnt < time_2_align){
        update_vel();
        write_vel();
      }
    else{
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
      while(absolute(xEnc.read() - x_finalpos_tz2) > dis_margin){
        Serial.print("enc read:");
        Serial.println(absolute(xEnc.read()));
        
        encoder_control = (x_finalpos_tz2 - xEnc.read())/5;
        junc_move_x(encoder_control);
      }
      stop_bot();
    }
    //////////////////// THROW WHEN READY //////////////////////////////////////////////
    else
    {
      stop_bot();          
      check_manual();                           // Uncomment when detecting manual
      if (manual_detect){
        if(!load_position){
          if (debug_enable)
          Serial.println("Start_throw");            
          load_ball();                      // Time to transfer/ Transfer Detect
          load_position = true;
        }
        if (load_position){
          check_ball_transfer();
          if (debug_enable)
            {
              Serial.println("ball_detection : ");
              Serial.println(ball_detect);
            }
          if (ball_detect){
            shoot_ball();
            throw_done_tz2 = true;
            load_position = false;
            ball_detect = false;
//            digitalWrite(LED_transfer, LOW);
            if (debug_enable)
            {
              Serial.println("Throw done");
            }
          }
        }
      }
      else{
        if (throw_done_tz2){
          unload_ball();
          load_position = false;
          state_change();
        }
      }
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
//    delay(2000);
    align_state = true;
    set_K();
    adjust_rack();
    while (!manual_reverse_detect)      // Uncomment when ultrasonic attached
    {
      update_PID();
      update_vel();
      write_vel();
//      delay(10);
//      stop_bot();
      if (debug_enable){
        Serial.print("V1: ");
        Serial.print(v1);
        Serial.print("\tV2: ");
        Serial.print(v2);
        Serial.print("\tV3: ");
        Serial.print(v3);
        Serial.println("Waiting for Manual");
      }
      check_manual_reverse();
    }
    if (debug_enable){
      Serial.println("Manual Detected");
    }
    int time_stamp = millis();
    while(millis() - time_stamp < 5000)
    {
      update_PID();
      update_vel();
      write_vel();
    }
    
    while(manual_reverse_detect)
    {
      update_PID();
//      if(lin_diff > 20 || ang_diff > 20){
//        update_PID();
        update_vel();
        write_vel();
//      }
      if (debug_enable){
        Serial.println("waiting for Transfer");
      }
      check_manual_reverse();
    }
    align_state = false;
    set_K();
    if (debug_enable){
      Serial.println("Transfer Done");
    }
    delay(500);
//    adjust_rack();
//    digitalWrite(LED_shoot, HIGH);       // Remove Later
//    digitalWrite(LED_shoot, LOW);
//    if (millis() - state_time_cnt < 5000)                  // Comment when ultrasonic attached
//    {
//      update_vel();
//      write_vel();
//    }
//    delay(time_2_detach);
    state_change();
  }
  else if(state == 15){
    if (!junc){
      if (((absolute(xEnc.read() - junc_pos_x) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state])))         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        max_base = -700;
      }
      update_vel();
      write_vel();
    }else{
      if (junc_counter < 5){
        junc_counter++;
        junc=false;
      }else{
        junc_align();
        stop_bot();
        x_finalpos_tz3 = xEnc.read();
        state_change();
      }
    }
  }
  else if (state == 16){
//    align_state = true;
//    set_K();
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
    ////////////////// THROW WHEN READY ////////////////////////////////////
    else
    {
//      update_PID();
//      if (lin_diff > 20 || ang_diff > 20)
//      {
//        base = 0;
//        digitalWrite(LED_manual, HIGH);
//        while(lin_diff > 20 || ang_diff > 20){
//          update_PID();
//          update_vel();
//          write_vel();
//        }
//        digitalWrite(LED_manual, LOW);
//        stop_bot();
//        base = 300;
//      }
      stop_bot();
      if (debug_enable)
      {
        Serial.println("Start_throw");
      }           
      load_gold_ball();
      gold_transfer();
      shoot_gold_ball();
      if (debug_enable)
      Serial.println("Throw done");
      digitalWrite(13,LOW);
    }
  }
  else if (state == 21)
  {
    if (!junc){
      if (analogRead(LSA_1) < lsa_star_val && start_read_1 == 0){
        start_read_1 = 1;
        acceleration = 8;
        set_K();
        pval2 = analogRead(LSA_1);
      }
      if (millis() - state_time_cnt > time_2_slow[state])         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        min_base = 625;
        max_base = 1650;
      }
      update_vel();
      write_vel();
    }else{
      if (junc_counter < 1){
        junc_counter++;
        digitalWrite(LED_junc, LOW);
        junc = false;
      }else{
        junc_align();       // retry
        state_change();
      }
    }
  }
  else if (state == 30)
  {
    if (!junc){
      update_vel();         
      write_vel();
    }else{
      junc_align();         // retry
      state_change();
    }
  }
  else if (state == 31)
  {
    if (!junc){
      if (analogRead(LSA_0) < lsa_star_val && start_read_0 == 0){
        start_read_0 = 1;
        acceleration = -8;
        set_K();
        pval1 = analogRead(LSA_0);
      }
      if (millis() - state_time_cnt > time_2_slow[state])         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        min_base = -1650;
        max_base = -700;
      }
      update_vel();
      write_vel();
    }else{
      if (junc_counter < 1){
        junc_counter++;
        digitalWrite(LED_junc, LOW);
        junc = false;
      }else{
        junc_align();
        state_change();
      }
    }
  }
  else if (state == 32)         // retry
  {
    if (millis() - state_time_cnt < time_2_slow[state]+1000)
    {
      if (millis() - state_time_cnt > time_2_slow[state])         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
      }
      update_vel();
      write_vel();
    }
    else
    {
      state_change();
    }
  }
  ///////////////////// States End ////////////////////////////////////////
  if(debug_enable){
    Serial.print(state);
    Serial.print("   ");
    Serial.println(junc);
  }
}
