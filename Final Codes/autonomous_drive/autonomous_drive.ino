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
#define SABERTOOTH_SERIAL Serial3

Encoder xEnc(43, 45);

int retry_b1 = 3, retry_b2 = 4, retry_b3 = 5;                           // Retry Buttons Pins

int L_01 = 1, L_23 = 1;                                                 // LSA Directions: +1 or -1 
int W_1 = -1, W_2 = -1, W_3 = 1;                                        // Wheel Directions: +1 or -1

int LSA_0 = A0, LSA_1 = A1, LSA_2 = A2, LSA_3 = A3;                     // LSA Pins
int LSA_0_i = 40, LSA_1_i = 47, LSA_2_i = 53, LSA_3_i = 51;             // LSA Interrupt Pins

int sol_pin = 26,  mag_pin = 32;          // Mech 1
int sol_pin_r = 34, mag_pin_r = 30;      // Mech 2
int ls_sol = 24, ls_mag = 25;            // LS Mech 1
int ls_sol_r = 35, ls_mag_r = 29;        // LS Mech 2
int limit_golden = 22;                   // LS Golden

int LS_1 = 37, LS_2 = 33, LS_3 = 31, LS_4 = 23;     // Drive Limit Switch

int LED_manual = 8, LED_transfer = 9, LED_shoot = 10, LED_shoot_2 = 11;
int LED_junc = 6, LED_rs2 = 10, LED_rs3 = 8;

int IR_manual = 36, IR_manual_reverse = 48, IR_ball_detect = 46;

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
float Kp2_align = 0.1, Kd2_align = 0.03;
float Kp1_y_rev = 0.55, Kd1_y_rev = 0.25;
float Kp2_y_rev = 0.25, Kd2_y_rev = 0.2;

float Kp1_curr, Kd1_curr;
float Kp2_curr, Kd2_curr;

float lin_diff = 0, lin_pdiff = 0;
float ang_diff = 0, ang_pdiff = 0;
float diff, pdiff;
float ang_control, lin_control;
float lin_differential, ang_differential;
int val = 0, val1 = 0, val2 = 0, pval1 = 0, pval2 = 0, ang = 0, i = 0;

////////////////// Motion ///////////////////////////////
///////////////////                   0      1       2       3     4     5       6       7      8     9     10    11     12     13     14     15     16    17      18     19
long min_base_val[40] =           {   0,     0,      0,      0,    0, -850,  -1450,   -840,     0,    0,  -600,    0,  -500,     0,     0, -1500,     0,    0,    400,   500};
long max_base_val[40] =           { 400,  2000,   1000,      0,  500, -100,      0,   1000,  1450,    0,     0,    0,     0,     0,     0,  -400,   300,    0,   1500,     0};
long acceleration_val[40] =       {   4,     4,     10,      0,   10,   -8,    -11,     10,    11,    0,   -10,    0,    -6,     0,     0,    -6,    15,    0,      6  ,     4};
long deacceleration_val[40] =     {   0,   -11,    -10,      0,   -2,   18,     10,    -10,   -10,    0,     2,    0,     0,     0,     0,     6,     0,    0,     -6,    -4};
int dis_2_slow[40] =              {   0, 15000,   6000,      0,    0,    0,   4000,   6000,  6000,    0,     0,    0,  3000,     0,     0, 16000,     0,    0,  16000,     0};
int time_2_slow[40] =             {   0,  2900,   1300,      0,  600,    0,   1000,   1200,  1000,    0,   600,    0,  1000,     0,     0,  2300,   600,    0,   2500,     0};

int omega = 200;

long min_base = 0, max_base = 0, acceleration = 0, deceleration = 0, base = 0;
int v1, v2, v3;

////////////////// Misc /////////////////////////////////

/////////////////////////// States //////////////////////
int state = 0;
long state_time_cnt = 0;
bool shaft = true, final_align_pos = false;
bool state_3_flag, state_9_flag;
int retry_state = 0;
boolean align_state = false;
boolean rotate = true;
bool set_rotate_pos = true;
int rotate_pos;
bool gamble_state = false;
bool skip_state = false;

//////////////////////////// LSA /////////////////////////
int lsa_star_val = 780;
int target = 335;

int junc_time_cnt = 0;
long int junc_pos_x = 0;
bool junc = false;
int start_read_0 = 1, start_read_1 = 1;
int junc_counter;

/////////////////////////// Throw //////////////////////
int golden_rack_w = 110, adjust_rack_w = 40;

boolean throw_done_tz1 = false, throw_done_tz2 = false, throw_done_tz3 = false;
boolean manual_detect = false, ball_detect = false, manual_reverse_detect = false;
boolean load_position = false;
long x_finalpos_tz1, x_finalpos_tz2, x_finalpos_tz3;
int throw_cnt_tz1 = 0, throw_cnt_tz2 = 0, throw_cnt_tz3 = 0;

///////////////////////////// Small PID ////////////////////////
int tz3_align_val = target;

int other_lsa_a = 0, other_lsa_perror = 0, other_lsa_error = 0, other_lsa_diff = 0;
float other_control = 0;
int lsa_margin = 150, lsa_control = 0, LSA_0_val = 0, pLSA_0_val;
int encoder_control = 0;

///////////////////////////// Time //////////////////////////////////////////
int time_2_align = 1000;
int time_2_transfer = 3000;
int time_2_detach = 1000;

int time_cnt = 0;
///////////////////////////// Distances /////////////////////////////////////
int TZ1_dis_2_move = 2600, TZ2_dis_2_move = 1400, TZ3_dis_2_move = -4500, dis_margin =250; 

///////////////////////////// Debounce ////////////////////////
int debounceTime=75;
boolean prevVal=false;
int lastDebounceTime=0;

////////////////////////////////////////////// SETUP /////////////////////////////////////

void setup()
{
  pin_init();
  if (debug_enable){
    Serial.begin(19200);
  }
  Kangaroo_Init();
  SABERTOOTH_SERIAL.begin(9600);            // Rack Motor

  pval1 = analogRead(LSA_2);
  pval2 = analogRead(LSA_3);
  
  state=0;
  set_retry_state();                        // retry
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
  junc_counter = 0;
  state_time_cnt = millis();
  ///////////////// Debug //////////////////////
  if (debug_enable){
    Serial.println("Setup Completed");
    Serial.print("\t Retry State: ");
    Serial.println(retry_state);
  }
//  state = 12;
////  retry_state = 2;
//  state_change();
}

////////////////////////////////////////////// LOOP /////////////////////////////////////

void loop()
{
  base = constrain(base + acceleration, min_base, max_base);
  update_PID();
  if (state == 0){
    if(millis() - state_time_cnt > 1800 && analogRead(LSA_0) < lsa_star_val)        // Time after which to read LSA_0
    {
      state_change();
    }
    else if (!junc){
      update_vel();         
      write_vel();
    }else{
      junc_align();
      state_change();
    }
  }
  else if (state == 1){
    if (!junc){
      if (analogRead(LSA_1) < lsa_star_val && start_read_1 == 0){                     // Speed up when LSA_1 comes on line
        if (debug_enable){
          Serial.print("val2 ");
          Serial.print(val2);
          Serial.print("\tLSA1 ");
          Serial.print(analogRead(LSA_1));
          Serial.println("Speed Up");
        }
        start_read_1 = 1;
        acceleration = 8;
        set_K();
        pval2 = analogRead(LSA_1);
      }
      if (millis() - state_time_cnt > time_2_slow[state])         // Slow After Time
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
    check_manual();
    if((manual_detect == false) && (throw_done_tz1 == true))
    {
      state_change(); 
    }
    ////////////// ALIGN WITH WALL ///////////////
    if ((digitalRead(LS_2) == LOW) && (digitalRead(LS_1) == LOW))
    {
       if (millis()-state_time_cnt > time_2_slow[state])
      {
        acceleration = deacceleration_val[state];
        min_base = 150;
      }
      junc_move_y(base);
      write_vel();
    }
    else if ((digitalRead(LS_2) == HIGH) && (digitalRead(LS_1) == LOW))
    {
      v1 = 0;
      v2 = -base / 3;
      v3 = -base / 3;
      write_vel();
    }
    else if ((digitalRead(LS_2) == LOW) && (digitalRead(LS_1) == HIGH))
    {
      v1 = 0;
      v2 = +base / 3;
      v3 = +base / 3;
      write_vel();
    }
    else if (absolute(xEnc.read() - x_finalpos_tz1) > dis_margin)
    {
      while(absolute(xEnc.read() - x_finalpos_tz1) > dis_margin){
        encoder_control = (x_finalpos_tz1 - xEnc.read())/7;
        junc_move_x(encoder_control);
      }
      stop_bot();
    }
    //////////////// THROW WHEN READY ///////////////////
    else
     {
      stop_bot();
      if(!load_position && !throw_done_tz1){
        if (debug_enable)
          Serial.println("Start_throw");            
        load_ball();                      // Time to transfer/ Transfer Detect
        load_position = true;
      }
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
            if(skip_state){
              state_change();
            }else{
              throw_done_tz1 = true;
              load_position = false;
              ball_detect = false;
              if (debug_enable)
              {
                Serial.println("Throw done");
              }
            }
          }         // Ball detect if
        }         // Load position if
      }         // Manual Detect if
    }         // Throw if
  }
  else if(state == 5){
    junc_move_y(-400);
    delay(200);
    stop_bot();
    base = -500;
    junc_move_x(base);
//    Serial.println("start");
    while(!junc && analogRead(LSA_3) > lsa_star_val){
      base = constrain(base + acceleration, min_base, max_base);
      junc_move_x(base);
      delay(1);
    }
    acceleration = deacceleration_val[state];
    while(analogRead(LSA_0) > lsa_star_val)
    {
      base = constrain(base + acceleration, min_base, max_base);
      junc_move_x(base);
      delay(1);
    }
    stop_bot();
    state_change();
  }
  else if(state == 6){
    if (!junc){
      if (millis() - state_time_cnt > time_2_slow[state])         // Slow Distance/Time
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
      if (millis() - state_time_cnt > time_2_slow[state])         // Slow Distance/Time
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
    check_manual();
      if((!manual_detect)&&(throw_done_tz2)&&(!throw_done_tz3))
      {
        if (load_position)
        {
          unload_ball();
          load_position = false;
        }
        state_change();
      }
    ///////////////////// ALIGN WITH WALL ////////////////////////////////////////////
    if ((digitalRead(LS_4) == LOW) && (digitalRead(LS_3) == LOW))
    {
      if (millis()-state_time_cnt > time_2_slow[state])
      {
        acceleration = deacceleration_val[state];
        max_base = -180;
      }
      junc_move_y(base);
      write_vel();
    }
    else if ((digitalRead(LS_4) == HIGH) && (digitalRead(LS_3) == LOW))
    {
      v1 = 0;
      v2 = -base / 3;
      v3 = -base / 3;
      write_vel();
    }
    else if ((digitalRead(LS_4) == LOW) && (digitalRead(LS_3) == HIGH))
    {
      v1 = 0;
      v2 = +base / 3;
      v3 = +base / 3;
      write_vel();
    }
     else if (absolute(xEnc.read() - x_finalpos_tz2) > dis_margin)
    {
      while(absolute(xEnc.read() - x_finalpos_tz2) > dis_margin){
        if (debug_enable){
        Serial.print("enc read:");
        Serial.println(absolute(x_finalpos_tz2 - xEnc.read()));
        }
        encoder_control = (x_finalpos_tz2 - xEnc.read())/4;
        junc_move_x(encoder_control);
      }
      stop_bot();
    }
    //////////////////// THROW WHEN READY //////////////////////////////////////////////
    else
    {
      stop_bot();
      if(!load_position && !throw_done_tz2){
        if (debug_enable)
        Serial.println("Start_throw");            
        load_ball();                      // Time to transfer/ Transfer Detect
        load_position = true;
      }    
//      check_manual();                           // Uncomment when detecting manual
      if (manual_detect || skip_state){
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
            skip_state = false;
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
    }
  }
  else if(state == 11){
    junc_align();
    state_change();
  }
  else if(state == 12){
    if (((absolute(xEnc.read() - junc_pos_x) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state]+1000)))         // Slow Distance/Time
    {
      base = 0;
      acceleration = 0;
      stop_bot();
      int time_stamp = millis();
      while(millis() - time_stamp < 2000){
        update_PID();
        update_vel();
        write_vel();
      }
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
//    adjust_rack();
//    int time_stamp = millis();
//    while(millis() - time_stamp < 1000){
//      update_PID();
//      update_vel();
//      write_vel();
//      delay(5);
//    }
//    stop_bot();
    while (!manual_reverse_detect)      // Uncomment when ultrasonic attached
    {
      update_PID();
      update_vel();
      write_vel();
      delay(5);
      if (debug_enable){
        Serial.println("Waiting for Manual");
      }
      check_manual_reverse();
    }
    stop_bot();
    delay(200);
    if (debug_enable){
      Serial.println("Manual Detected");
    }
    delay(4000);
//    time_stamp = millis();
//    while (millis() - time_stamp < 3000)
//    {
//      update_PID();
//      if (absolute(lin_diff) > 20 || absolute(ang_diff) > 20){
//        update_vel();
//        write_vel();
//      }else{
//        stop_bot();
//      }
//    }
    while(manual_reverse_detect)
    {
      stop_bot();
      if (debug_enable){
        Serial.println("waiting for Transfer");
      }
      check_manual_reverse();
    }
    if (debug_enable){
      Serial.println("Transfer Done");
    }
    int time_stamp = millis();
    update_PID();
    while (millis() - time_stamp < 2000)
    {
      update_PID();
      update_vel();
      write_vel();
      delay(5);
    }
    stop_bot();
    //shake_rack();
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
        pLSA_0_val = analogRead(LSA_0);
        state_change();
      }
    }
  }
  else if (state == 16)
  {
    load_initial_gold();
    digitalWrite(LED_manual, HIGH);
    ///////////////////// ALIGN WITH WALL ////////////////////////////////////////////
    if ((digitalRead(LS_2) == LOW) && (digitalRead(LS_1) == LOW))
    {
      if (millis()-state_time_cnt > time_2_slow[state])
      {
        acceleration = deacceleration_val[state];
        min_base = 150;
      }
      junc_move_y(base);
      write_vel();
      LSA_0_val = analogRead(LSA_0);
      if(LSA_0_val > lsa_star_val){
          LSA_0_val = pLSA_0_val;
      }
      pLSA_0_val = LSA_0_val;
    }
    else if ((digitalRead(LS_2) == HIGH) && (digitalRead(LS_1) == LOW))
    {
      v1 = 0;
      v2 = -base / 8;
      v3 = -base / 8;
      write_vel();
      LSA_0_val = analogRead(LSA_0);
      if(LSA_0_val > lsa_star_val){
          LSA_0_val = pLSA_0_val;
      }
      pLSA_0_val = LSA_0_val;
    }
    else if ((digitalRead(LS_2) == LOW) && (digitalRead(LS_1) == HIGH))
    {
      v1 = 0;
      v2 = +base / 8;
      v3 = +base / 8;
      write_vel();
      LSA_0_val = analogRead(LSA_0);
      if(LSA_0_val > lsa_star_val){
          LSA_0_val = pLSA_0_val;
      }
      pLSA_0_val = LSA_0_val;
    }
     else if (absolute(analogRead(LSA_0) - tz3_align_val) > lsa_margin)
    {
      int time_stamp = millis();
      while(absolute(analogRead(LSA_0) - tz3_align_val) > lsa_margin && millis() - time_stamp < 5000){
        delay(1);
        LSA_0_val = analogRead(LSA_0);
        if(LSA_0_val > lsa_star_val){
          LSA_0_val = pLSA_0_val;
        }
        pLSA_0_val = LSA_0_val;
        lsa_control = (tz3_align_val - LSA_0_val)/4;
        delay(1);
        junc_move_x(lsa_control);
      }
      stop_bot();
    }
    ////////////////// THROW WHEN READY ////////////////////////////////////
    else
    {
      digitalWrite(LED_manual, LOW);
      stop_bot();
      if (debug_enable)
      {
        Serial.println("Start_throw");
      }           
      load_gold_ball();
      gold_transfer();
      shoot_gold_ball();
      throw_cnt_tz3++;
      if (debug_enable)
      Serial.println("Throw done");
      digitalWrite(13,LOW);
      if(throw_cnt_tz3 > 3){
        throw_cnt_tz3 = 0;
        if (retry_state != 93){
          state_change();
        }
      }
    }
  }
  else if (state == 17)
  {
    junc_align();
    int time_stamp = millis();
    while(millis()-time_stamp < 1000){
      update_PID();
      update_vel();
      write_vel();
    }
    state_change();
  }
  else if(state == 18){
    if (!junc){
      if ((absolute(xEnc.read() - junc_pos_x) > dis_2_slow[state]) || (millis() - state_time_cnt > time_2_slow[state]))         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        max_base = 700;
      }
      update_vel();
      write_vel();
    }else{
      if (junc_counter < 4){
        junc_counter++;
        junc=false;
      }else{
        if (set_rotate_pos){
          rotate_pos = junc_pos_x + 13000;
          set_rotate_pos = false;
          
        }else{
          while(absolute(xEnc.read() - rotate_pos) > dis_margin){
            update_PID();
            update_vel();
            write_vel();
          }
  //        delay(100);
          stop_bot();
          int time_stamp = millis();
          base = 0;
          while(millis()-time_stamp < 2000){
            update_PID();
            update_vel();
            write_vel();
          }
          stop_bot();
          state_change();
        }
      }
    }
  }
  else if(state == 19)
  {
    if (rotate){
      rotate_bot();
      stop_bot();
      rotate = false;
      junc = false;
      delay(100);
    }
    if (!junc){
      update_PID();
      update_vel();
      write_vel();
    }
    else
    {
      stop_bot();
      state_change();
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
        min_base = 900;
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
  else if (state == 30)       // unused
  {
    if(millis() - state_time_cnt > 1800 && analogRead(LSA_1) < lsa_star_val)
    {
      state_change();
    }
    if (!junc){
      update_vel();         
      write_vel();
    }else{
      junc_align();         // retry
      state_change();
    }
  }
  else if (state == 31)     // unused
  {
    if (!junc){
      if (analogRead(LSA_0) < lsa_star_val && start_read_0 == 0){
        start_read_0 = 1;
        acceleration = -11;
        set_K();
        pval1 = analogRead(LSA_0);
//        Serial.print("Speed up");
      }
      if (millis() - state_time_cnt > time_2_slow[state])         // Slow Distance/Time
      {
        acceleration = deacceleration_val[state];
        min_base = -1650;
        max_base = -900;
        Serial.print("Slow down");
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
    if (absolute(xEnc.read() - junc_pos_x) > 8000)
    {
      acceleration = deacceleration_val[state];
    }
    if (absolute(xEnc.read() - junc_pos_x) > 13000)
    {
      stop_bot();
      delay(100);
      base = 0;
      int time_stamp = millis();
      while (millis() - time_stamp < 1000)
      {
        update_PID();
        update_vel();
        write_vel();
        delay(2);
      }
      stop_bot();
      rotate_bot();
      stop_bot();
      delay(100);
      base = 0;
      time_stamp = millis();
      while (millis() - time_stamp < 2000)
      {
        update_PID();
        update_vel();
        write_vel();
        delay(2);
      }
      stop_bot();
      state_change();
    }else{
      update_vel();
      write_vel();
    }
  }
  else if (state == 91)
  {
    stop_bot();
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
        digitalWrite(LED_transfer, LOW);
        if (debug_enable)
        {
          Serial.println("Throw done");
        }
      }
    }
  }
  else if (state == 93)
  {
      stop_bot();
      if (debug_enable)
      {
        Serial.println("Start_throw");
      }           
      load_gold_ball();
      gold_transfer();
      shoot_gold_ball();
      throw_cnt_tz3++;
      if (debug_enable)
      Serial.println("Throw done");
      digitalWrite(13,LOW);
      if (throw_cnt_tz3 > 3){
        throw_cnt_tz3 = 0;
      }
  }
  else if (state == 99){
    // LED_manual LED_transfer LED_shoot LED_shoot2 LED_junc LED_rs2 LED_rs3
    boolean r_b1 = digitalRead(retry_b1);
    boolean r_b2 = digitalRead(retry_b2);
    boolean r_b3 = digitalRead(retry_b3);
    if (r_b1 && r_b2 && !r_b3){
      golden_rack.motor(2,golden_rack_w);
    }else{
      golden_rack.motor(2,0);
    }
    //////// LS Drive //////////////
    if (digitalRead(LS_1) == HIGH || digitalRead(LS_2) == HIGH || digitalRead(LS_3) == HIGH || digitalRead(LS_4) == HIGH){
      digitalWrite(LED_manual, HIGH);
    }else{
      digitalWrite(LED_manual, LOW);      
    }
    
    //////// LS Mech 1 //////////////
    if (digitalRead(ls_mag) == HIGH){
      digitalWrite(LED_transfer, HIGH);
    }else{
      digitalWrite(LED_transfer, LOW);      
    }
    if (digitalRead(ls_sol) == HIGH){
      digitalWrite(LED_rs2, HIGH);
    }else{
      digitalWrite(LED_rs2, LOW);      
    }

    //////// LS Mech 2 //////////////
    if (digitalRead(ls_mag_r) == HIGH){
      digitalWrite(LED_shoot, HIGH);
    }else{
      digitalWrite(LED_shoot, LOW);      
    }
    if (digitalRead(ls_sol_r) == HIGH){
      digitalWrite(LED_rs3, HIGH);
    }else{
      digitalWrite(LED_rs3, LOW);      
    }

    //////// LS golden rack //////////////
    if (digitalRead(limit_golden) == HIGH){
      digitalWrite(LED_junc, HIGH);
    }else{
      digitalWrite(LED_junc, LOW);      
    }
    
    //////// IR Distance //////////////
    if (digitalRead(IR_manual) == LOW || digitalRead(IR_manual_reverse) == LOW || digitalRead(IR_ball_detect) == LOW){
      digitalWrite(LED_shoot_2, HIGH);
    }else{
      digitalWrite(LED_shoot_2, LOW);      
    }

    
  }
  ///////////////////// States End ////////////////////////////////////////
  if(debug_enable){
//    Serial.print("Base ");
//    Serial.print(base);
//    Serial.print("\tLSA0 ");
//    Serial.print(analogRead(LSA_0));
//    Serial.print("\tLSA1 ");
//    Serial.print(analogRead(LSA_1));
//    Serial.print("\tLSA2 ");
//    Serial.print(analogRead(LSA_2));
//    Serial.print("\tLSA3 ");
//    Serial.print(analogRead(LSA_3));
//
//    Serial.print("\tpval1 ");
//    Serial.print(pval1);
//    Serial.print("\tpval2 ");
//    Serial.print(pval2);
    Serial.print("\tState");
    Serial.print(state);
    Serial.println(junc);
  }
}
