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
int LED_junc = 6, LED_rs2 = 7, LED_rs3 = 12;

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
long min_base_val[40] =           {   0,   300,      0,      0,    0,    0,      0,    400,     0,    0,  -600,    0,  -500,     0,     0,  -900,     0,    0,      0,   500};
long max_base_val[40] =           { 300,   300,    800,      0,  800,    0,      0,    800,  1450,    0,     0,    0,     0,     0,     0,     0,   300,    0,    900,     0};
long acceleration_val[40] =       {   4,     0,      8,      0,    8,    0,      0,      8,    11,    0,   -10,    0,    -6,     0,     0,    -3,    15,    0,      2,     4};
long deacceleration_val[40] =     {   0,   -11,    -10,      0,  -10,    0,      0,    -10,   -10,    0,     2,    0,     0,     0,     0,     6,     0,    0,     -6,    -4};
int dis_2_slow[40] =              {   0, 15000,   6000,      0, 6000,    0,   4000,   4000,  6000,    0,     0,    0,  3000,     0,     0, 16000,     0,    0,  16000,     0};
int time_2_slow[40] =             {   0,  2900,   1300,      0, 1300,    0,   1000,   1200,  1000,    0,   600,    0,  1000,     0,     0,  4000,   600,    0,   3500,     0};

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
int debounceTime_g=2000;

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
  
  state=2;
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
//  state = 16;
////  retry_state = 2;
state_change();
}

////////////////////////////////////////////// LOOP /////////////////////////////////////
//char cmd;
//bool print_LSA = false;
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
      int tmp_LSA_1 = analogRead(LSA_1);
      if (tmp_LSA_1 < lsa_star_val && start_read_1 == 0){                     // Speed up when LSA_1 comes on line
        if (debug_enable){
          Serial.print("val2 ");
          Serial.print(val2);
          Serial.print("\tLSA1 ");
          Serial.print(tmp_LSA_1);
          Serial.println("Speed Up");
        }
        start_read_1 = 1;
        acceleration = 6;
        min_base = 400;       // 400
        max_base = 1600;
        set_K();
        pval2 = tmp_LSA_1;
      }
      if (millis() - state_time_cnt > time_2_slow[state])         // Slow After Time
      {
        acceleration = deacceleration_val[state];
        if (base > 700){
          min_base = 700;
        }else{
          min_base = base;
        }
        max_base = 1650;
      }
      update_vel();
      write_vel();
    }else{
      junc_align();
      state_change();
    }
  }
    else if (state == 2){
      rotate_bot();
      int time_stamp = millis();
      while(millis() - time_stamp < 3000){
        update_PID();
        update_vel();
        write_vel();
      }
      state_change();
    }
    else if (state == 3){
      stop_bot();
      if(!load_position && !throw_done_tz2){
        if (debug_enable)
        Serial.println("Start_throw");            
        load_ball();                      // Time to transfer/ Transfer Detect
        load_position = true;
      }    
      check_manual();                           // Uncomment when detecting manual
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
            throw_done_tz1 = true;
            load_position = false;
            ball_detect = false;
            if (debug_enable)
            {
              Serial.println("Throw done");
            }
          }
        }
      }
      if((manual_detect == false) && (throw_done_tz1 == true))
      {
      }
    }
    else if (state == 4){
      if (!junc){
        if (millis() - state_time_cnt > time_2_slow[state])         // Slow Distance/Time
        {
          acceleration = deacceleration_val[state];
          min_base = 300;
        }
        update_vel();
        write_vel();
      }else{
        junc_align(); // LSA_1
        stop_bot();
        pLSA_0_val = analogRead(LSA_0);
        state_change();
      }
    }
    else if (state == 5){
      
      int time_stamp = millis();
      while(millis() - time_stamp < 3000){
        update_PID();
        update_vel();
        write_vel();
      }
      rotate_bot();
      rotate_bot();
      state_change();
    }
    else if (state == 6){
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
    else if (state == 7){
      
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
          //state_change();
        }
      }
    }

  ///////////////////////////////////////////////////
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
    Serial.print("\tState");
    
    Serial.print(state);
    Serial.println(junc);
  }
}
