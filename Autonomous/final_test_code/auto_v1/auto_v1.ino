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


////////////////// Connections & Pins //////////////////////////////////
#define KANGAROO_12_SERIAL Serial1
#define KANGAROO_3_SERIAL Serial2

Encoder yEnc(2, 3);
Encoder xEnc(18, 19);

int L_01 = 1, L_23 = 1;                                  // LSA Directions: +1 or -1 
int W_1 = -1, W_2 = -1, W_3 = 1;                          // Wheel Directions: +1 or -1

int LSA_0 = A2, LSA_1 = A1, LSA_2 = A0, LSA_3 = A3;       // LSA Pins
int LSA_0_i = 53, LSA_1_i = 99, LSA_2_i = 49, LSA_3_i = 51;       // LSA Interrupt Pins

int sol_pin = 28, mag_pin = 34,ls_sol = 27, ls_mag = 33;  // TZ1 TZ2
int sol_pin_r = 99, mag_pin_r = 99, ls_sol_r = 27, ls_mag_r = 33;   // TZ3

int LS_1 = 33, LS_2 = 31, LS_3 = 22, LS_4 = 24;       // Drive Limit Switcher   // Check

int ultrasonic_pin = 99;

/////////////// Kangaroo ////////////////////////////////
boolean kang_enable = true;
KangarooSerial K1(Serial1);
KangarooSerial K2(Serial2);
KangarooChannel k2(K1, '1');
KangarooChannel k3(K1, '2');
KangarooChannel k1(K2, '2');

////////////////// PID Variables ////////////////////////
float Kp1_x = -0.45, Kd1_x = -0.15;
float Kp2_x = 0.24, Kd2_x = 0.2;
float Kp1_y = -0.45, Kd1_y = -0.15;
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
///////////////////                0    1       2       3    4      5  6  7  8  9  10 11 12 13 14 15 16
long min_base_val[16] =           {0,   0,      0,      0,   0, -200, -1450, 0, 0, 0, 0, 0, 0, 0, 0, 0};
long max_base_val[16] =           {500, 1700,   1000,   0,  250,    0,  -300, 0, 0, 0, 0, 0, 0, 0, 0, 0};
long acceleration_val[16] =       {4,   2,      7,      3,  3,};
long deacceleration_val[16] =     {0,   -7,     -5,     0,  0,};
int dis_2_slow[16] = {0, 15000, 6000};
int time_2_slow[16] = {0, 1800, 1000};

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
boolean state_3_flag;

//////////////////////////// LSA /////////////////////////
int lsa_star_val = 850;
int target = 490;

/////////////////////////// Ultrasonic //////////////////
long duration, inches, cm;

/////////////////////////// Throw ///////////////////////
boolean throw_done_tz1 = false, throw_done_tz2 = false, throw_done_tz3 = false;
boolean manual_detect = false;
long x_finalpos_tz1, x_finalpos_tz2, x_finalpos_tz3;


boolean debug_enable = true;
int time_2_align = 500;
int time_cnt = 0;
int time_2_transfer = 10000;
int manual_dis_min = 0;
int manual_dis_max = 0;


void junc_move_x();
void junc_move_y();
void state_change();
void junction();
void junction_R();
void stop_bot();

void setup()
{
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
  shaft = false;
  reset_LSA();
  state=1;
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
  }else if(state == 1){
    if (!junc){
      if (val2 < lsa_star_val && start_read_1 == 0){
        start_read_1 = 1;
        acceleration = 7;
        set_K();
        pval2 = analogRead(LSA_3);
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
  }else if(state == 2){
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
  }else if(state == 3){
    if (state_3_flag == 0){
      if (millis()- state_time_cnt < time_2_align){
        update_vel();
        write_vel();
      }else{
        junc_move_x(-170);
        while (digitalRead(LSA_2_i) == LOW){
          delay(1);
        }
        stop_bot();
        delay(100);
        junc_move_x(70);
        while (digitalRead(LSA_2_i) == LOW){
          delay(1);
        }
        state_3_flag = 1;
        time_cnt = millis();
      }
    }else{
      if (millis()- time_cnt < time_2_align){
        update_vel();
        write_vel();
      }else{
        state_change();
      }
    }
  }else if(state == 4){
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
    //////////////////// THROW WHEN READY //////////////////////////////////////////////
    else if ((digitalRead(LS_2) == HIGH) && (digitalRead(LS_1) == HIGH))
    {
      stop_bot();
      if (final_align_pos == false)
      {
        final_align_pos = true;
        x_finalpos_tz1 = xEnc.read();
      }
      check_manual();
      if (manual_detect){
        throw_ball();
        throw_done_tz1 = true;
      }else{
        if (throw_done_tz1){
          state_change();
        }
      }
    }
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }else if(state == 2){
    
  }
}








