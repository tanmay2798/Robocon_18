// Library
#include <SPI.h>
#include <Kangaroo.h>
#include <Encoder.h>
#include <Sabertooth.h>

///////////// Kangaroo /////////////////
boolean kang_enable = true;
long two_motor_kang_baud_rate = 115200;
long one_motor_kang_baud_rate = 115200;
long drive_kang_ramp_1 = 440;        // kang ramp              |---------------------|
long drive_kang_ramp_2 = 470;        // kang ramp              | Change in loop also |
long drive_kang_ramp_3 = 1250;        // kang ramp             |---------------------|

KangarooSerial K(Serial2);          // for two motor kang.
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2'); 
KangarooSerial M(Serial3);          // for one motor kang. 
KangarooChannel M1(M, '2');

//////////////////////// PS3 Variable /////////////////////////////
boolean PS3_enable = true;
boolean ramp_change = false;
long ps3_deadband_trans = 25;
long ps3_deadband_rot_analog = 15;
long ps3_deadband_rot_lr = 20;
float rot_multiplier_analog=1;
float rot_multiplier_lr=0.1;

float ps_x, ps_y, ps_w;

/////////////////////////System Variable //////////////////////////
boolean speed_factor_change = false;
boolean debug_enable = false;
long Serial_baud_rate = 19200;
long speed_factor = 15;
long speed_factor_initial = 28; // |-----Change in loop---|
long speed_factor_final = 28;   // |---------also---------|

float x_vel_factor = 1;
float y_vel_factor = 1;

float cor_fact_1 = 0.94;
float cor_fact_2 = 1;
float cor_fact_3 = -0.945;

float v1, v2, v3, vx, vy, w,vs;
long avg_speed;
float target_x;
float target_y;
boolean update_state;
////////////////////////Fine Control //////////////////////////
boolean fine_control = false;
long fine_vel = 28;
double bot_theta;
double cur_time;
double prev_time;
double theta_correction_factor = 1.05;
boolean theta_correct = false;
boolean change_control = false;

///////////////////// Manual Ramping //////////////////////////////
boolean manual_ramp = false;
double ramp_factor_acc=75;                            // step_acc * speed_factor
double ramp_factor_deacc=140;                            // step_acc * speed_factor
double step_acc = ramp_factor_acc/speed_factor;
double step_deacc = ramp_factor_deacc/speed_factor;
double step_acc_rot = 7;
double step_deacc_rot = 10;

long omega = 0;
long vel_x = 0;
long vel_y = 0;
boolean motion_stop = false;

//////////////////// Automation Variables ////////////////////////
long dis_start_to_load = 14000;
long dis_start_to_load_slow = 2500;
long dis_load_to_tz1 = 43500;
long dis_load_to_tz1_slow = 1500;
long dis_transfer;
long dis_stop;

long max_speed_1=100;
long max_speed_2=180;
long slow_speed_1=20;
long slow_speed_2=20;

Encoder Enc(20,21);
boolean automation=false;
long enc_dis;
long auto_x = 0;
long auto_y = 0;
long automation_state = 0;
long dis_origin = 0;

/////////////////////Rack Variable/////////////////////////////
long rack_w = 75;
long rack_baud_rate = 115200;
long rack_1_pin_grip = 30;
long rack_1_pin_lift = 32;
long rack_2_pin = 34;
bool rack_1_state = false;
bool rack_2_state = false;
long rack_rotate = 0;
boolean rack_fine_control=false;

Encoder y_enc(2,3);
Sabertooth rack_st(128,Serial1);
long cur_rack_pos = 0;
long prev_rack_pos = 0;
long rack_angle = 300;

void setup() {
  rack_init();
  Serial.begin(Serial_baud_rate);
  if (kang_enable){
    Serial3.begin(one_motor_kang_baud_rate);    // Sabertooth only
    Serial2.begin(two_motor_kang_baud_rate);
    one_motor_kangaroo_init();
    Serial.println("One motor kang started");
    two_motor_kangaroo_init();
    Serial.println("Two motor kang started");
  }
  Serial1.begin(rack_baud_rate);            // Rack Motor
  rack_st.autobaud();
  if (PS3_enable){
    PS3_init();
    Serial.println("PS3 Initialized");
  }
  pinMode(13,OUTPUT);

}

void loop() {
  if (PS3_enable){
    PS3_getValue();                 // Get Value from PS3
  }
  if (update_state){
    get_encoder_val();              // Get Free Wheel Encoder Value
    automatic_motion();             // Add automatic velocities
    target_x = (auto_x + ps_x)*x_vel_factor;       // Target velocities
    target_y = (auto_y + ps_y)*y_vel_factor;
    //----------- Get velocities of wheels from vel_x and vel_y --------------
    if (fine_control){                             // Reduce speed and ramp factor for fine manual control
      speed_factor_initial = 7;
      speed_factor_final = 7;
      ramp_factor_acc = 50;
      step_acc = ramp_factor_acc/speed_factor;
      manual_ramp = false;
    }
    else{
      speed_factor_initial = 28;
      speed_factor_final = 28;
      manual_ramp = false;
    }
    if (manual_ramp){
      ramp();                         // Ramp velocities target_x ---> vel_x
      vx = -vel_x;
      vy = -vel_y;
      w = -omega;
    }
    else{
      vx = target_x;
      vy = target_y;
      w = -ps_w;
      if (vx == 0 && vy == 0 && ps_w == 0){
        drive_kang_ramp_1 = 40000;
        drive_kang_ramp_2 = 40000;
        drive_kang_ramp_3 = 40000;
      }
      else if (fine_control){
        drive_kang_ramp_1 = 80;
        drive_kang_ramp_2 = 80;
        drive_kang_ramp_3 = 160;
      }
      else{
        /* if(ramp_change==true){
           drive_kang_ramp_1 = 400;
           drive_kang_ramp_2 = 440;
           drive_kang_ramp_3 = 1000;
          }*/
          //if(ramp_change==false){
            drive_kang_ramp_1 = 450;
           drive_kang_ramp_2 = 490;
           drive_kang_ramp_3 = 1050; 
        //  }
         
      }
    }
    avg_speed = (int)pow((vx*vx + vy*vy),0.5);
    speed_factor = speed_factor_final;  

    v1 = -(0.3333 * vx - 0.5774 * vy - 0.3333 * w)*cor_fact_1;
    v2 = (-0.3333 * vx - 0.5774 * vy + 0.3333 * w)*cor_fact_2;
    v3 = (0.6667 * vx + 0.3333 * w)*cor_fact_3;
    if (motion_stop){               // Motion Disable
      v1 = 0;
      v2 = 0;
      v3 = 0;
      automation_state = 0;
    }
    //-----------------------------------------------------------
    if (kang_enable){
      drive_kangaroo_update(v1, v2, v3, true);        // Write value to kangaroo
    }
    rack_1_rotate();                                // Check and rotate rack
  }
  
  if (debug_enable){
    Serial.print(" Enc_dis ");
    Serial.print(enc_dis);
    Serial.print(" y_dis ");
    Serial.print(y_enc.read());
    Serial.print(" Tar_X ");
    Serial.print(target_x);
    Serial.print(" Actual_X ");
    Serial.print(vel_x);
    Serial.print(" Tar_Y ");
    Serial.print(target_y);
    Serial.print(" Actual_Y ");
    Serial.print(vel_y);
    Serial.print(" Tar_w ");
    Serial.print(ps_w);
    Serial.print(" Actual_w ");
    Serial.println(omega);
  }
}

void ramp(){
  if (target_x >= vel_x){
    if (vel_x >= 0){
      if ((target_x - vel_x) > step_acc){
        vel_x += step_acc;
      }
      else{
        vel_x = target_x;
      }
    }else{
      if ((target_x - vel_x) > step_deacc){
        vel_x += step_deacc;
      }
      else{
        vel_x = target_x;
      }
    }
  }
  else if (target_x < vel_x){
    if (vel_x < 0){
      if ((vel_x - target_x) > step_acc){
        vel_x -= step_acc;
      }
      else{
        vel_x = target_x;
      }
    }
    else{
      if ((vel_x - target_x) > step_deacc){
        vel_x -= step_deacc;
      }
      else{
        vel_x = target_x;
      }
    }
  }
  if (target_y >= vel_y){
    if (vel_y >= 0){
      if ((target_y - vel_y) > step_acc){
        vel_y += step_acc;
      }
      else{
        vel_y = target_y;
      }
    }
    else{
      if ((target_y - vel_y) > step_deacc){
        vel_y += step_deacc;
      }
      else{
        vel_y = target_y;
      }
    }
  }
  else if (target_y < vel_y){
    if (vel_y < 0){
      if ((vel_y - target_y) > step_acc){
        vel_y -= step_acc;
      }
      else{
        vel_y = target_y;
      }
    }
    else{
      if ((vel_y - target_y) > step_deacc){
        vel_y -= step_deacc;
      }
      else{
        vel_y = target_y;
      }
    }
  }
  if (ps_w >= omega){
    if (omega >= 0){
      if ((ps_w - omega) > step_acc_rot){
        omega += step_acc_rot;
      }
      else{
        omega = ps_w;
      }
    }
    else{
      if ((ps_w - omega) > step_deacc_rot){
        omega += step_deacc_rot;
      }
      else{
        omega = ps_w;
      }
    }
  }
  else if (ps_w < omega){
    if (omega < 0){
      if ((omega - ps_w) > step_acc_rot){
        omega -= step_acc_rot;
      }
      else{
        omega = ps_w;
      }
    }
    else{
      if ((omega - ps_w) > step_deacc_rot){
        omega -= step_deacc_rot;
      }
      else{
        omega = ps_w;
      }
    }
  }
}

