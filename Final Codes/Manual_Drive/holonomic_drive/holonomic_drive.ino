     // Library
#include <SPI.h>
#include <Kangaroo.h>
#include <Encoder.h>
#include <Sabertooth.h>

///////////// Kangaroo /////////////////
boolean kang_enable = true;
long two_motor_kang_baud_rate = 115200;
long one_motor_kang_baud_rate = 115200;
long drive_kang_ramp_1 = 450;        // kang ramp              |---------------------|
long drive_kang_ramp_2 = 500;        // kang ramp              | Change in loop also |
long drive_kang_ramp_3 = 1225;         // kang ramp            |---------------------|

KangarooSerial K(Serial2);          // for two motor kang.
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2'); 
KangarooSerial M(Serial3);          // for one motor kang. 
KangarooChannel M1(M, '2');

//////////////////////// PS3 Variable /////////////////////////////
boolean PS3_enable = true;
long ps3_deadband_trans = 25;
long ps3_deadband_rot_analog = 15;
long ps3_deadband_rot_lr = 20;
float rot_multiplier_analog=1;
float rot_multiplier_lr=0.2;
float ps_x, ps_y, ps_w;

/////////////////////////System Variable //////////////////////////
boolean debug_enable = false; 
boolean update_state;
boolean motion_stop = false;
long Serial_baud_rate = 19200;
long speed_factor = 15; // change in loop also

float x_vel_factor = 1;
float y_vel_factor = 1;

float cor_fact_1 = 0.94;
float cor_fact_2 = 1;
float cor_fact_3 = -0.945;

float v1, v2, v3, vx, vy, w;
float target_x;
float target_y;
////////////////////////Fine Control //////////////////////////
boolean fine_control = false;
long fine_vel = 28;
boolean change_control = false;

/////////////////////Rack Variable/////////////////////////////
long rack_w = 40;
long rack_baud_rate = 9600;
long rack_1_pin_grip = 30;
long rack_1_pin_lift = 32;
long rack_2_pin = 34;
bool rack_1_state = false;
bool rack_2_state = false;
long rack_rotate = 0;
boolean rack_fine_change = false;
Sabertooth rack_st(128,Serial1);

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
  if(change_control==true || rack_fine_change==true){
    digitalWrite(13,HIGH);
  }
  else{
     digitalWrite(13,LOW);
  }
  if (update_state){
    target_x =  (ps_x)*x_vel_factor;       // Target velocities
    target_y =  (ps_y)*y_vel_factor;
    //----------- Get velocities of wheels from vel_x and vel_y --------------
    if (fine_control){                             // Reduce speed and ramp factor for fine manual control
      speed_factor = 8;
    }
    else{
      speed_factor = 30;
    }
    vx = target_x;
    vy = target_y;
    w = -ps_w;
    if (vx == 0 && vy == 0 && w == 0){
      drive_kang_ramp_1 = 6000;
      drive_kang_ramp_2 = 6300;
      drive_kang_ramp_3 = 22000;
    }
    else if (fine_control){
      drive_kang_ramp_1 = 60000;
      drive_kang_ramp_2 = 60000;
      drive_kang_ramp_3 = 60000;
    }
    else{
       drive_kang_ramp_1 = 450;
       drive_kang_ramp_2 = 510   ;
       drive_kang_ramp_3 = 1500;
    }
    v1 = -(0.3333 * vx - 0.5774 * vy - 0.3333 * w)*cor_fact_1;
    v2 = (-0.3333 * vx - 0.5774 * vy + 0.3333 * w)*cor_fact_2;
    v3 = (0.6667 * vx + 0.3333 * w)*cor_fact_3;
    if (motion_stop){               // Motion Disable
      v1 = 0;
      v2 = 0;
      v3 = 0;
    }
    if (kang_enable){
      drive_kangaroo_update(v1, v2, v3, true);        // Write value to kangaroo
    }
    rack_1_rotate();                                // Check and rotate rack
  }  
  if (debug_enable){  
    Serial.print(" Tar_X ");
    Serial.print(target_x);
    Serial.print(" Actual_X ");
    Serial.print(vx);
    Serial.print(" Tar_Y ");
    Serial.print(target_y);
    Serial.print(" Actual_Y ");
    Serial.print(vy);
    Serial.print(" Tar_w ");
    Serial.print(ps_w);
    Serial.print(" Actual_w ");
    Serial.println(w);
  }
}


