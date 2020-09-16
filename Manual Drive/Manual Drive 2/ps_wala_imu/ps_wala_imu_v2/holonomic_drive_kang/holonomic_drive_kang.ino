// Library
#include <SPI.h>
#include <PS3USB.h>
#include <Kangaroo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/////////////////////////IMU Variable //////////////////////////
float yaw;
#define BNO055_SAMPLERATE_DELAY_MS 10
uint8_t system_status, self_test_results, system_error;
long t1;
boolean imu_enable = true;
float init_yaw=0;
//////////////////////// PS3 Variable /////////////////////////////
float lhx, lhy, rhx, rhy, l2;
int up = 0, down = 0;
int curup = 0, curdown = 0;

int ps3_deadband_trans = 25;
int ps3_deadband_rot_analog = 15;
int ps3_deadband_rot_lr = 20;
float rot_multiplier_analog=0.4;
float rot_multiplier_lr=1;

boolean dir_reset;

// for back motors.
KangarooSerial K(Serial1);
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2');

// for front motor. 
KangarooSerial M(Serial2);
KangarooChannel M1(M, '2');
/////////////////////////System Variable //////////////////////////
float v1, v2, v3, vx, vy, w,vs;
int angle = 120;
int Serial_baud_rate = 19200;
int back_kangaroo_baud_rate = 19200;
int front_kangaroo_baud_rate = 19200;
int speed_factor = 15;
boolean debug_enable = true;
int avg_speed;

int drive_kang_ramp = 40000;              // change later
float max_speed = 3000;                  // change later
int speed_factor_initial = 8;
int speed_factor_final = 28;
int auto_speed = 200;
int auto_speed_ramp = 15;

int ledpin=33;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setup() {
  analogWrite(A8, 1023);
  Serial1.begin(front_kangaroo_baud_rate);    // Sabertooth only
  Serial2.begin(back_kangaroo_baud_rate);
  Serial.begin(Serial_baud_rate);
  Serial.println("Initializing IMU");
  imu_init();
  Serial.println("IMU initialized");
  PS3_init();
  Serial.println("PS3 started");

  back_Kangaroo_init();
  Serial.println("back kangaroo started");
  front_kangaroo_init();
  Serial.println("front kangaroo started");
  pinMode(ledpin,OUTPUT);
}

void loop() {
  if (system_error && imu_enable){
    digitalWrite(ledpin,HIGH);
    Serial.println("Resetting-2");
    delay(100);
    resetFunc();
  }
  else{
    digitalWrite(ledpin,LOW);
  }
  displaySensorStatus();
  PS3_getValue();
  if (dir_reset){
    reset_angle();
  }
  vx = lhx;
  vy = lhy;
  w = rhx;
  Serial.print("calc ");
  if (imu_enable){
    calc_vxvy();
  }
  Serial.print("done ");
  avg_speed = (int)pow((vx*vx + vy*vy),0.5);
  vs=abs(avg_speed);
  speed_factor = map(avg_speed, 0, (127-ps3_deadband_trans)*1.42, speed_factor_initial, speed_factor_final);
  v1 = -0.3333 * vx - 0.5774 * vy + 0.3333 * w;
  v2 = -0.3333 * vx + 0.5774 * vy + 0.3333 * w;
  v3 = 0.6667 * vx + 0.3333 * w;
  
  if (debug_enable) {
    Serial.print(" lhx = ");
    Serial.print(lhx);
    Serial.print("\t");
    Serial.print("lhy = ");
    Serial.print(lhy);
    Serial.print("\t");
    Serial.print("rhx = ");
    Serial.print(rhx);
    Serial.print("\t");
    Serial.print("vx = ");
    Serial.print(vx);
    Serial.print("\t");
    Serial.print("vy = ");
    Serial.print(vy);
    Serial.print("\t");
    Serial.print("yaw = ");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print("IMU Status = ");
    Serial.println(imu_enable);
  }
  drive_kangaroo_update(v1, v2, v3, true);
}
