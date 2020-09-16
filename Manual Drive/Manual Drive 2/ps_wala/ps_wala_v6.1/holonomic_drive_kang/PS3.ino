#include <PS3USB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
USB Usb;      
PS3USB PS3(&Usb);

void PS3_init() {
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo,
#endif             // Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    while (1); //halt
  }
}
        
int temp_x = 0;
int temp_y = 0;
boolean man_control=true;

void PS3_getValue() {
  Usb.Task();
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {    //   by default
    lhx = PS3.getAnalogHat(RightHatX) - 127;                //          |
    lhy = (127 - PS3.getAnalogHat(RightHatY));                //   (0,0)|
    // rhx = rl with deadband * fast rot + analog with deadband * slow rot
    rhx = PS3_deadband_rot((PS3.getAnalogButton(R2) - PS3.getAnalogButton(L2)),ps3_deadband_rot_lr)*rot_multiplier_lr + PS3_deadband_rot((PS3.getAnalogHat(LeftHatX) - 127), ps3_deadband_rot_analog)*rot_multiplier_analog;               //   -------|-------
    PS3_deadband_trans(ps3_deadband_trans);
    if (PS3.getButtonClick(UP)){
      if (temp_y < auto_speed){
        temp_y+=auto_speed_ramp;
      }
    }
    else if (PS3.getButtonClick(DOWN)){
      if (temp_y > -auto_speed){
        temp_y-=auto_speed_ramp;
      }
    }
    else if (PS3.getButtonClick(LEFT)){
      if (temp_x > -auto_speed){
        temp_x-=auto_speed_ramp;
      }
    }
    else if (PS3.getButtonClick(RIGHT)){
      if (temp_x < auto_speed){
        temp_x+=auto_speed_ramp;
      }
    }
    if (PS3.getButtonClick(TRIANGLE)){
      man_control = true;
      Serial.println("Manual Mode ON");
    }
    if (PS3.getButtonClick(SQUARE)){
      man_control = false;
      Serial.println("Auto Mode ON");
    }
    if (PS3.getButtonClick(CROSS)){
      temp_y = 0;
      temp_x = 0;
      lhx=0;
      lhy=0;
      rhx=0;
      Serial.println("HALT!");
    }
    if (!man_control){
      lhx = temp_x;
    }
  }
}

void PS3_deadband_trans(int deadband_value) {
  lhx = dead_band(lhx, deadband_value);
  lhy = dead_band(lhy, deadband_value);
  if (lhx > 0){
    lhx-=ps3_deadband_trans;
  }else if (lhx < 0){
    lhx+=ps3_deadband_trans;
  }
  if (lhy > 0){
    lhy-=ps3_deadband_trans;
  }else if (lhy < 0){
    lhy+=ps3_deadband_trans;
  }
}

int PS3_deadband_rot(int val, int deadband_value) {
  val = dead_band(val, deadband_value);
  if (val > 0){
    val-=deadband_value;
  }else if (val < 0){
    val+=deadband_value;
  }
  return val;
}

int dead_band(int parameter, int deadband1) {
  if (abs(parameter) < deadband1) {
    return 0;
  } else {
    return parameter;
  }
}
