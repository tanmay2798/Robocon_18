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
        


void PS3_getValue() {
  Usb.Task();
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {    //   by default
    lhx = PS3.getAnalogHat(RightHatX) - 127;                //          |
    lhy = (127 - PS3.getAnalogHat(RightHatY));                //   (0,0)|
    // rhx = rl with deadband * fast rot + analog with deadband * slow rot
    rhx = PS3_deadband_rot((PS3.getAnalogButton(R2) - PS3.getAnalogButton(L2)),ps3_deadband_rot_lr)*rot_multiplier_lr + PS3_deadband_rot((PS3.getAnalogHat(LeftHatX) - 127), ps3_deadband_rot_analog)*rot_multiplier_analog;               //   -------|-------
    PS3_deadband_trans(ps3_deadband_trans);

    
    if (PS3.getButtonClick(UP)){
      rack_rotate = !rack_rotate;
    }
    else if (PS3.getButtonClick(DOWN)){
      rack_2_state = !rack_2_state;
    }
    else if (PS3.getButtonClick(LEFT)){
      automation_state = 2;
      dis_reset();
    }
    else if (PS3.getButtonClick(RIGHT)){
      automation_state = 1;
      dis_reset();
    }
    if (PS3.getButtonClick(TRIANGLE)){
      motion_stop = false;
    }
    if (PS3.getButtonClick(SQUARE)){
    }
    if (PS3.getButtonClick(CROSS)){
      Serial.println("HALT!");
      motion_stop = true;
    }
    if (PS3.getButtonClick(L1)){
      rack_1_state = !rack_1_state;
    }
    if (PS3.getButtonClick(R1)){
      automation = !automation;
      automation_state = 0;
      dis_reset();
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
