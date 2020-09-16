/////////////// PS3 USB ////////////////////////////////////

#include <PS3USB.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
USB Usb;      
PS3USB PS3(&Usb);

////////////////////////////////////////////////////////////

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
  motion_stop = false;
  update_state = false;
  fine_control = false;
  rack_rotate = 0;
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {    //   by default
    update_state = true;
    ps_x = (PS3.getAnalogHat(RightHatX) - 127);                //          |
    ps_y = (127 - PS3.getAnalogHat(LeftHatY));                //   (0,0)|
    // ps_w = rl with deadband * fast rot + analog with deadband * slow rot
    ps_w = PS3_deadband_rot((PS3.getAnalogButton(R2) - PS3.getAnalogButton(L2)),ps3_deadband_rot_lr)*rot_multiplier_lr;              
    PS3_deadband_trans(ps3_deadband_trans);
    if (PS3.getButtonPress(UP)){
      if(rack_fine_change==false){
      rack_rotate = 1;
      if (debug_enable){
         Serial.println("UP");
      }   
      }
      else {
        fine_control=true;
        ps_x=0;
        ps_y=fine_vel;
      }
    }
    else if (PS3.getButtonPress(DOWN)){
      if(rack_fine_change==false){
      rack_rotate = 2; 
      if (debug_enable){
        Serial.println("DOWN");
      }
      }
      else {
        fine_control=true;
        ps_x=0;
        ps_y=-fine_vel;
      }
    }
    else if (PS3.getButtonPress(LEFT)){
     fine_control=true;
     if (debug_enable){
       Serial.print("LEFT");
     }
     ps_x = -fine_vel;
     ps_y = 0;
    }
    else if (PS3.getButtonPress(RIGHT)){
      fine_control=true;
      if (debug_enable){
        Serial.print("RIGHT");
      }
      ps_x = fine_vel;
      ps_y = 0;
    }
    if (PS3.getButtonClick(SQUARE)){
      motion_stop = true;
    }
    if (PS3.getButtonClick(TRIANGLE)){
      if (debug_enable){
        Serial.println("TRIANGLE");
      }
       rack_2_state = ! rack_2_state;
       rack_2_grip();
    }
    if (PS3.getButtonClick(CROSS)){
      if (debug_enable){
        Serial.println("CROSS");
      }
      rack_1_state = !rack_1_state;
      rack_1_grip();
    }
    if (PS3.getButtonClick(L1)){
      if (debug_enable){
        Serial.println("L1");
      }
     change_control=!change_control;
    }
     if (PS3.getButtonClick(R1)){
      rack_fine_change=!rack_fine_change;
    }
  }
}

void PS3_deadband_trans(int deadband_value) {
  ps_x = dead_band(ps_x, deadband_value);
  ps_y = dead_band(ps_y, deadband_value);
  if(change_control==true){
    ps_x=-ps_x;
    ps_y=-ps_y;
  }
  if (ps_x > 0){
    ps_x-=ps3_deadband_trans;
  }else if (ps_x < 0){
    ps_x+=ps3_deadband_trans;
  }
  if (ps_y > 0){
    ps_y-=ps3_deadband_trans;
  }else if (ps_y < 0){
    ps_y+=ps3_deadband_trans;
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
