
///////////////////// PS3 BT ///////////////////////////////
/*
  #include <PS3BT.h>
  #include <usbhub.h>
  #ifdef dobogusinclude
  #include <spi4teensy3.h>
  #endif
  #include <SPI.h>
  USB Usb;
  //USBHub Hub1(&Usb); // Some dongles have a hub inside
  BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
  // You can create the instance of the class in two ways
  PS3BT PS3(&Btd); // This will just create the instance
  //PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
*/
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
  update_state = false;
  rack_rotate = 0;
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {    //   by default
    update_state = true;
    ps_x = (PS3.getAnalogHat(RightHatX) - 127);                //          |
    ps_y = (127 - PS3.getAnalogHat(RightHatY));                //   (0,0)|
    // ps_w = rl with deadband * fast rot + analog with deadband * slow rot
    ps_w = PS3_deadband_rot((PS3.getAnalogButton(R2) - PS3.getAnalogButton(L2)),ps3_deadband_rot_lr)*rot_multiplier_lr + PS3_deadband_rot((PS3.getAnalogHat(LeftHatX) - 127), ps3_deadband_rot_analog)*rot_multiplier_analog;               //   -------|-------
    PS3_deadband_trans(ps3_deadband_trans);
    if (PS3.getButtonPress(UP)){
      rack_rotate = 1;
    }
    else if (PS3.getButtonPress(DOWN)){
      rack_rotate = 2;
    }
    else if (PS3.getButtonClick(LEFT)){
      automation_state = 2;
      dis_reset();
    }
    else if (PS3.getButtonClick(RIGHT)){
      automation_state += 1;
      dis_reset();
    }
    if (PS3.getButtonClick(TRIANGLE)){
      motion_stop = false;
    }
    if (PS3.getButtonClick(SQUARE)){
      speed_factor_change = !speed_factor_change;
    }
    if (PS3.getButtonClick(CROSS)){
      Serial.println("HALT!");
      motion_stop = true;
    }
    if (PS3.getButtonClick(L1)){
      rack_1_state = !rack_1_state;
      rack_1_grip();
    }
    if (PS3.getButtonClick(R1)){
      automation = true;
      dis_reset();
    }
    if (PS3.getButtonClick(R3){
      rack_2_state = ! rack_2_state;
    }
  }
}

void PS3_deadband_trans(int deadband_value) {
  ps_x = dead_band(ps_x, deadband_value);
  ps_y = dead_band(ps_y, deadband_value);
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
