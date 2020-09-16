USB Usb;      
PS3USB PS3(&Usb);

void PS3_init() {
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo,
#endif             // Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 USB Library Started"));
}              

void PS3_getValue() {
  Usb.Task();
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {    //   by default
    lhx = PS3.getAnalogHat(LeftHatX) - 127;                //          |
    lhy = 127 - PS3.getAnalogHat(LeftHatY);                //   (0,0)  |
    rhx = PS3.getAnalogHat(RightHatX) - 127;               //   -------|-------
    //rhy = 127 - PS3.getAnalogHat(RightHatY);             //          |  (255, 255)
    if (PS3.getButtonClick(UP)){
      if (curup==0){
        up=1;
        curup=1; 
      }
      else{
        up=0;
        curup=0;
      }
    }
    if (PS3.getButtonClick(DOWN)){
      if (curdown==0){
        down=1;
        curdown=1;
      }
      else{
        down=0;
        curdown=0;
      }
    }
  }
}

void PS3_deadband(int deadband_value) {
  lhx = dead_band(lhx, deadband_value);
  lhy = dead_band(lhy, deadband_value);
  rhx = dead_band(rhx, deadband_value);
  rhy = dead_band(rhy, deadband_value);
}

int dead_band(int parameter, int deadband1) {
  if (abs(parameter) < deadband1) {
    return 0;
  } else {
    return parameter;
  }
}
