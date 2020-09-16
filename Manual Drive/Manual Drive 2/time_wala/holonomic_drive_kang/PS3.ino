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

// time based parameters

void PS3_getValue() {
  Usb.Task();
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {    //   by default
    // Buttons State Change
    if(PS3.getButtonClick(CIRCLE)){
      ps3_auto_state = !ps3_auto_state;     // Manual <-> Automatic
      ps3_movt_state = 0;
    }else if (PS3.getButtonClick(TRIANGLE)){
      ps3_movt_state = 1;
    }else if(PS3.getButtonClick(R1)){
      ps3_movt_state = 2;
    }else if(PS3.getButtonClick(R2)){
      ps3_movt_state = 3;
    }else if(PS3.getButtonClick(L1)){
      ps3_movt_state = 4;
    }else if(PS3.getButtonClick(L2)){
      ps3_movt_state = 5;
    }else if(PS3.getButtonClick(CROSS)){
      ps3_movt_state = 6;
    }else if(PS3.getButtonClick(SQUARE)){
      ps3_movt_state = 7;
    }
    // States ================================================================
    if (!ps3_auto_state){         // Manual mode
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
    }else{      // Auto mode
      if (ps3_movt_state == 0){
        movt = false;
        lhy = 0;
        lhx = 0;
      }else if(ps3_movt_state == 1){        // Start -> Loading
        if (!movt){
          start_time = millis();
          movt = true;
        }
        if (millis() - start_time < 1000){
          lhy = -120;    // forward speed
          lhx = 10;     // wall contact speed
        }else{
          lhy = 0;
          lhx = 0;
          ps3_movt_state = 0;
        }
      }else if(ps3_movt_state == 2){        // Loading -> TZ1
        if (!movt){
          start_time = millis();
          movt = true;
        }
        if (millis() - start_time < 2000){
          lhy = 120;    // forward speed
          lhx = 10;     // wall contact speed
        }else{
          lhy = 0;
          lhx = 0;
          ps3_movt_state = 0;
        }        
      }else if(ps3_movt_state == 3){        // TZ1 -> Loading
        if (!movt){
          start_time = millis();
          movt = true;
        }
        if (millis() - start_time < 2000){
          lhy = -120; // forward speed
          lhx = 10;     // wall contact speed
        }else{
          lhy = 0;
          lhx = 0;
          ps3_movt_state = 0;
        }
      }else if(ps3_movt_state == 4){        // Loading -> TZ2
        if (!movt){
          start_time = millis();
          movt = true;
        }
        if (millis() - start_time < 2500){
          lhy = 120; // forward speed
          lhx = 10;     // wall contact speed
        }else{
          lhy = 0;
          lhx = 0;
          ps3_movt_state = 0;
        }
      }else if(ps3_movt_state == 5){        // TZ2 -> Loading
        if (!movt){
          start_time = millis();
          movt = true;
        }
        if (millis() - start_time < 2500){
          lhy = -120; // forward speed
          lhx = 10;     // wall contact speed
        }else{
          lhy = 0;
          lhx = 0;
          ps3_movt_state = 0;
        }
      }else if(ps3_movt_state == 6){        // Stop
        ps3_movt_state = 0;
      }else if(ps3_movt_state == 7){       // Reset
        lhx = 10;       // wall contact speed
      }
    }
  } // PS3 connected
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
