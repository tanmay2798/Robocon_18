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
    //Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
//  //Serial.println("Started");
  //Serial.print(F("\r\nPS3 USB Library Started"));
}
//int ramp_factor = 10;              
int temp_x = 0;
int temp_y = 0;
boolean man_control=true;
void PS3_getValue() {
  Usb.Task();
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {    //   by default
    if (man_control){
      lhx = PS3.getAnalogHat(LeftHatX) - 127;                //          |
      lhy = (127 - PS3.getAnalogHat(LeftHatY));                //   (0,0)|
      rhx = PS3.getAnalogHat(RightHatX) - 127;               //   -------|-------
      rhy=  PS3.getAnalogHat(RightHatY)-127;                 //            |  (255, 255)
    }else{
      lhx = 0;
      lhy = 0;
      rhx = 0;
      rhy=  0;
    }
    //rhy = 127 - PS3.getAnalogHat(RightHatY);             //          
      ////Serial.println("test");
   /* if (PS3.getButtonPress(UP)){
      //Serial.println("UP");
      if (temp_y < 120){
        temp_y+=10;
      }
    }
    else if (PS3.getButtonPress(DOWN)){
      if (temp_y > -120){
        temp_y-=10;
      }
    }
    else if (PS3.getButtonPress(LEFT)){
      if (temp_x > -120){
        temp_x-=10;
      }
    }
    else if (PS3.getButtonPress(RIGHT)){
      if (temp_x < 120){
        temp_x+=10;
      }
    }else{
      if (temp_x > 0){
        temp_x -= ramp_factor;
      }else if(temp_x < 0){
        temp_x += ramp_factor;
      }
      if (temp_y > 0){
        temp_y -= ramp_factor;
      }else if(temp_y < 0){
        temp_y += ramp_factor;
      }
    }*/
    if (PS3.getButtonClick(TRIANGLE)){
      temp_y = 0;
      temp_x = 0;
      lhx=0;
      lhy=0;
      rhx=0;
      curdown=0;
      man_control = !man_control;
      Serial.println("chal raha hoon bhai");
    }
    //lhx = temp_x;
    //lhy = temp_y;
   //    rhx=0;
   
  }
}

void PS3_deadband_trans(int deadband_value) {
  lhx = dead_band(lhx, deadband_value);
  lhy = dead_band(lhy, deadband_value);
}

void PS3_deadband_rot(int deadband_value) {
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
