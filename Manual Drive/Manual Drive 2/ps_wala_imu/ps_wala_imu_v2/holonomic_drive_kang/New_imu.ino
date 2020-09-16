Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event;

void imu_init()
{
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    Serial.print("\n Resetting IMU");
    delay(100);
    resetFunc();
    imu_enable = false;
  } else {
    Serial.println("IMU is Initialized");
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  displaySensorStatus();
}

void displaySensorStatus(void)
{
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void calc_vxvy()
{  t1 = millis();
  bno.getEvent(&event);
  yaw = event.orientation.x - init_yaw;  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  Serial.print(millis() - t1);
  float yaw_rad = -yaw * 0.01745329;
  float cosine = cos(yaw_rad);
  float sine = sin(yaw_rad);
  vx = (lhx * cosine + lhy * sine);
  vy = (-lhx * sine + lhy * cosine);
}

void reset_angle(){
  bno.getEvent(&event);
  init_yaw = event.orientation.x;
  }
