void back_Kangaroo_init() {
  K1.start();
  K1.home().wait();
  K1.s(0);
  Serial.println("Started K1");
  K2.start();
  K2.home().wait();
  K2.s(0);
  Serial.println("Started K2");
}

void front_kangaroo_init() {
  M1.start();
  M1.home().wait();
  M1.s(0);
  Serial.println("Started M1");
}

void drive_kangaroo_update(float motor1, float motor2, float motor3, boolean ramp) {
  if (ramp) {
    K1.s(motor1 * speed_factor, drive_kang_ramp);
    K2.s(motor2 * speed_factor, drive_kang_ramp);
    M1.s(motor3 * speed_factor, drive_kang_ramp);
  } else {
    K1.s(motor1 * speed_factor);
    K2.s(motor2 * speed_factor);
    M1.s(motor3 * speed_factor);
  }
}
