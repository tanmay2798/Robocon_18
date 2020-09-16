int LSAread()
{

  posx = k1.getp().value();
  velx = k1.gets().value();
  posy = yEnc.read();
  vely = k2.getp().value();

  if (shaft == false)
  {
    if (analogRead(pin2) < 900)
    {
      valread_y = 1;
      acceleration = 100;
      Kp1 = -0.225, Kd1 = -0.65;
      Kp2 = 0.4, Kd2 = 0.3;
      pval2 = analogRead(pin2);
    }
    if (valread_y == 1)
    {
      val2 = analogRead(pin2);
    }
    else val2 = target;
    val1 = analogRead(pin1);
  }
  else
  {
    val1 = analogRead(pin4);
    val2 = analogRead(pin3);
  }

  if (val1 > 1000) val1 = pval1;
  if (val2 > 1000) val2 = pval2;
  val = (val1 + val2) / 2;
  pval1 = val1;
  pval2 = val2;

  if (val < 1000)
    return (target - val);
  else
    return (pdifference);

}
int angRead()
{
  ang = val1 - val2;
  return ang;
}
