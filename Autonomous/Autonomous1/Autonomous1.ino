/*
    Y               0_________70
    ^               |__LSA_0__|
    |                   /\
    |                  /  \
    |                 /    \
    |                /      \
    |             - / wheel1 \ +
    |   70 __      /          \      __ 70
    |     |L |    /            \    | L|
    |     |S |   /              \   | S|
    |     |A |  /                \  | A|
    |     |3_| /                  \ |_2|
    |    0    /                    \    0
    |       +/w                    w\-
    |       /  h                  h  \
    |      /     e               e    \                                  
    |     /       e             e      \
    |    /         l           l        \
    |   /___________3_________2__________\
    |               -         +
    |               0_________70
    |               |__LSA_1__|
    |___________________________________________________________>X
*/

//SETUP
int W_1=-1,W_2=-1,W_3=1;                                                                                                                               //set W_X=0 or 1 according to make it work

//LIBRARIES
#include <Kangaroo.h>

//CONNECTION PINS
int LSA_0=A3,LSA_1=A1,LSA_2=A2,LSA_3=A0,LSA_junction_0=2,LSA_junction_2=3;                                                                            //-------LSA Input Pins

KangarooSerial kangaroo_12(Serial1);                                                                                                                  //----|
KangarooSerial kangaroo_3(Serial2);                                                                                                                   //----|
KangarooChannel wheel_1(kangaroo_3,'2');                                                                                                              //------Kangaroo Input/Output Pins (make changes in setup also), don't use Serial0
KangarooChannel wheel_2(kangaroo_12,'1');                                                                                                             //----|
KangarooChannel wheel_3(kangaroo_12,'2');                                                                                                             //----|

//CONSTANTS
int max_base_speed=0, acceleration=100;

//VARIABLES
int base_speed=0;

void setup()
{
  Serial.begin(19200);
  Serial1.begin(115200);                   //----|
  Serial2.begin(115200);                   //----|
  wheel_1.start();                        //----|
  wheel_1.home().wait();                  //--------Initializing Kangaroo
  wheel_2.start();                        //----|
  wheel_2.home().wait();                  //----|
  wheel_3.start();                        //----|
  wheel_3.home().wait();                  //----|
}

void loop()
{
  base_speed=constrain(base_speed+acceleration,0,max_base_speed);
  wheel_1.s(W_1*base_speed);
  wheel_2.s(W_2*base_speed);
  wheel_3.s(W_3*base_speed);
  Serial.print("LSA_0=");
  Serial.print(analogRead(LSA_0));
  Serial.print("LSA_1=");
  Serial.print(analogRead(LSA_1));
  Serial.print("LSA_2=");
  Serial.print(analogRead(LSA_2));
  Serial.print("LSA_3=");
  Serial.println(analogRead(LSA_3));
  delay(2000);
}
