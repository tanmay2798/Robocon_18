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
int L_01=1,L_23=1;                                                                                                                                   //set L_XY=1 or -1 according to orientation of LSA_X and LSA_Y
int W_1=-1,W_2=-1,W_3=1;                                                                                                                             //set W_X=1 or -1 according to orientation of wheelX

//LIBRARIES
#include <Kangaroo.h>

//CONNECTION PINS
int LSA_0=A3,LSA_1=A0,LSA_2=A2,LSA_3=A1,LSA_junction_0=2,LSA_junction_2=3;                                                                            //-------LSA Input Pins

KangarooSerial kangaroo_12(Serial1);                                                                                                                  //----|
KangarooSerial kangaroo_3(Serial2);                                                                                                                   //----|
KangarooChannel wheel_1(kangaroo_3,'2');                                                                                                              //------Kangaroo Input/Output Pins (make changes in setup also), don't use Serial0
KangarooChannel wheel_2(kangaroo_12,'1');                                                                                                             //----|
KangarooChannel wheel_3(kangaroo_12,'2');                                                                                                             //----|

//CONSTANTS
float target=450;                                                                                                                                     //Desired Position for Line Following
float linear_Kp_x=0,linear_Kd_x=0,angular_Kp_x=0,angular_Kd_x=0,linear_Kp_y=0.3,linear_Kd_y=0,angular_Kp_y=0,angular_Kd_y=0;    //Constants of PD Controller for Line Following
boolean shaft=false;                                                                                                                                   //true=> Movement in x, false=> Movement in y
float max_base_speed=000,acceleration=0;

//VARIABLES
float linear_error=0,angular_error=0,previous_linear_error=0,previous_angular_error=0;                                                                //Errors in position
float angular_control=0,linear_control=0,base_speed=0;                                                                                                //Robot Velocities
int v_1=0,v_2=0,v_3=0;                                                                                                                                //Wheel Velocities
int LSA_front=LSA_3,LSA_rear=LSA_2;                                                                                                                   //LSA's in use
float linear_Kp=linear_Kp_x,linear_Kd=linear_Kd_x,angular_Kp=angular_Kp_x,angular_Kd=angular_Kd_x;                                                    //PD constants in use

void setup()
{
  Serial1.begin(115200);                  //----|
  Serial2.begin(115200);                  //----|
  wheel_1.start();                        //----|
  wheel_1.home().wait();                  //--------Initializing Kangaroo
  wheel_2.start();                        //----|
  wheel_2.home().wait();                  //----|
  wheel_3.start();                        //----|
  wheel_3.home().wait();                  //----|
  if (shaft==true)
  {
    LSA_front=LSA_3;
    LSA_rear=LSA_2;
    linear_Kp=linear_Kp_x;
    linear_Kd=linear_Kd_x;
  }
  else
  {
    LSA_front=LSA_0;
    LSA_rear=LSA_1;
    linear_Kp=linear_Kp_y;
    linear_Kd=linear_Kd_y;
  }
}

void loop()
{
  float position_front=analogRead(LSA_front);
  float position_rear=analogRead(LSA_rear);
  base_speed=constrain(base_speed+acceleration,0,max_base_speed);
  if (position_front<1000 && position_rear <1000)
  {
    linear_error=(position_front+position_rear)/2-target;
    angular_error=position_front-position_rear;
  }
  linear_control=linear_Kp*linear_error+linear_Kd*(linear_error-previous_linear_error);
  angular_control=angular_Kp*angular_error+angular_Kd*(angular_error-previous_angular_error);
  previous_linear_error=linear_error;
  previous_angular_error=angular_error;
  if (shaft==true)
  {
    v_1=-base_speed+L_23*angular_control;
    v_2=base_speed*0.5-L_23*(linear_control*0.866-angular_control);
    v_3=base_speed*0.5+L_23*(linear_control*0.866+angular_control);
  }
  else
  {
    v_1=L_01*(linear_control+angular_control);
    v_2=-base_speed*0.866-L_01*(linear_control*0.5-angular_control);
    v_3=base_speed*0.866-L_01*(linear_control*0.5+angular_control);
  }
  wheel_1.s(W_1*v_1);
  wheel_2.s(W_2*v_2);
  wheel_3.s(W_3*v_3);
}
