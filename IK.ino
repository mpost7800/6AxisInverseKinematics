#include <Servo.h>

#define degtorad 0.0174533
#define radtodeg 57.2958
#define l1  5 //length arms in cm
#define l2  5
#define b  5 //height first rotational joint in cm
#define g 5 //gripper radius adjustment
#define gz 5 // gripper z direction adjustment
#define pi 3.14159265359
int g_s;
float i;
int pulse_length;
int Speed = 5 ;                     // Speed of servo
int precession = 200;
float omega,a1,a2;
float x,y,z,r,z_p,x_a,y_a,z_a;
float x0,y0,z0,dx,dy,dz;
float angles[5];                    // All the angles of the servos
float anglexy;

int servo[6][3]={                   // Presets for servo values
  {135,325,530},
  {110,335,560},
  {110,340,555},
  {110,335,560},
  {150,370,600},
  {170,290,370}
};

Servo servoo[6];

void setup() {
  Serial.begin(9600);
  servoo[1].attach(3);
  servoo[2].attach(5);
  servoo[3].attach(6);
  servoo[4].attach(9);
  servoo[5].attach(10);
  servoo[6].attach(11);
}

void inversekinematics (float x, float y, float z)
{
  z_p = z-b;
  r = sqrt(x*x+y*y); 
  angles[0] = radtodeg*atan2(y,x);
  a1 =  sqrt( 1 - sq( (r*r+z_p*z_p-l1*l1-l2*l2)/(2*l1*l2) ) );
  a2 = (r*r+z_p*z_p-l1*l1-l2*l2)/(2*l1*l2);
  float res = atan2(-a1,a2);
  angles[2] = -radtodeg*res;
  float k1 = l1+l2*cos(res);
  float k2 = l2*sin(res);
  angles[1] = radtodeg*(atan2(z_p,r) - atan2(k2,k1));
  angles[3] = 45 - angles[1] + angles[2];
  servoo[1].write(angles[0]);
  delay(100);
  servoo[2].write(angles[1]);
  delay(100);
  servoo[3].write(angles[2]);
  delay(100);
  servoo[4].write(angles[3]);
  delay(100);
}

void loop() {               //Write your IK commands here!!!
  inversekinematics(3,3,0);
  delay(5000);
  inversekinematics(5,3,0);
  delay(5000);
}
