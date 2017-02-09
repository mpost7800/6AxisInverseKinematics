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
  /*servoangle(0,0,angles[0]);
  servoangle(1,1,angles[1]);
  servoangle(2,2,angles[1]);
  servoangle(3,3,angles[2]);*/
  angles[3] = 45 - angles[1] + angles[2];
  //servoangle(4,4,angles[3]);
}
/*void servoangle(int servo_number, int channel, float angle)
{
    int n = servo_number; //n so that the formulas don't get cluttered with servo_angle everywhere
    if (angle >= 0 && angle <= 90)
    {
      pulse_length = int(float(servo[n][0]+angle*( (servo[n][1]-servo[n][0]) )/90.0));  
    }    
    else if(angle > 90 && angle <= 180)
    {
      pulse_length = int(float(  servo[n][1] + (angle-90.0) *( (servo[n][2]-servo[n][1]))/90.0 ) );     
    }
    else // if (angle <0 && angle>180) //redudant protection just in case
    {
      return;
    }
    servoo[channel].write(pulse_length);
}*/

void loop() {
  inversekinematics(3,3,0);
  Serial.println(angles[0]);
  Serial.println(angles[1]);
  Serial.println(angles[2]);
  Serial.println(angles[3]);
  servoo[1].write(angles[0]);
  delay(100);
  servoo[2].write(angles[1]);
  delay(100);
  servoo[3].write(angles[2]);
  delay(100);
  servoo[4].write(angles[3]);
  delay(5000);
  inversekinematics(5,3,0);
  Serial.println(angles[0]);
  Serial.println(angles[1]);
  Serial.println(angles[2]);
  Serial.println(angles[3]);
  servoo[1].write(angles[0]);
  delay(100);
  servoo[2].write(angles[1]);
  delay(100);
  servoo[3].write(angles[2]);
  delay(100);
  servoo[4].write(angles[3]);
  delay(5090);
}
