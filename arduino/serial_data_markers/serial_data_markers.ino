#include <Servo.h>

const float pi = 3.14159, theta_r = radians(48.0), theta_p = radians(23.2), theta_s[] = {-pi/3,
2*pi/3, pi, 0, pi/3, -2*pi/3},
RD = 2.395, PD = 3.3, L1 = .617, L2 = 5.2813, z_home = 5.25, servo_min = -radians(80), servo_max = radians(80),
servo_mult = 400/(pi/4),
p[2][6] = {{PD*cos(pi/6 + theta_p), PD*cos(pi/6 - theta_p), PD*cos(-(pi/2 - theta_p)),
-PD*cos(-(pi/2 - theta_p)), -PD*cos(pi/6 - theta_p), -PD*cos(pi/6 + theta_p)},
{PD*sin(pi/6 + theta_p), PD*sin(pi/6 - theta_p), PD*sin(-(pi/2 - theta_p)),
PD*sin(-(pi/2 - theta_p)), PD*sin(pi/6 - theta_p), PD*sin(pi/6 + theta_p)}},
re[2][6] = {{RD*cos(pi/6 + theta_r), RD*cos(pi/6 - theta_r), RD*cos(-(pi/2 - theta_r)),
-RD*cos(-(pi/2 - theta_r)), -RD*cos(pi/6 - theta_r), -RD*cos(pi/6 + theta_r)},
{RD*sin(pi/6 + theta_r), RD*sin(pi/6 - theta_r), RD*sin(-(pi/2 - theta_r)),
RD*sin(-(pi/2 - theta_r)), RD*sin(pi/6 - theta_r), RD*sin(pi/6 + theta_r)}};
/*
theta_r = angle between attachment points
theta_p = angle between rotation points
theta_s = orientation of the servos
RD = distance to end eector attachment points
PD = distance to servo rotation points
L1 = servo arm length
L2 = connecting arm length 
z_home = default z height with servo arms horizontal
servo_min = lower limit for servo arm angle
servo_max = upper limit for servo arm angle
servo_mult = multiplier to convert to milliseconds
p = location of servo rotation points in base frame [x/y][1-6]
re = location of attachment points in end eector frame [x/y][1-6]
*/
const int servo_pin[] = {9,3, 5, 11, 6, 10}, servo_zero[6] = {1710, 1280, 1700, 1300, 1680, 1300};
/*
servo_pin = servo pin assignments,
servo_zero = zero angles for each servo (horizontal)
*/


Servo servo[6];
/*
Servos 0, 2, 4: reversed (+ = down, - = up)
Servos 1, 3, 5: normal (+ = up, - = down)
*/

static float pe[6] = {0,0,0,radians(0),radians(0),radians(0)};
/*
  pe = location and orientation of end eector frame relative to the base frame [sway, surge,
  heave, pitch, roll, yaw)
  theta_a = angle of the servo arm
  servo_pos = value written to each servo
  q = position of lower mounting point of connecting link [x,y,x][1-6]
  r = position of upper mounting point of connecting link
  dl = dierence between x,y,z coordinates of q and r
  dl2 = distance between q and r
  */


void setup()
{
  Serial.begin(9600);
  for(int i = 0; i < 6; i++)
  {
    servo[i].attach(servo_pin[i]);
    servo[i].writeMicroseconds(servo_zero[i]);
  }
  delay(1000);
}


const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;
float data;


void loop()
{

  

  recvWithStartEndMarkers();
  showNewData();
  kinematics();
  
}


void kinematics()
{
  float theta_a[6], servo_pos[6], q[3][6], r[3][6], dl[3][6], dl2[6];
  for(int i = 0; i < 6; i++)
  {
    q[0][i] = L1*cos(-theta_a[i])*cos(theta_s[i]) + p[0][i];
    q[1][i] = L1*cos(-theta_a[i])*sin(theta_s[i]) + p[1][i];
    q[2][i] = -L1*sin(-theta_a[i]);
    
    r[0][i] = re[0][i]*cos(pe[4])*cos(pe[5]) + re[1][i]*(sin(pe[3])*sin(pe[4])*cos(pe[5]) -
    cos(pe[3])*sin(pe[5])) + pe[0];
    r[1][i] = re[0][i]*cos(pe[4])*sin(pe[5]) + re[1][i]*(cos(pe[3])*cos(pe[5]) +
    sin(pe[3])*sin(pe[4])*sin(pe[5])) + pe[1];
    r[2][i] = -re[0][i]*sin(pe[4]) + re[1][i]*sin(pe[3])*cos(pe[4]) + z_home + pe[2];
    
    dl[0][i] = q[0][i] - r[0][i];
    dl[1][i] = q[1][i] - r[1][i];
    dl[2][i] = q[2][i] - r[2][i];
    
    dl2[i] = sqrt(dl[0][i]*dl[0][i] + dl[1][i]*dl[1][i] + dl[2][i]*dl[2][i]) - L2;
    
    theta_a[i] += dl2[i];

//    Serial.print("\nbefore\t");
//    Serial.print(theta_a[i]);
    
    theta_a[i] = constrain(theta_a[i], servo_min, servo_max);
    
    if(i%2 == 1) servo_pos[i] = servo_zero[i] + theta_a[i]*servo_mult;
    else servo_pos[i] = servo_zero[i] - theta_a[i]*servo_mult;
  }
  
  for(int i = 0; i < 6; i++)
  {
//    Serial.print("\nServo number:\t");
//    Serial.print(i);
//    Serial.print("\nValue:\t");
//    Serial.print(theta_a[i]);
//    Serial.print("\t");
//    Serial.println(servo_pos[0]);
//    Serial.println(pe[3],DEC);
    servo[i].writeMicroseconds(servo_pos[i]);
    
//    servo[i].writeMicroseconds(1500);
  }
}

void limit_check()
{
  if(pe[3] > radians(5)) { pe[3] = radians(5); }
  if(pe[4] > radians(5)) { pe[4] = radians(5); }
  if(pe[3] < -radians(5)) { pe[3] = -radians(5); }
  if(pe[4] < -radians(5)) { pe[4] = -radians(5); }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != 't' && rc != 'p') {
              
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
              
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;

                float temp = atof(receivedChars);
                if (temp != 0.0)
                {
                  if (rc == 't') { pe[4] = temp; }
                  if (rc == 'p') { pe[3] = temp; }
                  if (rc == 'z') { pe[2] = temp; }
                }
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
//        Serial.print("This just in ... ");
//        Serial.println(data,DEC);
//        Serial.println(receivedChars);
//        Serial.println(pe[4],DEC);
//        Serial.println(pe[3],DEC);
        newData = false;
    }
}
