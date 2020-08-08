
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include <math.h>    

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!                                                                                                                 
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  750 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2100 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
//  Serial.println("8 channel Servo test!");

  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(1000); //small delay to let the capacitors charge up
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

//define some constants
float UL = 40.5; //upper leg length in mm
float LL = 54; //lower leg length in mm
float OB_01; float OB_23; float OB_45; float OB_67;
float theta; //angle between horizontal and upper leg
float phi;   //angle between lower leg and upper leg
float alpha01; float alpha23; float alpha45; float alpha67;
float beta01; float beta23; float beta45; float beta67; 
float pi = 3.1415;
float target_x_01; float target_x_23; float target_x_45; float target_x_67;
float target_y_01; float target_y_23; float target_y_45; float target_y_67;
float start;
float duration;
float pos;
int theta1_start; int theta3_start; int theta5_start; int theta7_start; 
int theta1_end; int theta3_end; int theta5_end; int theta7_end;
int phi0_start; int phi2_start; int phi4_start; int phi6_start; 
int phi0_end; int phi2_end; int phi4_end; int phi6_end; 
int phi0; int phi2; int phi4; int phi6;
int theta1; int theta3; int theta5; int theta7; 
int i; int k; long j; long m; long n; long p;
int step_duration;

void loop() {

  
//robot starts in sleeping position
MoveRobot(20,20,20,20,11,11,11,11);
delay(250);


//stand upright, full straight legs
theta1_start=11; theta3_start=11; theta5_start=11; theta7_start=11; 
phi0_start=20; phi2_start=20; phi4_start=20; phi6_start=20;
duration = 1000; 
theta1_end=90; theta3_end=theta1_end; theta5_end=theta1_end; theta7_end=theta1_end;
phi0_end=180; phi2_end=phi0_end; phi4_end=phi0_end; phi6_end=phi0_end; 
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

// so called neutral position
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=90; theta3_end=theta1_end; theta5_end=theta1_end; theta7_end=theta1_end;
phi0_end=90; phi2_end=phi0_end; phi4_end=phi0_end; phi6_end=phi0_end; 
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//legs out straight, feet upwards
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=180; theta3_end=theta1_end; theta5_end=theta1_end; theta7_end=theta1_end;
phi0_end=90; phi2_end=phi0_end; phi4_end=phi0_end; phi6_end=phi0_end; 
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//full leg curl back
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 500; 
theta1_end=180; theta3_end=theta1_end; theta5_end=theta1_end; theta7_end=theta1_end;
phi0_end=20; phi2_end=phi0_end; phi4_end=phi0_end; phi6_end=phi0_end; 
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//fully splayed out
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=180; theta3_end=theta1_end; theta5_end=theta1_end; theta7_end=theta1_end;
phi0_end=180; phi2_end=phi0_end; phi4_end=phi0_end; phi6_end=phi0_end; 
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);


//lean backwards and forwards a few times
i=-1;
for (k = 0; k <= 3; k++) {
  i=-i;
  theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
  phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
  duration = 800; 
  if(i==1){
    theta1_end=90; theta3_end=90; 
    theta5_end=11; theta7_end=11;
    phi0_end=180; phi2_end=180; 
    phi4_end=20; phi6_end=20; }
  else{
    theta1_end=11; theta3_end=11; 
    theta5_end=90; theta7_end=90;
    phi0_end=20; phi2_end=20; 
    phi4_end=180; phi6_end=180; }
  DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);
}

//lean left and right a few times
i=-1;
for (k = 0; k <= 3; k++) {
  i=-i;
  //lean left
  theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
  phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
  duration = 800; 
  if(i==1){
    theta1_end=20; theta3_end=65; 
    theta5_end=20; theta7_end=65;
    phi0_end=45; phi2_end=120; 
    phi4_end=45; phi6_end=120;}
  else{
    theta1_end=65; theta3_end=20; 
    theta5_end=65; theta7_end=20;
    phi0_end=120; phi2_end=45; 
    phi4_end=120; phi6_end=45; 
  }
  DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);
}

//start to sit, front legs out straight
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=180; theta3_end=180; 
theta5_end=11; theta7_end=11;
phi0_end=180; phi2_end=180; 
phi4_end=20; phi6_end=20; 
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//sit down by sitting onto back legs
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=165; theta3_end=165;
theta5_end=11; theta7_end=11;
phi0_end=180; phi2_end=180; 
phi4_end=180; phi6_end=180;
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//sit up
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=120; theta3_end=120;
theta5_end=70; theta7_end=70;
phi0_end=180; phi2_end=180; 
phi4_end=180; phi6_end=180;
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//bend sit legs
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 500; 
theta1_end=120; theta3_end=120;
theta5_end=45; theta7_end=45;
phi0_end=180; phi2_end=180; 
phi4_end=135; phi6_end=135;
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

////wave top arms up and down while extended (straight)
//i=-1;
//for (k = 0; k <= 3; k++) {
//  i=-i;
//  theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
//  phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
//  duration = 1000; 
//  if (i==1){theta1_end=150; theta3_end=90;}
//  else{theta1_end=90; theta3_end=150;}
//  theta5_end=45; theta7_end=45;
//  phi0_end=180; phi2_end=180; 
//  phi4_end=135; phi6_end=135;
//  DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);
//  delay(100);
//}

//wave top arms up and down while bent
i=-1;
for (k = 0; k <= 3; k++) {
  i=-i;
  theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
  phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
  duration = 500; 
  if (i==1){theta1_end=170; theta3_end=45;}
  else{theta1_end=45; theta3_end=170;}  
//  theta1_end=170; theta3_end=70;
  theta5_end=45; theta7_end=45;
  phi0_end=60; phi2_end=60; 
  phi4_end=135; phi6_end=135;
  DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);
//  delay(100);
}

//start to return to normal 4 leg stance
//sit down by sitting onto back legs
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=180; theta3_end=180;
theta5_end=11; theta7_end=11;
phi0_end=180; phi2_end=180; 
phi4_end=180; phi6_end=180;
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//rotate back around so front legs are on ground, move back left first
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=180; theta3_end=180; 
theta5_end=11; theta7_end=11;
phi0_end=180; phi2_end=180; 
phi4_end=180; phi6_end=20; 
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//rotate back around so front legs are on ground
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=180; theta3_end=180; 
theta5_end=11; theta7_end=11;
phi0_end=180; phi2_end=180; 
phi4_end=20; phi6_end=20; 
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//half stance
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
target_x_01=0; target_x_23=0; target_x_45=0; target_x_67=0;
target_y_01=60; target_y_23=60; target_y_45=60; target_y_67=60;
InvKinematics_phi0_end(target_x_01, target_y_01); InvKinematics_phi2_end(target_x_23, target_y_23); InvKinematics_phi4_end(target_x_45, target_y_45); InvKinematics_phi6_end(target_x_67, target_y_67);
InvKinematics_theta1_end(target_x_01, target_y_01); InvKinematics_theta3_end(target_x_23, target_y_23); InvKinematics_theta5_end(target_x_45, target_y_45); InvKinematics_theta7_end(target_x_67, target_y_67);
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

//slide backwards and fowards a few times
i=-1;
for (k = 0; k <= 4; k++) {
  i=-i;
  theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
  phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
  duration = 800; 
  target_x_01=30*i; target_x_23=30*i; target_x_45=30*i; target_x_67=30*i;
  target_y_01=60; target_y_23=60; target_y_45=60; target_y_67=60;
  InvKinematics_phi0_end(target_x_01, target_y_01); InvKinematics_phi2_end(target_x_23, target_y_23); InvKinematics_phi4_end(target_x_45, target_y_45); InvKinematics_phi6_end(target_x_67, target_y_67);
  InvKinematics_theta1_end(target_x_01, target_y_01); InvKinematics_theta3_end(target_x_23, target_y_23); InvKinematics_theta5_end(target_x_45, target_y_45); InvKinematics_theta7_end(target_x_67, target_y_67);
  DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);
}


//go to start position to test leg
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
target_x_01= 0; target_x_23=0;  target_x_45=-22.5; target_x_67=15;
target_y_01=73; target_y_23=50; target_y_45=60;    target_y_67=60;
InvKinematics_phi0_end(target_x_01, target_y_01); InvKinematics_phi2_end(target_x_23, target_y_23); InvKinematics_phi4_end(target_x_45, target_y_45); InvKinematics_phi6_end(target_x_67, target_y_67);
InvKinematics_theta1_end(target_x_01, target_y_01); InvKinematics_theta3_end(target_x_23, target_y_23); InvKinematics_theta5_end(target_x_45, target_y_45); InvKinematics_theta7_end(target_x_67, target_y_67);
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);
//Serial.println("end of start pos");
//delay(5000);


//walk!
float leg01_x_array[10]={0,-7.5,-15,-22.5,-30,0,30,22.5,15,7.5};
float leg01_y_array[10]={70,65,65,65,70,50,65,65,65,70};
float leg23_x_array[10]={0,30,22.5,15,7.5,0,-7.5,-15,-22.5,-30};
float leg23_y_array[10]={50,65,65,65,70,70,65,65,65,70};
float leg45_x_array[10]={-22.5,-30,0,30,22.5,15,7.5,0,-7.5,-15};
float leg45_y_array[10]={65,65,50,70,65,65,65,70,70,65};
float leg67_x_array[10]={15,7.5,0,-7.5,-15,-22.5,-30,0,30,22.5};
float leg67_y_array[10]={65,65,70,70,65,65,65,50,70,65};
for (k = 0; k <= 11; k++) {  
  for (p = 0; p <= 9; p++) {  
//    Serial.print(k); Serial.print(" "); Serial.println(p);
    theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
    phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
    duration = 120; 
//    if(k>7){duration = 80;}
    target_x_01= leg01_x_array[p]; target_x_23= leg23_x_array[p]; target_x_45=leg45_x_array[p]; target_x_67=leg67_x_array[p];
    target_y_01= leg01_y_array[p]; target_y_23= leg23_y_array[p]; target_y_45=leg45_y_array[p]; target_y_67=leg67_y_array[p];
    InvKinematics_phi0_end(target_x_01, target_y_01); InvKinematics_phi2_end(target_x_23, target_y_23); InvKinematics_phi4_end(target_x_45, target_y_45); InvKinematics_phi6_end(target_x_67, target_y_67);
    InvKinematics_theta1_end(target_x_01, target_y_01); InvKinematics_theta3_end(target_x_23, target_y_23); InvKinematics_theta5_end(target_x_45, target_y_45); InvKinematics_theta7_end(target_x_67, target_y_67);
    DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);
  }
}

// go to sleep position
theta1_start=theta1_end; theta3_start=theta3_end; theta5_start=theta5_end; theta7_start=theta7_end; 
phi0_start=phi0_end; phi2_start=phi2_end; phi4_start=phi4_end; phi6_start=phi6_end;
duration = 1000; 
theta1_end=11; theta3_end=theta1_end; theta5_end=theta1_end; theta7_end=theta1_end;
phi0_end=20; phi2_end=phi0_end; phi4_end=phi0_end; phi6_end=phi0_end; 
DoMapping(duration, theta1_start, theta1_end, theta3_start, theta3_end, theta5_start, theta5_end, theta7_start, theta7_end, phi0_start, phi0_end, phi2_start, phi2_end, phi4_start, phi4_end, phi6_start,phi6_end);

delay(20000);
}


float InvKinematics_phi0_end(float target_x_01, float target_y_01){
  OB_01=sqrt(sq(target_x_01)+sq(target_y_01)); 
  phi0_end=acos((sq(OB_01)-sq(UL)-sq(LL))/(-2*UL*LL))*180/pi;
  return phi0_end;
}
float InvKinematics_phi2_end(float target_x_23, float target_y_23){
  OB_23=sqrt(sq(target_x_23)+sq(target_y_23)); 
  phi2_end=acos((sq(OB_23)-sq(UL)-sq(LL))/(-2*UL*LL))*180/pi;
  return phi2_end;
}
float InvKinematics_phi4_end(float target_x_45, float target_y_45){
  target_x_45=-target_x_45;
  OB_45=sqrt(sq(target_x_45)+sq(target_y_45)); 
  phi4_end=acos((sq(OB_45)-sq(UL)-sq(LL))/(-2*UL*LL))*180/pi;
  return phi4_end;
}
float InvKinematics_phi6_end(float target_x_67, float target_y_67){
  target_x_67=-target_x_67;
  OB_67=sqrt(sq(target_x_67)+sq(target_y_67));
  phi6_end=acos((sq(OB_67)-sq(UL)-sq(LL))/(-2*UL*LL))*180/pi; 
  return phi6_end;
}
float InvKinematics_theta1_end(float target_x_01, float target_y_01){
  OB_01=sqrt(sq(target_x_01)+sq(target_y_01)); 
  alpha01=acos((sq(LL)-sq(UL)-sq(OB_01))/(-2*UL*OB_01))*180/pi;
  beta01=atan2(target_y_01,target_x_01)*180/pi;
  theta1_end=180-beta01-alpha01;
  return theta1_end;
}
float InvKinematics_theta3_end(float target_x_23, float target_y_23){
  OB_23=sqrt(sq(target_x_23)+sq(target_y_23)); 
  alpha23=acos((sq(LL)-sq(UL)-sq(OB_23))/(-2*UL*OB_23))*180/pi;
  beta23=atan2(target_y_23,target_x_23)*180/pi;
  theta3_end=180-beta23-alpha23;
  return theta3_end;
}
float InvKinematics_theta5_end(float target_x_45, float target_y_45){
  target_x_45=-target_x_45;
  OB_45=sqrt(sq(target_x_45)+sq(target_y_45)); 
  alpha45=acos((sq(LL)-sq(UL)-sq(OB_45))/(-2*UL*OB_45))*180/pi;
  beta45=atan2(target_y_45,target_x_45)*180/pi;
  theta5_end=180-beta45-alpha45;
  return theta5_end;
}
float InvKinematics_theta7_end(float target_x_67, float target_y_67){
  target_x_67=-target_x_67;
  OB_67=sqrt(sq(target_x_67)+sq(target_y_67));
  alpha67=acos((sq(LL)-sq(UL)-sq(OB_67))/(-2*UL*OB_67))*180/pi;
  beta67=atan2(target_y_67,target_x_67)*180/pi; 
  theta7_end=180-beta67-alpha67;
  return theta7_end;
}


int DoMapping(float duration, int theta1_start, int theta1_end, int theta3_start, int theta3_end, int theta5_start, int theta5_end, int theta7_start, int theta7_end, int phi0_start, int phi0_end, int phi2_start, int phi2_end, int phi4_start, int phi4_end, int phi6_start,int phi6_end){
//int DoMapping(float duration, int theta_start, int theta_end, int phi_start, int phi_end){
  start = millis();
  do{
    pos = ((millis() - start) / duration)*1000.0;
    int phi0 = map(pos,0,1000,phi0_start,phi0_end); 
    int phi2 = map(pos,0,1000,phi2_start,phi2_end); 
    int phi4 = map(pos,0,1000,phi4_start,phi4_end); 
    int phi6 = map(pos,0,1000,phi6_start,phi6_end); 
    int theta1 = map(pos,0,1000,theta1_start,theta1_end); 
    int theta3 = map(pos,0,1000,theta3_start,theta3_end); 
    int theta5 = map(pos,0,1000,theta5_start,theta5_end); 
    int theta7 = map(pos,0,1000,theta7_start,theta7_end); 
    MoveRobot(phi0,phi2,phi4,phi6,theta1,theta3,theta5,theta7);
  }while (millis() - start < duration);
  pos=1000;
  int phi0 = map(pos,0,1000,phi0_start,phi0_end); 
  int phi2 = map(pos,0,1000,phi2_start,phi2_end); 
  int phi4 = map(pos,0,1000,phi4_start,phi4_end); 
  int phi6 = map(pos,0,1000,phi6_start,phi6_end); 
  int theta1 = map(pos,0,1000,theta1_start,theta1_end); 
  int theta3 = map(pos,0,1000,theta3_start,theta3_end); 
  int theta5 = map(pos,0,1000,theta5_start,theta5_end); 
  int theta7 = map(pos,0,1000,theta7_start,theta7_end); 
  MoveRobot(phi0,phi2,phi4,phi6,theta1,theta3,theta5,theta7);  
}


int MoveRobot(int phi0, int phi2, int phi4, int phi6, int theta1, int theta3, int theta5, int theta7){
  int pwm0; int pwm1; int pwm2; int pwm3; int pwm4; int pwm5; int pwm6; int pwm7; 

  //upper leg theta = 11
  int theta1_11 = 2350; int theta3_11 = 550;  int theta5_11 = 500;  int theta7_11 = 2350; 
  //upper leg theta = 90
  int theta1_90 = 1475; int theta3_90 = 1375; int theta5_90 = 1350; int theta7_90 = 1500; 
  //upper leg theta = 180
  int theta1_180 = 520; int theta3_180 = 2300; int theta5_180 = 2350; int theta7_180 = 550; 
  //lower leg phi = 90
  int phi0_90 = 1550; int phi2_90 = 1475; int phi4_90 = 1500; int phi6_90 = 1550; 
  //lower leg phi = 180
  int phi0_180 = 650; int phi2_180 = 2400; int phi4_180 = 2425; int phi6_180 = 625; 
  //lower leg phi = 20
  int phi0_20 = 2300; int phi2_20 = 750; int phi4_20 = 770; int phi6_20 = 2220;

  // map theta and phi (degrees) to pwm duty cycle (microseconds)
  if (phi0>90){pwm0 = map(phi0, 90, 180, phi0_90, phi0_180);}
  else {pwm0 = map(phi0, 20, 90, phi0_20, phi0_90);}
  if (phi2>90){pwm2 = map(phi2, 90, 180, phi2_90, phi2_180);}
  else {pwm2 = map(phi2, 20, 90, phi2_20, phi2_90);}
  if (phi4>90){pwm4 = map(phi4, 90, 180, phi4_90, phi4_180);}
  else {pwm4 = map(phi4, 20, 90, phi4_20, phi4_90);}
  if (phi6>90){pwm6 = map(phi6, 90, 180, phi6_90, phi6_180);}
  else {pwm6 = map(phi6, 20, 90, phi6_20, phi6_90);}
  if (theta1>90){pwm1 = map(theta1, 90, 180, theta1_90, theta1_180);}
  else {pwm1 = map(theta1, 11, 90, theta1_11, theta1_90);}
  if (theta3>90){pwm3 = map(theta3, 90, 180, theta3_90, theta3_180);}
  else {pwm3 = map(theta3, 11, 90, theta3_11, theta3_90);}
  if (theta5>90){pwm5 = map(theta5, 90, 180, theta5_90, theta5_180);}
  else {pwm5 = map(theta5, 11, 90, theta5_11, theta5_90);}
  if (theta7>90){pwm7 = map(theta7, 90, 180, theta7_90, theta7_180);}
  else {pwm7 = map(theta7, 11, 90, theta7_11, theta7_90);}
  
  //check pwm values arent going past limits
  if (pwm0 < 650){pwm0 = 650;}    if (pwm2 > 2400){pwm2 = 2400;}  if (pwm4 > 2425){pwm4 = 2425;}  if (pwm6 < 625){pwm6 = 625;} 
  if (pwm0 > 2300){pwm0 = 2300;}  if (pwm2 < 750){pwm2 = 750;}    if (pwm4 < 770){pwm4 = 770;}    if (pwm6 > 2220){pwm6 = 2220;}
  if (pwm1 > 2350){pwm1 = 2350;}  if (pwm3 < 550){pwm3 = 550;}    if (pwm5 < 500){pwm5 = 500;}    if (pwm7 > 2350){pwm7 = 2350;} 
  if (pwm1 < 520){pwm1 = 520;}    if (pwm3 > 2300){pwm3 = 2300;}  if (pwm5 > 2350){pwm5 = 2350;}  if (pwm7 < 550){pwm7 = 550;} 
  //send pwm to servos
  pwm.writeMicroseconds(0, pwm0); pwm.writeMicroseconds(1, pwm1); pwm.writeMicroseconds(2, pwm2);
  pwm.writeMicroseconds(3, pwm3); pwm.writeMicroseconds(4, pwm4); pwm.writeMicroseconds(5, pwm5);   
  pwm.writeMicroseconds(6, pwm6); pwm.writeMicroseconds(7, pwm7);

}

