#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Float64.h>
double SetpointL, SetpointR;
double InputL, OutputL;
double InputR, OutputR;
const byte encoder0pinA = 2;//A pin -> the interrupt pin 2
const byte encoder0pinB = 4;//B pin -> the digital pin 4
const byte encoder1pinA = 3;//A pin -> the interrupt pi
const byte encoder1pinB = 8;//B pin -> the digital pin 8
byte encoder0PinALast;
long encoder_pulse_counterL;//the number of the pulses
long encoder_pulse_counterR;
int directionL , directionR ;//the rotation direction
long previousmillis = 0;
long currentmillis = 0;
float old_pwm =0 ;
float old_pwm1 = 0;
float left_Output , right_Output;
bool dirL,dirR;
unsigned long currentTime, previousTime;
double elapsedTime;
int left_pwm = 5;
int left_dir = 6;

int right_pwm = 10;
int right_dir = 11;

PID LPID(&InputL, &OutputL, &SetpointL,3.6,0.0001,0.0001, DIRECT);
PID RPID(&InputR, &OutputR, &SetpointR,3.7,0.0001,0.0001, DIRECT);
// ros part

ros::NodeHandle nh;

void arg( const geometry_msgs::Twist &msg){
    float angular_z, linear_x;
    float linear_vel_x_mins,linear_vel_y_mins,angular_vel_z_mins,tangential_vel,x_rpm,y_rpm,tan_rpm;
    float wheels_x_distance_ = 0.370;
    float wheel_circumference_ = 0.373;
   
    linear_x = msg.linear.x;
    angular_z = msg.angular.z;
    linear_vel_x_mins = linear_x * 60.0;
    angular_vel_z_mins = angular_z * 60.0;
    x_rpm = linear_vel_x_mins / wheel_circumference_;
    tangential_vel = angular_vel_z_mins * ((wheels_x_distance_) / 2.0);
    tan_rpm = tangential_vel / wheel_circumference_;

   
    float checkSetpointL = (x_rpm - tan_rpm);
    float checkSetpointR = (x_rpm + tan_rpm);
    if(checkSetpointL < 0){
      left_backward();

    }
    else if (checkSetpointL == 0.0){
      stopL();
    }
    else if (checkSetpointR == 0.0){
      stopR();
    }
    else if (checkSetpointL > 0.0){
      left_forward();
    }
 
    if(checkSetpointR < 0.0){
      right_backward();
    }
    else if (checkSetpointR == 0.0){
      stopR();
    }
    else if (checkSetpointR > 0.0){
      right_forward();
    }
    SetpointL = abs(checkSetpointL);
    SetpointR = abs(checkSetpointR);
}


ros::Subscriber<geometry_msgs::Twist> commands("/cmd_vel", &arg);


void EncoderInitL(){
    attachInterrupt(digitalPinToInterrupt(2), wheelSpeed, RISING);//int.0     if an interrupt occurs then go to wheel speed for left encoder values
}



void EncoderInitR(){
    attachInterrupt(digitalPinToInterrupt(3), wheelSpeed1, RISING);//int.0        if an interrupt  occurs then go to wheel speed 1 for right motor encoder values
}



void wheelSpeed(){
    encoder_pulse_counterL += 1;              // a= a+1 or a+=1 means increment of encoder values of left motor
    directionL = digitalRead(encoder0pinA) == digitalRead(encoder0pinB) ? -1 : 1; //
}



void wheelSpeed1(){
    encoder_pulse_counterR += 1;  //increment of encoder values of right motor
    directionR = digitalRead(encoder1pinA) == digitalRead(encoder1pinB) ? -1 : 1;
}


void left_backward(){
  digitalWrite(left_dir, HIGH);
  analogWrite(left_pwm,OutputL);
       
}

void right_backward(){
  digitalWrite(right_dir,LOW);
  analogWrite(right_pwm,OutputR);
}


void left_forward(){  
  digitalWrite(left_dir, LOW);
  analogWrite(left_pwm,OutputL);      
}

void right_forward(){
  digitalWrite(right_dir,HIGH);
  analogWrite(right_pwm,(255-OutputR));
}

void stopL(){
  analogWrite(left_pwm,OutputL);      
  digitalWrite(left_dir, LOW);
 
}
void stopR(){
   analogWrite(right_pwm,OutputR);
  digitalWrite(right_dir,LOW);
  }



void setup()
{
  Serial.begin(57600);//Initialize the serial t
  pinMode(encoder0pinA, INPUT_PULLUP);
  pinMode(encoder0pinB, INPUT_PULLUP);
  pinMode(encoder1pinA, INPUT_PULLUP);
  pinMode(encoder1pinB, INPUT_PULLUP);
  EncoderInitL();//Initialize the module
  EncoderInitR();
  pinMode(left_pwm, OUTPUT); // Set left_pwm pin as output
  pinMode(left_dir, OUTPUT);
  pinMode(right_pwm, OUTPUT); // Set left_pwm pin as output
  pinMode(right_dir, OUTPUT);
  previousmillis = millis();
  LPID.SetMode(AUTOMATIC);
  RPID.SetMode(AUTOMATIC);
 
  nh.initNode();
  nh.subscribe(commands);
}

void loop()
{
  currentmillis = millis();
  if(currentmillis - previousmillis > 100){
          previousmillis = currentmillis;
          InputL = encoder_pulse_counterL/11070.00* 600.0;     // here *600 means 1 min = 60000 milli seconds here we are caluculating per 100 milli seconds and we divide it by 100.
          InputR = encoder_pulse_counterR/11070.00* 600.0;
          encoder_pulse_counterL = 0; //pulse count of left motor
          encoder_pulse_counterR = 0;
          LPID.Compute();
          RPID.Compute();
         
        left_Output = OutputL + old_pwm;
        right_Output = OutputR + old_pwm1;
        left_Output  = abs(constrain(left_pwm , 0, 255));
        right_Output = abs(constrain(right_pwm ,0, 255));
       

  }

  nh.spinOnce();
  old_pwm = left_Output;
  old_pwm1 = right_Output;
  delay(1);
}