#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <Servo.h>
 Servo myservo;

#define LOOPTIME                      100    
const byte noCommLoopMax = 10;                
unsigned int noCommLoops = 0;                 

double speed_cmd_left2 = 0;      

const int PIN_ENCOD_A_MOTOR_LEFT = 2;                                 
const int PIN_ENCOD_B_MOTOR_LEFT = 4;               
const int PIN_ENCOD_A_MOTOR_RIGHT = 3;                  
const int PIN_ENCOD_B_MOTOR_RIGHT = 5;              

const int PIN_SIDE_LIGHT_LED = 46;                  

unsigned long lastMilli = 0;


const double radius = 0.04;                  
const double wheelbase = 0.187;              
const double encoder_cpr = 990 ;               
const double speed_to_pwm_ratio = 0.00235 ;   
const double min_speed_cmd = 0.0882  ;        

double speed_req = 0;                         
double angular_speed_req = 0;             

double speed_req_left = 0;                    
double speed_act_left = 0;                   
double speed_cmd_left = 0;                

double speed_req_right = 0;                  
double speed_act_right = 0;                  
double speed_cmd_right = 0;                  
                        
const double max_speed = 0.4;              

int PWM_leftMotor = 0;                     
int PWM_rightMotor = 0;                   
int angle =90;  
int angleStep =10;                                                      
// PID Parameters
const double PID_left_param[] = { 0, 0, 0.1 }; 
const double PID_right_param[] = { 0, 0, 0.1 }; 

volatile float pos_left = 0;      
volatile float pos_right = 0;      

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);        
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);     
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);    
  
ros::NodeHandle nh;

void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  
  
  speed_req = cmd_vel.linear.x;                                    

  angular_speed_req = cmd_vel.angular.z;                            
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);    
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);  
geometry_msgs::Vector3Stamped speed_msg;                               
ros::Publisher speed_pub("speed", &speed_msg);                          
const int lightIncNumber = 30;                                                                                                                                      
int lightInc = 0;                                                                                                                                                    
int lightValue [lightIncNumber]= { 10, 40, 80, 160, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 160, 80, 40, 10, 0, 0, 0, 0, 0, 0, 0, 0 };
int lightValueNoComm [25]= { 255, 0, 255, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 
int lightT = 0; //init light period


void setup() {

  
  pinMode(PIN_SIDE_LIGHT_LED, OUTPUT);      
  analogWrite(PIN_SIDE_LIGHT_LED, 255);     
  
  nh.initNode();                            
  nh.getHardware()->setBaud(57600);         
  nh.subscribe(cmd_vel);                 
  nh.advertise(speed_pub);               
  AFMS.begin();
  

  leftMotor->setSpeed(0);
  leftMotor->run(BRAKE);
  rightMotor->setSpeed(0);
  rightMotor->run(BRAKE);
 

  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
    

  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);              
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);


  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);               
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(1, encoderRightMotor, RISING);


    Serial.begin(9600);         
  myservo.attach(30); 
  pinMode(2,INPUT_PULLUP);
   Serial.println("Servo Button ");
}



void loop() {
  nh.spinOnce();
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           
    lastMilli = millis();
    
    if (!nh.connected()){
      analogWrite(PIN_SIDE_LIGHT_LED, lightValueNoComm[lightInc]);
      lightInc=lightInc+1;
      if (lightInc >= 25){
        lightInc=0;
      }
    }
    else{
      analogWrite(PIN_SIDE_LIGHT_LED, lightValue [lightInc]);
      lightT = 3000 - ((2625/max_speed)*((abs(speed_req_left)+abs(speed_req_right))/2));
      lightInc=lightInc+(30/(lightT/LOOPTIME));
      if (lightInc >= lightIncNumber){
        lightInc=0;
      }
    }
    
    
    if (abs(pos_left) < 5){                                                  
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          
    }
    
    if (abs(pos_right) < 5){                                                
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;       
    }
    
    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 

    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); 
    
    if (noCommLoops >= noCommLoopMax) {                  
      leftMotor->setSpeed(0);
      leftMotor->run(BRAKE);
    }
    else if (speed_req_left == 0){                        
      leftMotor->setSpeed(0);
      leftMotor->run(BRAKE);
    }
    else if (PWM_leftMotor > 0){                          
      leftMotor->setSpeed(abs(PWM_leftMotor));
      leftMotor->run(BACKWARD);
    }
    else {                                               
      leftMotor->setSpeed(abs(PWM_leftMotor));
      leftMotor->run(FORWARD);
    }
    
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 
   
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); 

    if (noCommLoops >= noCommLoopMax) {                   
      rightMotor->setSpeed(0);
      rightMotor->run(BRAKE);
    }
    else if (speed_req_right == 0){                       
      rightMotor->setSpeed(0);
      rightMotor->run(BRAKE);
    }
    else if (PWM_rightMotor > 0){                         
      rightMotor->setSpeed(abs(PWM_rightMotor));
      rightMotor->run(FORWARD);
    }
    else {                                                
      rightMotor->setSpeed(abs(PWM_rightMotor));
      rightMotor->run(BACKWARD);
    }

    if((millis()-lastMilli) >= LOOPTIME){        
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    
    publishSpeed(LOOPTIME);   
  }
   while(digitalRead(29) == LOW){
 
  angle = angle + angleStep;

    
    if (angle <= 0 || angle >= 180) {
      angleStep = -angleStep;
    }
    myservo.write(angle); 
      Serial.print("Moved to: ");
      Serial.print(angle);  
      Serial.println(" degree");    
  delay(100);
  }
 }

void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      
  speed_msg.vector.x = speed_act_left;    
  speed_msg.vector.y = speed_act_right;  
  speed_msg.vector.z = time/1000;        
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}


void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  else pos_left--;
}


void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
  else pos_right++;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
