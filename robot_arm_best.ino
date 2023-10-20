#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

//Fixed Variables
float base_height=0;                     //y value
float a=18;                               //link a
float b=18;                               //link b

//home parameters
float home_shoulder_angle=85;          //theta1
float home_elbow_angle=165;             //theta2
float home_hip_angle=45;                 //theta3

//object input variables
float object_dis=28;                    //object_dis value
float object_hip_angle=45;             //theta3

//object output variables
float object_shoulder_angle;           //theta1
float object_elbow_angle;              //theta2

float object_shoulder_angle_r;
float object_elbow_angle_r;
  
//target input variables
float target_dis=27;                   //object_dis value
float target_hip_angle=120;           //theta3

//target output variables
float target_shoulder_angle;          //theta1
float target_elbow_angle;             //theta2

float target_shoulder_angle_r;
float target_elbow_angle_r;

//gripper
float grip_angle=270;
float drop_angle=10;

void go_home_from_object(){
  delay(100);
  for (int i=object_shoulder_angle_r;i>home_shoulder_angle;i--){
  servo1.write(i);
  delay(10);
  }
  for (int j=object_elbow_angle_r;j>home_elbow_angle;j--){
  servo2.write(j);
  delay(10);
  }
  delay(100);

 
  
}

void go_home_from_target(){
  delay(100);
  for (int i=target_shoulder_angle_r;i>home_shoulder_angle;i--){
  servo1.write(i);
  delay(15);
  }
  for (int j=target_elbow_angle_r;j>home_elbow_angle;j--){
  servo2.write(j);
  delay(15);
  }
  delay(100);
 

}

void object_inverse_kinematics() {
  object_elbow_angle = acos((sq(object_dis)+ sq(base_height) - sq(a) - sq(b)) / (2*a*b));
  object_shoulder_angle= atan(base_height / object_dis) - atan((b*sin(object_elbow_angle)) / (a+ b*cos(object_elbow_angle)));

  object_shoulder_angle_r=195+(object_shoulder_angle*57.296);
  object_elbow_angle_r=165-((object_elbow_angle*57.296)-(object_shoulder_angle*57.296));

   
 servo3.write(object_hip_angle);
  delay(1000);


  for (int i=home_shoulder_angle;i<object_shoulder_angle_r;i++){
  servo1.write(i);
  delay(15);
  }
  for (int j=home_elbow_angle;j<object_elbow_angle_r;j++){
  servo2.write(j);
  delay(15);
  }
  

  Serial.println(-(object_shoulder_angle*57.296));
  Serial.println(object_elbow_angle*57.296);
}

void target_inverse_kinematics() {
  target_elbow_angle = acos((sq(target_dis)+ sq(base_height) - sq(a) - sq(b)) / (2*a*b));
  target_shoulder_angle= atan(base_height / target_dis) - atan((b*sin(target_elbow_angle)) / (a+ b*cos(target_elbow_angle)));

  target_shoulder_angle_r=195+(target_shoulder_angle*57.296);
  target_elbow_angle_r=165-((target_elbow_angle*57.296)-(target_shoulder_angle*57.296));

   
/*for (int k=home_hip_angle;k<target_hip_angle;k++){
  servo3.write(k);
  delay(15);
  }*/
  servo3.write(target_hip_angle);
  delay(1000);

  for (int i=home_shoulder_angle;i<target_shoulder_angle_r;i++){
  servo1.write(i);
  delay(15);
  }
  for (int j=home_elbow_angle;j<target_elbow_angle_r;j++){
  servo2.write(j);
  delay(15);
  }
  

}
void grip(){
 servo4.write(grip_angle);
}
void drop(){
  servo4.write(drop_angle);
}
void setup() {
  //SERVO SETUP
  servo1.attach(3);
  servo2.attach(4);
  servo3.attach(5);
  servo4.attach(6);

  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  drop();
  delay(1000);
  object_inverse_kinematics();
  delay(1000);
  grip();
  delay(1000);
  go_home_from_object();
  delay(2000);
  target_inverse_kinematics();
  delay(1000);
  drop();
  delay(1000);
  go_home_from_target();
}
