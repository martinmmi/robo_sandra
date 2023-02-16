#include <Arduino.h>
#include <Servo.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

std_msgs::UInt16 msg_m;
std_msgs::UInt16 msg_e;
std_msgs::UInt16 msg_l;


Servo servo;

int min_angle = 650;
int max_angle = 2100;
int rightDistance = 0, leftDistance = 0, middleDistance = 0;
int echoPin = 3;               
int trigPin = 2;               
int distance = 0;
int average = 0;
int stopDistance = 20;          //Change
int stopDistanceShort = 8;      //Change
int backDistance = 20;          //Change
int i = 0;

#define SERVO      4            //Servo Pin         
#define IN1        8            
#define IN2        6            
#define IN3       11            
#define IN4        5            
#define LT_R      !digitalRead(A4)  
#define LT_L      !digitalRead(A2)    

bool activeLineSensor = false;
bool activeEyeSensor = false;

long lastForward = 0;
long lastBack = 0;
long lastLeft = 0;
long lastRight = 0;
long lastStop = 0;
long lastDistance = 0;
long lastSetSensorBack = 0;
long lastPublished = 0;

///////////////////////////////////////////////////////////////

int updateDistance() {

  distance = 0;
  average = 0;

  for (int i = 0; i < 2; i++) {     //Build an Average

    digitalWrite(trigPin, LOW);     //0v
    delayMicroseconds(2);           //Emit 40 kHz sound
    digitalWrite(trigPin, HIGH);    //5v
    delayMicroseconds(10);          //for 10 microseconds
    digitalWrite(trigPin, LOW);   

    distance = pulseIn(echoPin, HIGH);  //detect a pulse 

    distance = distance / 74 / 2 * 2.54;  //Speed of sound 13511.81 inches/s
                                          //13511.81/10^6 inches per micro
    average += distance;                  //0.01351181 inches per microsecond
                                          //74.0092341 microseconds per inch
    delay(10);                             //dividing by 74 and dividing by 2 and multiple by 2.54 for cm
  }
  distance = average / 2;
  return distance;

}  

///////////////////////////////////////////////////////////////

void measureMiddleDistance () {

  if ((millis() - lastDistance > 1000)) {
    Serial.print("D: ");
    Serial.print(middleDistance);
    Serial.println(" cm");
    lastDistance = millis();
  }

}

///////////////////////////////////////////////////////////////

void forward(){ 

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  if ((millis() - lastForward > 1000)) {
      Serial.println("F");
      lastForward = millis();
  }
}

void back() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  if ((millis() - lastBack > 1000)) {
      Serial.println("B");
      lastBack = millis();
  }
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 

  if ((millis() - lastLeft > 1000)) {
      Serial.println("L");
      lastLeft = millis();
  }
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  if ((millis() - lastRight > 1000)) {
      Serial.println("R");
      lastRight = millis();
  }
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  if ((millis() - lastStop > 1000)) {
    Serial.println("S");
    lastStop = millis();
  }
} 

///////////////////////////////////////////////////////////////

void eyeFunction () {

  middleDistance = updateDistance();
  measureMiddleDistance();

  msg_e.data = middleDistance;

  if(middleDistance <= stopDistance) {      //Stop if something in the way
    stop();

    servo.write(10);
    delay(500);
    rightDistance = updateDistance();

    servo.write(180);
    delay(1000);
    leftDistance = updateDistance();

    servo.write(90);
    delay(500);
    
    if(rightDistance > leftDistance) {
      right();
      delay(600 + random(800));
    }
    else if(rightDistance < leftDistance) {
      left();
      delay(600 + random(800));
    }
    else if((rightDistance <= backDistance) || (leftDistance <= backDistance)) {
      back();
      delay(300 + random(400));
    }
  }
  
  if(middleDistance > stopDistance){
    forward();
    delay(300 + random(150));
  }
}

///////////////////////////////////////////////////////////////

void lineFunction () {

  middleDistance = updateDistance();
  measureMiddleDistance();

  msg_e.data = middleDistance;

  if(middleDistance <= stopDistanceShort) {      //Stop if something in the way
    stop();
  }

  if(middleDistance >= stopDistanceShort) {      

    if(LT_R) { 
      msg_l.data = LT_R;
      right();
      while(LT_R);                           
    }

    else if(LT_L) {
      msg_l.data = LT_L;
      left();
      while(LT_L);
    }
  }
}

///////////////////////////////////////////////////////////////

void subscriberCallback(const std_msgs::UInt16& msg_m) {

  if (msg_m.data == 0) {
    activeEyeSensor = false;
    activeLineSensor = false;
    stop();
  }
  
  if (msg_m.data == 1) {
    activeEyeSensor = true;
    activeLineSensor = false;
  }

  if (msg_m.data == 2) {
    activeEyeSensor = false;
    activeLineSensor = true;
  }

}

///////////////////////////////////////////////////////////////

ros::Subscriber<std_msgs::UInt16> sub_m("mode", &subscriberCallback);
ros::Publisher pub_e("eye", &msg_e);
ros::Publisher pub_l("line", &msg_l);

///////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);

  Serial.println("");
  Serial.println("ROBO!");

  servo.attach(SERVO,min_angle,max_angle);
  servo.write(90);

  nh.initNode();
  nh.subscribe(sub_m);
  nh.advertise(pub_e);
  nh.advertise(pub_l);

  pinMode(echoPin, INPUT);    
  pinMode(trigPin, OUTPUT);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);


  pinMode(LT_R,INPUT);
  pinMode(LT_L,INPUT);

  delay(1500);

}

///////////////////////////////////////////////////////////////

void loop() {

  if (activeEyeSensor == true) {
    pub_e.publish(&msg_e);
    eyeFunction();

  }

  if (activeLineSensor == true) {
    pub_e.publish(&msg_e);
    pub_l.publish(&msg_l);
    lineFunction();

  }

  nh.spinOnce();

}