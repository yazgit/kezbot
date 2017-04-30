#define USE_USBCON
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <Wire.h>
#include <QuadratureEncoder.h>
#include <Odometer.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

//servo pin setup
int fwdServoPin = 6;
int turnServoPin = 7;
int fwdThrust = 90;
int turnThrust = 90;
Servo fwdMotor;
Servo turnMotor;

//localizaton varialbles
float xVector = 0;
float yVector = 0;
float dWay = 0;
float ddWay = 0;
float hold = 0;

//bno055 setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> gyro;
int bnoRstPin = 26;

//encoderPinSetup
int encoderLA = 36;
int encoderLB = 38;
int encoderRA = 40;
int encoderRB = 42;

//object definitions
Encoder leftEncoder(LEFT);
Encoder rightEncoder(RIGHT);
Odometer odom;

//function definitions
void rightHandlerA();   //functions for encoder interrupt handlers
void rightHandlerB();
void leftHandlerA();
void leftHandlerB();

inline double degToRad(double deg);
float map_func(float value, float inMin, float inMax, float outMin, float outMax);
void reset_BNO055();


/*
    ------------- ROS ------------
*/


ros::NodeHandle  nh;
geometry_msgs::Pose2D pose2d;

double x = 1.0;
double y = 0.0;
double theta = 1.57;


void cmd_vel_handle( const geometry_msgs::Twist& msg);

ros::Publisher pose2d_publisher("/kezbot/pose2d", &pose2d);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_subscriber("/kezbot/cmd_vel", &cmd_vel_handle );

/*
    ------------- ROS ------------
*/




void setup()
{

  digitalWrite(13, LOW);
  pinMode(bnoRstPin, OUTPUT);

  //rosnode initialize
  nh.initNode();
  nh.subscribe(cmd_vel_subscriber);
  nh.advertise(pose2d_publisher);

  //encoder setup for odometer
  rightEncoder.attach(encoderRA, encoderRB);
  leftEncoder.attach(encoderLA, encoderLB);
  leftEncoder.initialize();
  rightEncoder.initialize();
  attachInterrupt(digitalPinToInterrupt(leftEncoder.greenCablePin), leftHandlerA , CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoder.yellowCablePin), leftHandlerB , CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder.greenCablePin), rightHandlerA , CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder.yellowCablePin), rightHandlerB , CHANGE);

  reset_BNO055();

  //initialize bno055 absolute orientation sensor
  while (!bno.begin()) {
    nh.logerror("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    reset_BNO055();
  }

  bno.setExtCrystalUse(true);
  delay(100);

  //PWM motor controller setup
  fwdMotor.attach(fwdServoPin);
  turnMotor.attach(turnServoPin);

  turnMotor.write(90);
  fwdMotor.write(90);

}

void loop()
{

  //get values from encoders for odometer
  hold = odom.getWay();
  dWay = hold - ddWay;
  ddWay = hold;
  sensors_event_t event;
  bno.getEvent(&event);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  yVector = yVector + dWay * sin(degToRad(event.orientation.x));
  xVector = xVector + dWay * cos(degToRad(event.orientation.x));

  //correct the orientation
  pose2d.x = xVector / 10.0;
  pose2d.y = yVector / 10.0;
  pose2d.theta = degToRad(map_func(event.orientation.x , 0 , 360 , 360 , 0));
  pose2d_publisher.publish( &pose2d );

  nh.spinOnce();

  delay(2);
}


void rightHandlerA() {
  rightEncoder.handleInterruptGreen();
  odom.rightEncoderTick = rightEncoder.encoderTicks;
}
void rightHandlerB() {
  rightEncoder.handleInterruptYellow();
  odom.rightEncoderTick = rightEncoder.encoderTicks;
}

void leftHandlerA() {
  leftEncoder.handleInterruptGreen();
  odom.leftEncoderTick = leftEncoder.encoderTicks;
}
void leftHandlerB() {
  leftEncoder.handleInterruptYellow();
  odom.leftEncoderTick = leftEncoder.encoderTicks;
}

inline double degToRad(double deg) {
  return deg * M_PI / 180.0;
}

float map_func(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void cmd_vel_handle( const geometry_msgs::Twist& msg) {
  float fwd_message = msg.linear.x;
  float turn_message = msg.angular.z;

  fwdMotor.write(
    int(map_func(fwd_message, -1.3, 1.3 , 20 , 160))
  );
  turnMotor.write(
    int(map_func(turn_message, -2.5, 2.5 , 20 , 160))
  );
}

void reset_BNO055(){
    digitalWrite(bnoRstPin, LOW);
    delay(100);
    digitalWrite(bnoRstPin, HIGH);
    delay(100);
}
