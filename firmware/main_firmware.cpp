/*
   rosserial Planar Odometry Example
*/
// #define USE_USBCON
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
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

#include <Arduino.h>

/*
   ------------general variables--
*/


//servo pin setup
int fwdServoPin = 6;
int turnServoPin = 7;
int fwdThrust = 90;
int turnThrust = 90;
Servo fwdMotor;
Servo turnMotor;

//localizaton varialbles
float head = 0;
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

//clock definitions
unsigned long cycles = 0;
unsigned long start;
int checkByte;
bool givePosition;

/*
   ------------general variables--
*/


/*
    ---------function declerations--
*/

//function definitions
void rightHandlerA();   //functions for encoder interrupt handlers
void rightHandlerB();
void leftHandlerA();
void leftHandlerB();

inline double degToRad(double deg);

float map_func(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
/*
   ---------function declerations--
*/

/*
    ------------- ROS ------------
*/


ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
geometry_msgs::Pose2D pose2d;
tf::TransformBroadcaster broadcaster;

double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link_name[] = "/kezbot/base_link";
char odom_name[] = "/kezbot/base";

ros::Publisher ros_odom("/kezbot/pose2d", &pose2d);



/*
    reads cmd_vel topic then turn the motors
*/
void cmd_vel_handle( const geometry_msgs::Twist& msg) {
  float fwd_message = msg.linear.x;
  float turn_message = msg.angular.z;

  fwdMotor.write(
    int(map_func(fwd_message, -1.3, 1.3 , 20 , 160))
  );

//  seg.print(int(map_func(fwd_message, -1.3, 1.3 , 0 , 10))%10);

  turnMotor.write(
    int(map_func(turn_message, -2.5, 2.5 , 20 , 160))
  );

}

ros::Subscriber<geometry_msgs::Twist> sub("/kezbot/cmd_vel", &cmd_vel_handle );

/*
    ------------- ROS ------------
*/




void setup()
{



  digitalWrite(13, LOW);
  //rosnode initialize
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(sub);
  nh.advertise(ros_odom);

  //encoder setup for odometer
  rightEncoder.attach(encoderRA, encoderRB);
  leftEncoder.attach(encoderLA, encoderLB);
  leftEncoder.initialize();
  rightEncoder.initialize();
  attachInterrupt(leftEncoder.greenCablePin, leftHandlerA , CHANGE);
  attachInterrupt(leftEncoder.yellowCablePin, leftHandlerB , CHANGE);
  attachInterrupt(rightEncoder.greenCablePin, rightHandlerA , CHANGE);
  attachInterrupt(rightEncoder.yellowCablePin, rightHandlerB , CHANGE);


  //reset bno055
  pinMode(bnoRstPin, OUTPUT);
  digitalWrite(bnoRstPin, LOW);
  delay(500);
  digitalWrite(bnoRstPin, HIGH);
  delay(500);

  //initialize bno055 absolute orientation sensor
  while (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    nh.logerror("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");

    /*
      your solution here
      give rst pin ground and 3.3 volts
    */

    delay(100);

  }
  bno.setExtCrystalUse(true);
  delay(100);

  //PWM motor controller setup
  fwdMotor.attach(fwdServoPin);
  turnMotor.attach(turnServoPin);

  turnMotor.write(90);
  fwdMotor.write(90);

  start = millis();

}

void loop()
{
  digitalWrite(13, HIGH);
  cycles =  millis() - start;
  start = millis();

  //get values from encoders for odometer
  hold = odom.getWay();
  dWay = hold - ddWay;
  ddWay = hold;
  sensors_event_t event;
  bno.getEvent(&event);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  yVector = yVector - dWay * sin(degToRad(event.orientation.x));
  xVector = xVector + dWay * cos(degToRad(event.orientation.x));

  //correct the orientation
  pose2d.x = xVector / 10.0;
  pose2d.y = yVector / 10.0;
  pose2d.theta = degToRad(map_func(event.orientation.x , 0 , 360 , 360 , 0));
  ros_odom.publish( &pose2d );

  // tf odom->base_link
  t.header.frame_id = base_link_name;
  t.child_frame_id = odom_name;

  t.transform.translation.x = xVector / 10.0;
  t.transform.translation.y = yVector / 10.0;

  t.transform.rotation = tf::createQuaternionFromYaw(
                           degToRad(map_func(event.orientation.x , 0 , 360 , 360 , 0))
                         );
  t.header.stamp = nh.now();

  broadcaster.sendTransform(t);

  nh.spinOnce();

  delay(2);
}


/* user defined functions bodies*/

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
