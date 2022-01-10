#include <ros.h>
#include <geometry_msgs/Twist.h>


#define CORRECTION_FACTOR 0.8942
#define MOTOR_GEAR_RATIO 21.3
#define NO_OF_SIGNALS 11
#define WHEEL_DIAMETER 0.065
#define R (WHEEL_DIAMETER / 2)
#define L 0.2405
#define ENCODER_PIN_LEFT 2
#define ENCODER_PIN_RIGHT 3
#define DIRECTION_PIN_LEFT 4
#define DIRECTION_PIN_RIGHT 7
#define PPR (MOTOR_GEAR_RATIO * NO_OF_SIGNALS)
#define ENA 5
#define ENB 6
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

volatile long leftCount = 0;
volatile long rightCount = 0;
unsigned long oldMillis = 0;

float desAngSpeedLeft = 0;
float desAngSpeedRight = 0;
float angularSpeedLeft = 0;
float angularSpeedRight = 0;

int leftPWM, rightPWM;
float v_actual = 0, omega_actual = 0;


ros::NodeHandle nh;

geometry_msgs::Twist twist_msg;
ros::Publisher twistPublisher("/encoder_twist", &twist_msg);

void twistCallback(const geometry_msgs::Twist &twist_msg)
{
  float v = twist_msg.linear.x;
  float omega = twist_msg.angular.z;
  setWheelAngularVel(v, omega, desAngSpeedRight, desAngSpeedLeft);
  leftPWM = calcLeftSpeed(desAngSpeedLeft);
  rightPWM = calcRightSpeed(desAngSpeedRight);
}

ros::Subscriber<geometry_msgs::Twist> velocitySub ("/motor_command", twistCallback);

void setup() 
{
  // put your setup code here, to run once:
  pinMode (ENCODER_PIN_LEFT, INPUT_PULLUP);
  pinMode (ENCODER_PIN_RIGHT, INPUT_PULLUP);
  pinMode (DIRECTION_PIN_LEFT, INPUT_PULLUP);
  pinMode (DIRECTION_PIN_RIGHT, INPUT_PULLUP);
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_LEFT), leftEncoderCount, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_RIGHT), rightEncoderCount, FALLING);

  nh.initNode();
  nh.subscribe(velocitySub);
  nh.advertise(twistPublisher);

  oldMillis = millis();
}

void loop() 
{
  // put your main code here, to run repeatedly:

  if (millis() - oldMillis > 25)
  {
    angularSpeedLeft = ((float(leftCount) / PPR) * 1000 * 2 * PI) / (millis() - oldMillis);
    angularSpeedLeft /= CORRECTION_FACTOR;
    leftCount = 0;   

    angularSpeedRight = ((float(rightCount) / PPR) * 1000 * 2 * PI) / (millis() - oldMillis);
    angularSpeedRight /= CORRECTION_FACTOR;
    rightCount = 0;   
    oldMillis = millis();

    setActualTwist(v_actual, omega_actual);
    twist_msg.linear.x = v_actual;
    twist_msg.angular.z = omega_actual;

    twistPublisher.publish(&twist_msg);
    nh.spinOnce();
    correctSpeeds();
  }
  
  moveMotors(leftPWM, rightPWM);
 
}

void leftEncoderCount()
{
  if (digitalRead(ENCODER_PIN_LEFT) != digitalRead(DIRECTION_PIN_LEFT))
  {
    leftCount++;
  }
  else
  {
    leftCount--;
  }
}

void rightEncoderCount()
{
  if (digitalRead(ENCODER_PIN_RIGHT) == digitalRead(DIRECTION_PIN_RIGHT))
  {
    rightCount++;
  }
  else
  {
    rightCount--;
  }
}

int calcLeftSpeed(float desiredAngularSpeed)
{
  int SIGN = desiredAngularSpeed >= 0? 1: -1;
  desiredAngularSpeed = abs(desiredAngularSpeed);

  if (desiredAngularSpeed < 3.375 && desiredAngularSpeed > 0)
    return 107 * SIGN;
    
  if (desiredAngularSpeed == 0)
    return 0;
    
  float lowerPWM, upperPWM, lowerAngSpeed, upperAngSpeed;

  if (desiredAngularSpeed > 3.375 && desiredAngularSpeed < 4.077)
  {
    lowerPWM = 107, upperPWM = 110, lowerAngSpeed = 3.375, upperAngSpeed = 4.077;
  }

  else if (desiredAngularSpeed < 5.625)
  {
    lowerPWM = 110, upperPWM = 120, lowerAngSpeed = 4.077, upperAngSpeed = 5.625;
  }

  else if (desiredAngularSpeed < 7.125)
  {
    lowerPWM = 120, upperPWM = 130, lowerAngSpeed = 5.625, upperAngSpeed = 7.125;
  }

  else if (desiredAngularSpeed < 8.737)
  {
    lowerPWM = 130, upperPWM = 140, lowerAngSpeed = 7.125, upperAngSpeed = 8.737;
  }

  else if (desiredAngularSpeed < 10.033)
  {
    lowerPWM = 140, upperPWM = 150, lowerAngSpeed = 8.737, upperAngSpeed = 10.033;
  }

  else if (desiredAngularSpeed < 11.210)
  {
    lowerPWM = 150, upperPWM = 160, lowerAngSpeed = 10.033, upperAngSpeed = 11.210;
  }

  else if (desiredAngularSpeed < 12.267)
  {
    lowerPWM = 160, upperPWM = 170, lowerAngSpeed = 11.210, upperAngSpeed = 12.267;
  }

  else if (desiredAngularSpeed < 13.220)
  {
    lowerPWM = 170, upperPWM = 180, lowerAngSpeed = 12.267, upperAngSpeed = 13.220;
  }

  else if (desiredAngularSpeed < 14.303)
  {
    lowerPWM = 180, upperPWM = 190, lowerAngSpeed = 13.220, upperAngSpeed = 14.303;
  }

  else if (desiredAngularSpeed < 15.230)
  {
    lowerPWM = 190, upperPWM = 200, lowerAngSpeed = 14.303, upperAngSpeed = 15.230;
  }

  else if (desiredAngularSpeed < 16.030)
  {
    lowerPWM = 200, upperPWM = 210, lowerAngSpeed = 15.230, upperAngSpeed = 16.030;
  }

  else if (desiredAngularSpeed < 17.023)
  {
    lowerPWM = 210, upperPWM = 220, lowerAngSpeed = 16.030, upperAngSpeed = 17.023;
  }

  else if (desiredAngularSpeed < 18.080)
  {
    lowerPWM = 220, upperPWM = 230, lowerAngSpeed = 17.023, upperAngSpeed = 18.080;
  }

  else if (desiredAngularSpeed < 19.553)
  {
    lowerPWM = 230, upperPWM = 240, lowerAngSpeed = 18.080, upperAngSpeed = 19.553;
  }

  else if (desiredAngularSpeed < 21.800)
  {
    lowerPWM = 240, upperPWM = 250, lowerAngSpeed = 19.553, upperAngSpeed = 21.800;
  }
  
  else
  {
    return 255 * SIGN;
  }
  

  float desiredPWM = lowerPWM + ((upperPWM - lowerPWM) * ((desiredAngularSpeed - lowerAngSpeed) / (upperAngSpeed - lowerAngSpeed)));

  return int(round(desiredPWM)) * SIGN;
}

int calcRightSpeed(float desiredAngularSpeed)
{
  int SIGN = desiredAngularSpeed >= 0? 1: -1;
  desiredAngularSpeed = abs(desiredAngularSpeed);

  if (desiredAngularSpeed < 2.875 && desiredAngularSpeed > 0)
    return 107 * SIGN;
    
  if (desiredAngularSpeed == 0)
    return 0;
    
  float lowerPWM, upperPWM, lowerAngSpeed, upperAngSpeed;

  if (desiredAngularSpeed > 2.875 && desiredAngularSpeed < 3.580)
  {
    lowerPWM = 107, upperPWM = 110, lowerAngSpeed = 2.875, upperAngSpeed = 3.580;
  }

  else if (desiredAngularSpeed < 4.945)
  {
    lowerPWM = 110, upperPWM = 120, lowerAngSpeed = 3.580, upperAngSpeed = 4.945;
  }

  else if (desiredAngularSpeed < 6.320)
  {
    lowerPWM = 120, upperPWM = 130, lowerAngSpeed = 4.945, upperAngSpeed = 6.320;
  }

  else if (desiredAngularSpeed < 8.073)
  {
    lowerPWM = 130, upperPWM = 140, lowerAngSpeed = 6.320, upperAngSpeed = 8.073;
  }

  else if (desiredAngularSpeed < 9.483)
  {
    lowerPWM = 140, upperPWM = 150, lowerAngSpeed = 8.073, upperAngSpeed = 9.483;
  }

  else if (desiredAngularSpeed < 10.470)
  {
    lowerPWM = 150, upperPWM = 160, lowerAngSpeed = 9.483, upperAngSpeed = 10.470;
  }

  else if (desiredAngularSpeed < 11.510)
  {
    lowerPWM = 160, upperPWM = 170, lowerAngSpeed = 10.470, upperAngSpeed = 11.510;
  }

  else if (desiredAngularSpeed < 12.570)
  {
    lowerPWM = 170, upperPWM = 180, lowerAngSpeed = 11.510, upperAngSpeed = 12.570;
  }

  else if (desiredAngularSpeed < 13.773)
  {
    lowerPWM = 180, upperPWM = 190, lowerAngSpeed = 12.570, upperAngSpeed = 13.773;
  }

  else if (desiredAngularSpeed < 14.627)
  {
    lowerPWM = 190, upperPWM = 200, lowerAngSpeed = 13.773, upperAngSpeed = 14.627;
  }

  else if (desiredAngularSpeed < 15.520)
  {
    lowerPWM = 200, upperPWM = 210, lowerAngSpeed = 14.627, upperAngSpeed = 15.520;
  }

  else if (desiredAngularSpeed < 16.467)
  {
    lowerPWM = 210, upperPWM = 220, lowerAngSpeed = 15.520, upperAngSpeed = 16.467;
  }

  else if (desiredAngularSpeed < 17.493)
  {
    lowerPWM = 220, upperPWM = 230, lowerAngSpeed = 16.467, upperAngSpeed = 17.493;
  }

  else if (desiredAngularSpeed < 19.083)
  {
    lowerPWM = 230, upperPWM = 240, lowerAngSpeed = 17.493, upperAngSpeed = 19.083;
  }

  else if (desiredAngularSpeed < 21.247)
  {
    lowerPWM = 240, upperPWM = 250, lowerAngSpeed = 19.083, upperAngSpeed = 21.247;
  }

  else
  {
    return 255 * SIGN;
  }

  float desiredPWM = lowerPWM + ((upperPWM - lowerPWM) * ((desiredAngularSpeed - lowerAngSpeed) / (upperAngSpeed - lowerAngSpeed)));

  return int(round(desiredPWM)) * SIGN;  
}

void moveMotors(int leftSpeed, int rightSpeed)
{
  if (leftSpeed == 0 && rightSpeed == 0)
  {
    stopNow();
    return;
  }
  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));

  if (leftSpeed >= 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  if (rightSpeed >= 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void stopNow()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void correctSpeeds()
{
  float k = 1;
  float errorLeft = desAngSpeedLeft - angularSpeedLeft;
  float errorRight = desAngSpeedRight - angularSpeedRight;

  leftPWM = constrain(int(round(float(leftPWM) + (errorLeft * k))), -255, 255);
  rightPWM = constrain(int(round(float(rightPWM) + (errorRight * k))), -255, 255);
}

void setWheelAngularVel(float v, float omega, float &rightAngularSpeed, float &leftAngularSpeed)
{
  rightAngularSpeed = ((2 * v) + (omega * L)) / (WHEEL_DIAMETER);
  leftAngularSpeed = ((2 * v) - (omega * L)) / (WHEEL_DIAMETER);
}

float setActualTwist(float &v_actual, float &omega_actual)
{
  v_actual = (R * (angularSpeedRight + angularSpeedLeft)) / 2;
  omega_actual = (R * (angularSpeedRight - angularSpeedLeft)) / L;
}
