#include <QTRSensors.h>
#define Kp 0.08// start with this slowly by increasing from 1. If at 1 it oveshoots decrease the value
#define Kd 3 // adjust this for varying speeds to reduce woobling
#define Ki 0
#define rightMaxSpeed 220 // max speed of the robot
#define leftMaxSpeed 220 // max speed of the robot
#define rightBaseSpeed 180 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 180 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define rightMotor1 8
#define rightMotor2 9
#define rightMotorPWM 10
#define leftMotor1 6
#define leftMotor2 7
#define leftMotorPWM 5

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
void setup()
{
pinMode(rightMotor1, OUTPUT);
pinMode(rightMotor2, OUTPUT);
pinMode(rightMotorPWM, OUTPUT);
pinMode(leftMotor1, OUTPUT);
pinMode(leftMotor2, OUTPUT);
pinMode(leftMotorPWM, OUTPUT);
qtr.setTypeRC();
qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,12,13}, SensorCount);
//qtr.setEmitterPin(2);
// 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
// = ~25 ms per calibrate() call.
// Call calibrate() 400 times to make calibration take about 10 seconds.
for (uint16_t i = 0; i < 400; i++)
{
qtr.calibrate();
}
digitalWrite(LED_BUILTIN, LOW);
Serial.begin(9600);
delay(4000); // wait for 4s to position the bot before entering the main loop
}
int lastError = 0;
int I=0;
int P;
int D;
void loop()
{
uint16_t position = qtr.readLineBlack(sensorValues);
int error = position - 3500;
Serial.println(error);
P = error;
D = error - lastError;
I = I+P;
int PIDvalue = (Kp * P)+(Kd *D)+ (Ki *I) ;
lastError = error;
int rightMotorSpeed= rightBaseSpeed + PIDvalue;
int leftMotorSpeed = leftBaseSpeed - PIDvalue;
if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
{
digitalWrite(rightMotor1, HIGH);
digitalWrite(rightMotor2, LOW);
analogWrite(rightMotorPWM, rightMotorSpeed);
digitalWrite(leftMotor1, HIGH);
digitalWrite(leftMotor2, LOW);
analogWrite(leftMotorPWM, leftMotorSpeed);
}
}
