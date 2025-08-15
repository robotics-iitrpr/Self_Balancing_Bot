#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Motor driver pins
const int PWMA = 5;
const int AIN1 = 8;
const int AIN2 = 7;
const int PWMB = 6;
const int BIN1 = 9;
const int BIN2 = 10;
const int STBY = 4;

// PID and control variables (initial defaults)
float Kp = 16.0;
float Kd = 0.5; // very small to prevent runaway
float Ki = 0.01;

int motorMax = 229;
int motorMin = 25;

float startAngle = 60.0;
float iLimit = 200.0;

float angleOffset = 0.0;
int ANGLE_SIGN = +1;
bool USE_EULER_PITCH = true;

float setpoint = 2.0;
float angle = 0.0;
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;

unsigned long lastMicros = 0;

// Button for setting offset
int buttonPin = 11;
int buttonVal = 1;
float tolerance = 0.0; // degrees
void motorDrive(int pwmLeft, int pwmRight)
{
  int spdL = abs(pwmLeft);
  if (spdL > 255)
    spdL = 255;
  if (pwmLeft > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (pwmLeft < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, spdL);

  int spdR = abs(pwmRight);
  if (spdR > 255)
    spdR = 255;
  if (pwmRight > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if (pwmRight < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, spdR);
}

void motorsBrake()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}

void setupPins()
{
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  motorsBrake();
}

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP); // Button
  setupPins();
  Serial.begin(115200);
  Wire.begin();

  if (!bno.begin())
  {
    // Serial.println("BNO055 not detected. Check wiring/address.");
    while (1)
    {
      delay(10);
    }
  }
  bno.setExtCrystalUse(true);

  lastMicros = micros();
  // Serial.println("Ready. Press button to set offset.");
}

void loop()
{

  // Button press to set angle offset
  buttonVal = digitalRead(buttonPin);
  if (buttonVal == LOW)
  { // Button pressed
    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    angleOffset = e.y();
    setpoint = 0; // Keep setpoint at upright
    // Serial.print("Offset set to: ");
    // Serial.println(angleOffset);
    delay(300); // debounce
  }

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0)
    dt = 0.001f;
  lastMicros = now;

  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  angle = ANGLE_SIGN * (e.y() - angleOffset);
  float angVel = g.y(); // deg/s
  float absAngle = fabs(angle);
  if (angle > -5 && angle < 5)
  {
    motorsBrake();
  }

  // ==== Dynamic PID + motor limits ====
  else if (absAngle > 20)
  {
    motorMax = 200;
    motorMin = 40;
    startAngle = 50.0;
    iLimit = 50.0;
    Kp = 20;
    Ki = 0.05;
    Kd = 0.6;
  }
  else if (absAngle > 5)
  {
    motorMax = 150;
    motorMin = 30;
    startAngle = 45.0;
    iLimit = 100.0;
    Kp = 16;
    Ki = 0.05;
    Kd = 0.3;
  }
  else
  {
    motorMax = 120;
    motorMin = 20;
    startAngle = 40.0;
    iLimit = 175.0;
    Kp = 12;
    Ki = 0.05;
    Kd = 0.2;
  }

  // Angular velocity damping
  if (fabs(angVel) > 50)
  {
    Kd += 0.2;
  }

  // ==== Safety cutoff ====
  if (absAngle > startAngle)
  {
    integral = 0;
    lastError = 0;
    motorsBrake();
    return;
  }

  // ==== PID ====
  error = setpoint - angle;
  integral += error * dt;
  if (integral > iLimit)
    integral = iLimit;
  if (integral < -iLimit)
    integral = -iLimit;

  float derivative = (error - lastError) / dt;
  lastError = error;

  if (fabs(error) < tolerance)
  {
    error = 0;
    derivative = 0;
  }

  float u = Kp * error + Ki * integral + Kd * derivative;

  // Motor deadband and limits
  int effort = (int)round(u);
  if (effort > 0 && effort < motorMin)
    effort = motorMin;
  if (effort < 0 && effort > -motorMin)
    effort = -motorMin;
  if (effort > motorMax)
    effort = motorMax;
  if (effort < -motorMax)
    effort = -motorMax;

  motorDrive(effort, effort);

  Serial.println(angle);
}
