#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>

#include "PID_lib.h"

#define SETPOINT 11.0
#define KP 30.0
#define KI 150.0
#define KD 0.5
#define SAMPLE_TIME_MS 10
#define R2D 180 / 3.14159265
#define FILTER_COEFF 0.98

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4ButtonA buttonA;
Balboa32U4Buzzer buzzer;

const char ready[] PROGMEM = "!O4 L8 C D E";
const char startMotor[] PROGMEM = "!O5 L16 C"; 
const char stopMotor[] PROGMEM = "!O3 L4 G";

double angleAcc;
double angle = 0.0;

double gYZero;
double gyro;
double gyroDps;
double unfGyro;
double currentTime;
double lastTime; 
double deltaTime;
double dt_d;
double pid_output;
bool MotorOn = false;

PID_d pid(KP, KI, KD, SAMPLE_TIME_MS, &pid_output);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s
  imu.writeReg(LSM6::CTRL1_XL, 0b01010000); // 208 Hz, +/-2G
  pid.SetLimit(-300, 300);
  pid.ReverseDirection();
  
  Serial.println("Stabilzing... Please do not touch!");
  delay(1000);

  Serial.println("Calibration.. also please do not touch");
  float total = 0;
  for (int i = 0; i < 100; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }

  gYZero = total / 100;
  lastTime = millis();
  buzzer.playFromProgramSpace(ready);
}


void loop() {

  if ((millis() - lastTime) >= SAMPLE_TIME_MS){
    
    currentTime = millis();
    deltaTime = currentTime - lastTime;
    dt_d = deltaTime / 1000.0;
    lastTime = currentTime;

    read_sensor_calc();

    complementary_filter();

    /*Serial.print(angleAcc);
    Serial.print(",");
    Serial.print(unfGyro);
    Serial.print(",");
    Serial.println(angle); Uncomment to plot in serial plotter.*/  

    pid.step(SETPOINT, angle);

    if (buttonA.getSingleDebouncedPress()){
      MotorOn = !MotorOn;
      if (MotorOn){
        buzzer.playFromProgramSpace(startMotor);
        ledRed(0);
        ledYellow(1);
      }
      else {
        buzzer.playFromProgramSpace(stopMotor);
        ledRed(1);
        ledYellow(0);
      }
    }

    if (MotorOn && abs(angle) <= 45){
      ledGreen(1);
      ledYellow(0);
      balance();
    }
    else if(MotorOn && abs(angle) > 45){
      ledGreen(0);
      ledYellow(1);
      stop_balance();
    }
    else {
      stop_balance();
    }
  }
}

void balance()
{
  motors.setSpeeds(pid_output, pid_output);
} 

void stop_balance()
{
  motors.setSpeeds(0,0);
}

void read_sensor_calc()
{
  imu.read();
  angleAcc = (atan2(imu.a.z, imu.a.x) * R2D);
  gyro = imu.g.y;
  gyroDps = ((gyro - gYZero) * 35)/ 1000.0;
  unfGyro += gyroDps * dt_d;
}

void complementary_filter()
{
  angle = FILTER_COEFF *(angle+gyroDps*dt_d) + (1-FILTER_COEFF) * angleAcc;
}
