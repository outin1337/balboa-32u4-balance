#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>

#include "PID_v1.h"
#include "PID_lib.h"

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
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
double lastTime; 
double deltaTime;
double dt_d;
double pid_output;
double pid_output2;
const double setpoint = 11.0;
bool MotorOn = false;

const double R2D =  180 / 3.14159265;
PID_d pid(30.0, 600, 0.7, 10, &pid_output);
PID myPID(&angle, &pid_output2, &setpoint, 39.0, 200, 0.4, REVERSE);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s
  pid.SetLimit(-300, 300);
  
  myPID.SetOutputLimits(-300, 300);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  
  
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

  if ((millis() - lastTime) >= 10 ){
    /*
    angleAFiltered = (1-a) * angle + a * angleAFiltered;
    angleGFiltered =  alfa * (angleG - prevAngleG + prevAngleGFiltered);
    */
   
    read_sensor_calc();
    complementary_filter();

    pid.step(11.0, angle);
    //myPID.Compute();
    //Serial.println(pid_output);
    Serial.println(pid_output);
    //Serial.println(angle);

    if (buttonA.getSingleDebouncedPress() ){
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
      ledRed(1);
      ledYellow(1);
      balance();
    }
    else {
      stop_balance();
    }


    lastTime = millis();
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
  deltaTime = millis() - lastTime;
  gyroDps = ((gyro - gYZero) * 35)/ 1000.0;
  dt_d = deltaTime / 1000.0;
}

void complementary_filter()
{
  angle = 0.98 *(angle+gyroDps*dt_d) + 0.02 * angleAcc;
}
