#include "PID_lib.h"
#include "Arduino.h"

PID_d::PID_d(double Kp, double Ki, double Kd,  double sampleTime, double* pid_output)
    : Kp(Kp), Ki(Ki), Kd(Kd), Max(0.0), Min(0.0), prevError(0.0), integral(0.0), sampleTime(sampleTime), pid_output(pid_output)
{}

double PID_d::step(double setpoint, double input)
{
    double time = millis();
    double time_diff = (time - prevTime);
    double dt = time_diff/1000;
    
    if (time_diff >= sampleTime)
    {
      double err = setpoint - input;
      integral += Ki*err*dt;

      if (integral > Max)
      {
        integral = Max;
      }
      else if(integral < Min)
      {
        integral = Min;
      }

      double proportional = Kp * err;
      double derivative = Kd * ((err-prevError)/dt);

      prevError = err;
      prevTime = time;

      double output = proportional + derivative + integral; 
      if (output > Max) 
      {
        output = Max;
      }
      else if(output < Min)
      {
        output = Min;
      }

      *pid_output = output;
    }
    return 0;
}

void PID_d::SetLimit(double min, double max)
{
  Min = min;
  Max = max;
}

void PID_d::ReverseDirection()
{
  Kp *= -1;
  Ki *= -1;
  Kd *= -1;
}

void PID_d::reset()
{
    integral = 0.0;
    prevError = 0.0;
}
