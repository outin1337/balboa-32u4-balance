#pragma once

#define FORWARD true
#define BACKWARDS false

class PID_d {
public:
    PID_d(double Kp, double Ki, double Kd, double sampleTime, double *pid_output);

    double step(double setpoint, double input);
    void SetLimit(double min, double max);
    void ReverseDirection();
    void reset();

private:
    double Kp, Ki, Kd;
    double prevError;
    double prevTime;
    double integral;
    double Min;
    double Max;
    double sampleTime;
    double *pid_output;
};
