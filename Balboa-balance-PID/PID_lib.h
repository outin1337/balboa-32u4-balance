#pragma once

class PID_d {
public:
    PID_d(double Kp, double Ki, double Kd, double sampleTime, double *pid_output);

    double step(double setpoint, double input);
    void SetLimit(double min, double max);
    void reset();

private:
    double Kp, Ki, Kd;
    double prevError;
    double prevTime;
    double prevInput;
    double integral;
    double integral_term;
    double Min;
    double Max;
    double sampleTime;
    double *pid_output;
};
