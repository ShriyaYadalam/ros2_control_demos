#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController 
{
public:
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double setpoint, double measured, double dt) 
    {
        double error = setpoint - measured;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() 
    {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double prev_error_;
    double integral_;
};

#endif // PID_CONTROLLER_HPP
