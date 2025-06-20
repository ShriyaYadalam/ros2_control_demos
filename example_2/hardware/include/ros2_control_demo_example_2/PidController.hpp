#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController 
{
public:
    PIDController() : kp_(0), ki_(0), kd_(0), prev_error_(0.0), integral_(0.0) {}

    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double setpoint, double measured, double dt)
    {
        double error = setpoint - measured;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        double max_speed = 13.5;
        if(output>max_speed)
        {
          output = max_speed;
        }
        else if(output<-max_speed)
        {
          output = -max_speed;
        }
        return output; 
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
