#ifndef WHEEL_HPP_
#define WHEEL_HPP_

#include <string>
#include <cmath> 
#include <lgpio.h>
#include <iostream> 

class Wheel
{
    public:
    std::string name = "";
    double cmd = 0;
    double vel = 0;
    double pos = 0;
    double rads_l_per_count = 0;
    double rads_r_per_count = 0;
    //rads_per_count = (2 * π) / counts_per_revolution
    //wheel_angle_radians = encoder_count * rads_per_count;

    volatile int en_count = 0;
    int prev_en_count = 0;
    int counts_l_per_rev = 0;
    int counts_r_per_rev = 0; 
    
    int dir_pin = 0;
    int pwm_pin = 0;

    int en_a = 0;
    int en_b = 0; 
 
    int h = 0;

    Wheel() = default;

    void setup(const std::string &wheel_name, int counts_l_per_rev, int counts_r_per_rev)
    {
        name = wheel_name;
        rads_l_per_count = (2*M_PI)/counts_l_per_rev;
        rads_r_per_count = (2*M_PI)/counts_r_per_rev;

        this->counts_l_per_rev = counts_l_per_rev;
        this->counts_r_per_rev = counts_r_per_rev;
    }
    void setMotorPins(int dir_p, int pwm_p)
    {
        
        dir_pin = dir_p;
        pwm_pin = pwm_p; 

        lgGpioClaimOutput(h,0,dir_pin,0); 
        lgGpioClaimOutput(h,0,pwm_pin,0);      

        lgTxPwm(h,pwm_pin,1000,0,0,0); 
    }
    void setEncPins(int enc_a,int enc_b)
    {
        en_a = enc_a;
        en_b = enc_b; 

        lgGpioClaimInput(h,LG_SET_PULL_UP,en_a);
        lgGpioClaimInput(h,LG_SET_PULL_UP,en_b);

        lgGpioClaimAlert(h, 0, LG_BOTH_EDGES, en_a, -1);
        lgGpioSetAlertsFunc(h,en_a,Wheel::en_callback, this);
        std::cout << "ENCODER PINS HAVE BEEN SETTTT" <<std::endl;
    }

   void setMotorSpeed(double speed_rad_per_sec )
    {//solely for duty-cycle conversion 
        if(speed_rad_per_sec>=0)
        {   
            lgGpioWrite(h,dir_pin,1);
        }
        else
        {
            lgGpioWrite(h,dir_pin,0);
        }

        double abs_speed = std::abs(speed_rad_per_sec);
        double max_wheel_speed = 13.5;
        double duty_cycle = std::min(abs_speed / max_wheel_speed, 1.0);
        double pwm_duty = duty_cycle*100; 

        if (lgTxPwm(h, pwm_pin, 1000, pwm_duty, 0, 0) < 0)
        {
            std::cerr << "Failed to set PWM for wheel: " << name << std::endl;
        }
    } 
    
    
    void setMotorPWM(double pwm )
    {
        if(pwm>=0)
        {     
            lgGpioWrite(h,dir_pin,1);
        }
        else 
        {
            lgGpioWrite(h,dir_pin,0);
        }

 
        double abs_pwm = std::abs(pwm);
        double max_pwm = 255;
        double duty_cycle = std::min(abs_pwm / max_pwm, 1.0);
        double duty = duty_cycle*100;  

        std::cout<<"wheel-"<<name<<"pwm-"<<pwm<<"dutycycle-"<<duty<<std::endl;


        if (lgTxPwm(h, pwm_pin, 1000, duty, 0, 0) < 0)
        {
            std::cerr << "Failed to set PWM for wheel: " << name << std::endl;
        }
    }
    

     static void en_callback(int num_alerts, lgGpioAlert_p alerts, void* userdata)
    {
        Wheel* self = static_cast<Wheel*>(userdata);
        if(!self) return;
        
        for(int i = 0; i < num_alerts; i++)
        {
            lgGpioAlert_t alert = alerts[i];
            
            if(alert.report.gpio == self->en_a && alert.report.level == 1)
            {
                int channel_a = alert.report.level;
                int channel_b = lgGpioRead(self->h, self->en_b);
                
                if(channel_a == channel_b)
                {
                    self->en_count++;
                }
                else 
                {
                    self->en_count--;
                }
            }
        }
    }

    double getLeftEncPos()
    {
        //int relative_count = en_count % counts_l_per_rev; 
        //return relative_count * rads_per_count;
        return en_count * rads_l_per_count;
    }
    double getRightEncPos()
    {
        //int relative_count = en_count % counts_r_per_rev; 
        //return relative_count * rads_per_count;
        return en_count * rads_r_per_count;
    }

    void stop() 
    {
        lgTxPwm(h,pwm_pin,0,0,0,0);
    }
};

#endif // WHEEL_HPP_