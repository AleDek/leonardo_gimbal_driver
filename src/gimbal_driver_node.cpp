


//application
#include <iostream>
#include <chrono>
#include <thread>
#include "boost/thread.hpp"
#include "GimbalInterface.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

class GIMBAL_CONTROLLER{

    private:
        ros::NodeHandle _nh;
        GimbalInterface _gimbal;

        /*parameters*/
        std::string _arduino_port;
        int _roll_min_pwm;
        int _roll_max_pwm;
        int _pitch_min_pwm;
        int _pitch_max_pwm;
        double _roll_min_rad;
        double _roll_max_rad;
        double _pitch_min_rad;
        double _pitch_max_rad;

        /*state*/
        int _roll_sp_rad;
        int _pitch_sp_rad;
        

    public:


    GIMBAL_CONTROLLER();
    void load_param();
    void set_setpoint_rad(double & r, double & p);
    void get_state_rad(double & r,double & p);
    
};

void GIMBAL_CONTROLLER::GIMBAL_CONTROLLER(){
    load_param();

    _gimbal(_arduino_port);
    if(!_gimbal.is_serial_ok()){
        ROS_ERROR("serial port connection error");
        exit(0);
    }
    
}

void GIMBAL_CONTROLLER::load_param(){
    if( !_nh.getParam("/gimbal_driver/serial_port", _arduino_port) ){
        _arduino_port = "/dev/ttyACM0";
    }
    if( !_nh.getParam("/gimbal_driver/roll_min_pwm", _roll_min_pwm) ){
        _roll_min_pwm = 1000;
    }
    if( !_nh.getParam("/gimbal_driver/roll_max_pwm", _roll_max_pwm) ){
        _roll_max_pwm = 2000;
    }
    if( !_nh.getParam("/gimbal_driver/pitch_min_pwm", _pitch_min_pwm) ){
        _pitch_min_pwm = 1000;
    }
    if( !_nh.getParam("/gimbal_driver/pitch_max_pwm", _pitch_max_pwm) ){
        _pitch_max_pwm = 2000;
    }
    if( !_nh.getParam("/gimbal_driver/roll_min_rad", _roll_min_rad) ){
        _roll_min_rad = -1.57;
    }
    if( !_nh.getParam("/gimbal_driver/roll_max_rad", _roll_max_rad) ){
        _roll_max_rad = 1.57;
    }
    if( !_nh.getParam("/gimbal_driver/pitch_min_rad", _pitch_min_rad) ){
        _pitch_min_rad = -1.57;
    }
    if( !_nh.getParam("/gimbal_driver/pitch_max_rad", _pitch_max_rad) ){
        _pitch_max_rad = 1.57;
    }
}



int main(int argc, char **argv)
{
    

    std::string serial_dev  = "/dev/ttyACM0";


    GimbalInterface gimbal(serial_dev);

    printf("opening %s \n",serial_dev.c_str());

    uint16_t _roll_sp;
    uint16_t _pitch_sp;
    
    _roll_sp = 1500;
    _pitch_sp = 1500;

    char dummy = ' ';
    int pwm = 0;
    while(1){
        //gimbal.poll_serial();
        std::cout<<"gimbal console: \npress r to enter roll \npress p to enter pitch\npress x to exit\n";
        std::cin>>dummy;
        if(dummy =='r'){
            std::cout<<"enter roll setpoint [us]: ";
            std::cin>>pwm;
            _roll_sp = (uint16_t)pwm;
            gimbal.write_us_setpoint(_roll_sp, _pitch_sp);
        }
        if(dummy =='p'){
            std::cout<<"enter pitch setpoint [us]: ";
            std::cin>>pwm;
            _pitch_sp = (uint16_t)pwm;
            gimbal.write_us_setpoint(_roll_sp, _pitch_sp);
        }
        if(dummy =='x'){
            exit(0);
        }

        //printf("setpoints: [r p] [%u %u]\n", _roll_sp,_pitch_sp);
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
 
    }

    return 0;
}

