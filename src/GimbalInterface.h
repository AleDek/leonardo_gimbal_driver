#ifndef _GIMBAL_INTERFACE
#define _GIMBAL_INTERFACE

// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <cstdlib>
#include <iostream>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()



class GimbalInterface {

    public :
        GimbalInterface(std::string port );
        ~GimbalInterface();

        //bool poll_serial(void);
       // bool get_thermal_data(float& chip_temp_fwd, float* temp_array_fwd, float& chip_temp_bwd, float* temp_array_bwd);
        bool write_us_setpoint(uint16_t roll_us,uint16_t pitch_us);
        bool is_serial_ok(void){return _is_serial_ok; };
        void reset_serial(void);
        
    private :
        std::string _port_name;
        speed_t _baud;
        int _serial_port;
        bool _is_serial_ok;
        uint8_t _rcv_buff[21];
        uint8_t _snd_buff[6];
        int _num_bytes;
        float _chip_temp_fwd;
        float _array_temp_fwd[8];
        float _chip_temp_bwd;
        float _array_temp_bwd[8];

        void configure_serial(void);
        void flush(void);
};


#endif