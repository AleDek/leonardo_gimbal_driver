#include "GimbalInterface.h"

#define THERMAL_BAUD B115200
#define DEBUG_PRINT 1


GimbalInterface::GimbalInterface(std::string port){
    _port_name = port;
    _baud = THERMAL_BAUD;
    _is_serial_ok = true;
    configure_serial();
}

GimbalInterface::~GimbalInterface(){
    close(_serial_port); 
}

void GimbalInterface::reset_serial(void){
    close(_serial_port);
    _is_serial_ok = false;
    configure_serial();
}

void GimbalInterface::flush(void){
    //usleep(2000000);
    memset(&_rcv_buff, 0, 21);
    tcflush(_serial_port,TCIOFLUSH);
}

void GimbalInterface::configure_serial(void){
    _serial_port = open(_port_name.c_str(), O_RDWR);

	// Check for errors
    if (_serial_port < 0) {
        #if DEBUG_PRINT
            printf("Error %i from open: %s\n", errno, strerror(errno));
        #endif
        _is_serial_ok = false;
    }

    struct termios tty;
    if(tcgetattr(_serial_port, &tty) != 0) {
        #if DEBUG_PRINT
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        #endif
        _is_serial_ok = false;
    }
    /* refer to: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/*/
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;      //n of byte that block read()
    cfsetispeed(&tty, _baud);
    cfsetospeed(&tty, _baud);

    // Save tty settings, also checking for error
    if (tcsetattr(_serial_port, TCSANOW, &tty) != 0) {
        #if DEBUG_PRINT
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        #endif
        _is_serial_ok = false;
    }
    _is_serial_ok = true;
}


// bool GimbalInterface::poll_serial(void){
//     memset(&_rcv_buff, 0, 21);
//     _num_bytes = read(_serial_port, &_rcv_buff[0], 2);
//     /*check for recive errors*/
//     if (_num_bytes < 0) {
//         #if DEBUG_PRINT
//         printf("Error reading: %s", strerror(errno));
//         #endif
//         flush();
//         _is_serial_ok = false;
//         return _is_serial_ok;
//     }
//     if(_rcv_buff[0] == 0xFF && _rcv_buff[1] == 0xFF){     
//         for(int i = 2;i<21;i++){
//             if(read(_serial_port, &_rcv_buff[i], 1)<1){
//                 flush();
//                 _is_serial_ok = false;
//                 return false;
//             }
//         }
//         _chip_temp_fwd = (float)((_rcv_buff[3] << 8U) | _rcv_buff[2]) / 10.0f;
//         for (uint32_t i = 4, n = 0; n < 8; i += 2, n++) {
//             _array_temp_fwd[n] = (float)((int16_t)((_rcv_buff[i+1] << 8U) | _rcv_buff[i]))/10.0f  ;
//         }

//         #if DEBUG_PRINT
//             //printf("temp frame received\n");
//             printf("ptat: %.1f temp: [",_chip_temp_fwd);
//             for(int i=0;i<8;i++){
//                 printf("%.1f, ",_array_temp_fwd[i]);
//             }
//             printf("]\n");
//         #endif
//         _is_serial_ok = true;
//         return true;
//     }
//     else{
//         flush();
//         _is_serial_ok = false;
//         return false;
//     }
//     return _is_serial_ok; //
// }

// bool GimbalInterface::get_thermal_data(float& chip_temp_fwd, float* temp_array_fwd, float& chip_temp_bwd, float* temp_array_bwd){
//     chip_temp_fwd = _chip_temp_fwd;
//     chip_temp_bwd = _chip_temp_bwd;
//     for(int i = 0;i<8;i++){
//         temp_array_fwd[i]  = _array_temp_fwd[i];
//         temp_array_bwd[i]  = _array_temp_bwd[i];
//     }
//     return _is_serial_ok;
// }

bool GimbalInterface::write_us_setpoint(uint16_t roll_us,uint16_t pitch_us){

    memset(&_snd_buff, 0, 6);
    _snd_buff[0] = '$';
    _snd_buff[1] = roll_us >> 8;
    _snd_buff[2] = roll_us;
    _snd_buff[3] = pitch_us >> 8;
    _snd_buff[4] = pitch_us;
    _snd_buff[5] = '%';

    //int ret = write(_serial_port, &_snd_buff[0], 6);
    int ret = write(_serial_port, _snd_buff, 6);
    if(ret != 6){
        #if DEBUG_PRINT
        printf("Error writing: %s", strerror(errno));
        #endif
        return false;
    }
    
    return true;
}