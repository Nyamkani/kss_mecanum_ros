
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef   __ARDUINO_MOTOR_DRIVER_HPP
#define   __ARDUINO_MOTOR_DRIVER_HPP

// C library headers
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <errno.h> // Error integer and strerror() function

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

#include <vector>
#include <string>


#ifdef __cplusplus
extern "C" {
#endif

#include <ArduinoMotorDriver/serial_uart.hpp>
#include <ArduinoMotorDriver/arduino_motor_command.h>

#ifdef __cplusplus
}
#endif


#define MAX_BUF 100

struct ArduinoCommand
{
    char command;
    std::vector<float> args;
};

struct MotorData
{
    int32_t encoder;
    float rpm;

    MotorData()
    {
        encoder = 0;
        rpm = 0.0f;
    }
};

enum DriverState
{
    sHWInit = 0,
    sInit = 1,
    sRun = 10,
    sError = 20
};



class ArduinoMotorDriver
{
    private:
        DriverState state_ = DriverState::sHWInit;  
        bool isHWInit = false;

        
        SerialComm Serial_;

        int target_vel_[4] = {0,0,0,0};
        bool isDoWrite_ = false;

        int read_queue_ = 0;

        std::vector<ArduinoCommand> command_queue_;

        int no_of_motors_= 0;
        std::vector<MotorData> motor_data_;

        double wheel_radius_ = 0;
        // double wheel_radius_ = 0;
        // double wheel_radius_ = 0;


    public:


    private:
        int SerialInitialize(const std::string& ttydir, int32_t baud_rate);
        int MainWorker();

    public:
        ArduinoMotorDriver();
        ArduinoMotorDriver(int no_of_motors);
        ArduinoMotorDriver(int port, int no_of_motors);
        virtual ~ArduinoMotorDriver();

        int Initialize(const std::string& ttydir, const int32_t baud_rate);

        // int Drive();
        int Write(const ArduinoCommand cmd);
        int Read(const ArduinoCommand cmd);


        void SetMacanummVelData(
            float vel1,
            float vel2,
            float vel3,
            float vel4);

        float ReadRPM(int motor);
        
        int ReadEncoder(int motor);
};




#endif