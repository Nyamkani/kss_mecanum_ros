#ifndef   __WT901C_HPP
#define   __WT901C_HPP


#ifdef __cplusplus
extern "C" {
#endif

#include "wt901c/REG.h"
#include "wt901c/serial.h"
#include "wt901c/wit_c_sdk.h"
#include <stdint.h>
#include <math.h>
#ifdef __cplusplus
}
#endif


#include <chrono>
#include <string>

//for using Node
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80


#define WT901C_DPRINT    0


//wt901c class

class Wt901c
{
    public:
        //data
        float fAcc[3] = {0};
        float fGyro[3] = {0};
        float fAngle[3] = {0};
        int fMag[3] = {0};
        unsigned char cBuff[1] = {0};

    private:
        int fd_= 0;
        const char* dir_ = "/dev/ttyUSB0";
        int s_iCurBaud_ = 115200;
        bool is_init_ = false;
        

    public:
        Wt901c();
        virtual ~Wt901c();


        int Initialize();
        int Initialize(std::string dir, int baud_rate);
        int Drive();        
 
    private:
        void AutoScanSensor(unsigned char* dev);
        static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
        void Delayms(uint16_t ucMs);


};




//ros2 node class

class ImuSensorNode : public rclcpp::Node
{
    public:
        Wt901c sensor_;

    private:
        //To publish imu val
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_msg_pubs_;

        //To publish mag val
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_msg_pubs_;

        float init_fAcc[3] = {0};
        float init_fGyro[3] = {0};
        float init_fAngle[3] = {0};
        int init_fMag[3] = {0};


    public:
        ImuSensorNode();
        virtual ~ImuSensorNode();

        int Initialize();

        int Drive();

        int PublishImuMsg();
        int PublishMagMsg();

        float GetAcc(int id);
        float GetGyro(int id);
        float GetAngle(int id);
        float GetMag(int id);

    private:
    	std::string serial_port_;
        int serial_baudrate_;
        bool x_axis_inverted_ = false;
        bool y_axis_inverted_ = false;
        bool z_axis_inverted_ = false;
};








#endif /* __WT901C_HPP */
