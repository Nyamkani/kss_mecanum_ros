#include "ArduinoMotorDriver/arduino_motor_driver.hpp"


#include <thread>
#include <mutex>
#include <memory>
#include <chrono>
#include <math.h>


//for using Node
#include "rclcpp/rclcpp.hpp"

//for using msgs
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

//tf2
#include <geometry_msgs/msg/transform_stamped.hpp>

// #include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


#include <sensor_msgs/msg/joint_state.hpp>

#define MOTOR_ENC_CNT 4320U

// #inlcude "std_msgs`"

class MacanumMotorDriver : public rclcpp::Node
{
    public:

    private:
        //for data publisher
        rclcpp::TimerBase::SharedPtr twist_pub_timer_;
    
        rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;
    

        //For getting Speed data
        // rclcpp::Subscription<>
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subs_;

        //For Sending motor status data
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pubs_;

        //for broadcast odom
        std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_bc_;

        //for publish joint_state
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pubs_;

        //time (ms)
        size_t odom_pub_time_ = 15;
        size_t joint_state_pub_time_ = 50;

        //main class
        ArduinoMotorDriver* motor_Driver_ = nullptr;

        std::string serial_port;
        int serial_baudrate;


        std::vector<ArduinoCommand> command_queue_;
        bool test_flag = false;
        int no_of_motors_= 0;
        std::vector<MotorData> motor_data_;
        int read_queue_ = 0;

        //thread mutex
        std::timed_mutex cmd_mtx_;


        //hardware
        double wheel_angle_ = 45.0f; //degree
        double wheel_seperate_ = 205.0f; //mm
        double wheel_base_= 190.0f; //mm
        double wheel_radius_ = 41.0f; //mm

        //encoder
        uint8_t encoder_invalid_count_ = 0;

        int32_t front_left_motor_prev_enc_ = 0;
        int32_t front_right_motor_prev_enc_ = 0;
        int32_t rear_left_motor_prev_enc_ = 0;
        int32_t rear_right_motor_prev_enc_ = 0;


        double front_left_motor_rpm_ = 0.0f;
        double front_right_motor_rpm_ = 0.0f;
        double rear_left_motor_rpm_ = 0.0f;
        double rear_right_motor_rpm_ = 0.0f;
        int is_motor_reversed_ = -1;

        double front_left_motor_dtheta_ = 0.0f;
        double front_right_motor_dtheta_ = 0.0f;
        double rear_left_motor_dtheta_ = 0.0f;
        double rear_right_motor_dtheta_ = 0.0f;

        //geometry position
        double x_pos_ = 0.0f;
        double y_pos_ = 0.0f;
        double z_pos_ = 0.0f;

        double w_x_ = 0.0f;
        double w_y_ = 0.0f;
        double w_z_ = 0.0f;

        //geomrtry velocity
        double vbx_ = 0.0f;
        double vby_ = 0.0f;
        double vbz_ = 0.0f;

        double wbx_ = 0.0f;
        double wby_ = 0.0f;
        double wbz_ = 0.0f;


        //time
        std::chrono::steady_clock::time_point prev_time_;
        
    public:

        MacanumMotorDriver() ;

        // MacanumMotorDriver(int no_of_motor) ;
        // MacanumMotorController();

        virtual ~MacanumMotorDriver();

        int Initialize();

        int Drive();

        int Write();

        int Read();

        void TwistPubWorker();

        void JointStatePubWorker();

        void RobotTransformwWorker();

    private:

        void CalOdomByEncoder();

        void CalOdomByRPM();

        void PublishOdomMsg();

        void BroadCastOdomMsg();

        void SubscribeCmdVelMsg(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);


};
