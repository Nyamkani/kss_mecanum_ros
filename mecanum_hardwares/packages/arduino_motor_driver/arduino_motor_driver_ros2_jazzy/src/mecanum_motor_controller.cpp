#include "ArduinoMotorDriver/mecanum_motor_controller.hpp"


static int64_t EncoderDelta(
    int32_t current,
    int32_t previous)
{
    int64_t delta =
        static_cast<int64_t>(current) -
        static_cast<int64_t>(previous);

    constexpr int64_t ENC_RANGE =
        (1LL << 32);

    if(delta > INT32_MAX)
    {
        delta -= ENC_RANGE;
    }
    else if(delta < INT32_MIN)
    {
        delta += ENC_RANGE;
    }

    return delta;
}


using std::placeholders::_1;


const char node_name[] = "MacanumMotorController_node";

MacanumMotorDriver::MacanumMotorDriver() : Node(node_name)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    //init subscriver
    this->vel_subs_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 
            10, 
            std::bind(&MacanumMotorDriver::SubscribeCmdVelMsg , this, _1)
            );

    this->odom_pubs_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", 
            qos_profile );

    this->twist_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(this->odom_pub_time_), std::bind(&MacanumMotorDriver::TwistPubWorker, this));

    this->odom_tf_bc_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //joint state
    this->joint_state_pubs_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 
            qos_profile );

    this->joint_state_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(this->joint_state_pub_time_), std::bind(&MacanumMotorDriver::JointStatePubWorker, this));

    this->declare_parameter("serial_port", "/dev/ttyUSB1");
    this->declare_parameter("serial_baudrate", 115200);

    this->serial_port = this->get_parameter("serial_port").as_string();
    this->serial_baudrate = this->get_parameter("serial_baudrate").as_int();
}

// MacanumMotorDriver::MacanumMotorDriver(int no_of_motor) : Node(node_name)
// {
//     auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

//     this->vel_subs_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "cmd_vel", 
//             10, 
//             std::bind(&MacanumMotorDriver::SubscribeCmdVelMsg , this, _1)
//             );

//     this->odom_pubs_ = this->create_publisher<nav_msgs::msg::Odometry>(
//             "odom", 
//             qos_profile );

//     this->twist_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(this->odom_pub_time_), std::bind(&MacanumMotorDriver::TwistPubWorker, this));

//     this->odom_tf_bc_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

//     //joint state
//     this->joint_state_pubs_ = this->create_publisher<sensor_msgs::msg::JointState>(
//             "joint_states", 
//             qos_profile );

//     this->joint_state_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(this->joint_state_pub_time_), std::bind(&MacanumMotorDriver::JointStatePubWorker, this));

// }

// MacanumMotorController::MacanumMotorController();


MacanumMotorDriver::~MacanumMotorDriver()
{
    if(this->motor_Driver_ !=  nullptr)
    {
        delete this->motor_Driver_;

        this->motor_Driver_ = nullptr;
    }
}

int MacanumMotorDriver::Initialize()
{   
    /*1. Motor Driver*/
    if(this->motor_Driver_ !=  nullptr)
    {
        delete this->motor_Driver_;

        this->motor_Driver_ = nullptr;
    }

    this->motor_Driver_ = new ArduinoMotorDriver(0,4);  //port 0, no of motors = 4

    ArduinoCommand cmd;

    if(this->motor_Driver_->Initialize(
        this->serial_port,
        this->serial_baudrate) < 0)
    {
        printf("Motor serial initialize failed\n");
        return -1;
    }

    printf("Motor serial initialize OK\n");

    // CH340/Arduino serial open 후 reset/boot 대기
    usleep(2000 * 1000);

    cmd.command = 'r';
    cmd.args.clear();

    if(this->motor_Driver_->Write(cmd) < 0)
    {
        printf("Motor reset ACK failed\n");
        return -1;
    }

    cmd.command = 'e';
    cmd.args.clear();

    if(this->motor_Driver_->Read(cmd) < 0)
    {
        printf("Motor initial encoder read failed\n");
        return -1;
    }

    this->front_left_motor_prev_enc_ = motor_Driver_->ReadEncoder(1);
    this->front_right_motor_prev_enc_ = motor_Driver_->ReadEncoder(4);    //actually arduino motor 4
    this->rear_left_motor_prev_enc_ = motor_Driver_->ReadEncoder(2);    //actually arduino motor 2
    this->rear_right_motor_prev_enc_ = motor_Driver_->ReadEncoder(3);
        

    this->prev_time_ = std::chrono::steady_clock::now();

    return 0;
}


int MacanumMotorDriver::Drive()
{
    if(!(this->command_queue_.empty()))
    {
        //write pwm speed mm/s-> rpm -> pwm
        //1. Get command and args from queue
        ArduinoCommand cmd = this->command_queue_.front();

        //2. do Write thing
        if(this->motor_Driver_->Write(cmd) < 0)
            return -1;

        //3. delete first command data
        this->command_queue_.erase(this->command_queue_.begin());

    }
    else
    {
        //read encoders
        ArduinoCommand cmd;
        std::string write_data_buf;

        //1. send data with read encoder or rpms
        int read_cmd = this->read_queue_;

        switch(read_cmd)
        { 
            case 0: //read encoder
            {
                cmd.command = 'e';

                break;
            }

            case 1: //read rpm
            {
                cmd.command = 'z';

                break;
            }

            default: break;
        }

        //2. do thing
        if(this->motor_Driver_->Read(cmd) < 0)
            return -1;


        if((this->read_queue_)++ >1)
            this->read_queue_ = 0;

    }

    return 0;
}


int test_cnt = 0;

//When tick is on, publish motor data
void MacanumMotorDriver::TwistPubWorker()
{
    this->CalOdomByEncoder();

    // this->CalOdomByRPM();

    return;
}

void MacanumMotorDriver::CalOdomByEncoder()
{
    // ---------------------------------------------------------
    // 1. Read encoder
    // ---------------------------------------------------------
    ArduinoCommand cmd;

    cmd.command = 'e';
    cmd.args.clear();

    if(this->motor_Driver_->Read(cmd) < 0)
    {
        return;
    }

    const int32_t front_left_motor_enc =
        this->motor_Driver_->ReadEncoder(1);

    const int32_t front_right_motor_enc =
        this->motor_Driver_->ReadEncoder(4);

    const int32_t rear_left_motor_enc =
        this->motor_Driver_->ReadEncoder(2);

    const int32_t rear_right_motor_enc =
        this->motor_Driver_->ReadEncoder(3);


    // ---------------------------------------------------------
    // 2. Calculate elapsed time
    // ---------------------------------------------------------
    const auto now =
        std::chrono::steady_clock::now();

    const double dt_sec =
        std::chrono::duration<double>(
            now - this->prev_time_).count();

    if(dt_sec <= 0.0)
    {
        return;
    }


    // ---------------------------------------------------------
    // 3. Calculate encoder delta
    //    EncoderDelta() handles int32_t wrap-around.
    // ---------------------------------------------------------
    const int64_t front_left_delta_enc =
        EncoderDelta(
            front_left_motor_enc,
            this->front_left_motor_prev_enc_);

    const int64_t front_right_delta_enc =
        EncoderDelta(
            front_right_motor_enc,
            this->front_right_motor_prev_enc_);

    const int64_t rear_left_delta_enc =
        EncoderDelta(
            rear_left_motor_enc,
            this->rear_left_motor_prev_enc_);

    const int64_t rear_right_delta_enc =
        EncoderDelta(
            rear_right_motor_enc,
            this->rear_right_motor_prev_enc_);


    // ---------------------------------------------------------
    // 4. Encoder sanity check
    //
    // maximum allowable encoder movement is calculated from:
    //
    // RPM -> rev/s -> encoder count / dt
    //
    // 200 RPM is intentionally larger than normal driving speed.
    // ---------------------------------------------------------
    constexpr double ODOM_MAX_WHEEL_RPM = 200.0;
    constexpr double ODOM_DELTA_MARGIN = 1.5;

    const double max_delta_enc =
        (ODOM_MAX_WHEEL_RPM / 60.0)
        * static_cast<double>(MOTOR_ENC_CNT)
        * dt_sec
        * ODOM_DELTA_MARGIN;

    const auto is_delta_valid =
        [max_delta_enc](int64_t delta)
        {
            return std::abs(
                static_cast<double>(delta))
                <= max_delta_enc;
        };

    constexpr uint8_t ODOM_RESYNC_INVALID_COUNT = 3;

    const bool encoder_delta_valid =
        is_delta_valid(front_left_delta_enc)  &&
        is_delta_valid(front_right_delta_enc) &&
        is_delta_valid(rear_left_delta_enc)   &&
        is_delta_valid(rear_right_delta_enc);

    if(!encoder_delta_valid)
    {
        this->encoder_invalid_count_++;

        if(this->encoder_invalid_count_ >=
        ODOM_RESYNC_INVALID_COUNT)
        {
            // Persistent invalid delta:
            // assume encoder reference has changed
            // (e.g. MCU reboot / encoder reset).
            //
            // Re-sync only the ROS-side baseline.
            this->front_left_motor_prev_enc_ =
                front_left_motor_enc;

            this->front_right_motor_prev_enc_ =
                front_right_motor_enc;

            this->rear_left_motor_prev_enc_ =
                rear_left_motor_enc;

            this->rear_right_motor_prev_enc_ =
                rear_right_motor_enc;

            this->prev_time_ = now;

            this->encoder_invalid_count_ = 0;
        }

        return;
    }

    this->encoder_invalid_count_ = 0;


    // ---------------------------------------------------------
    // 5. Valid frame -> update previous encoder
    // ---------------------------------------------------------
    this->front_left_motor_prev_enc_ =
        front_left_motor_enc;

    this->front_right_motor_prev_enc_ =
        front_right_motor_enc;

    this->rear_left_motor_prev_enc_ =
        rear_left_motor_enc;

    this->rear_right_motor_prev_enc_ =
        rear_right_motor_enc;


    // ---------------------------------------------------------
    // 6. Encoder count -> wheel angle delta [rad]
    // ---------------------------------------------------------
    constexpr double ENC_TO_RAD =
        (2.0 * M_PI)
        / static_cast<double>(MOTOR_ENC_CNT);

    const double front_left_motor_delta_theta =
        static_cast<double>(front_left_delta_enc)
        * ENC_TO_RAD
        * this->is_motor_reversed_;

    const double front_right_motor_delta_theta =
        static_cast<double>(front_right_delta_enc)
        * ENC_TO_RAD
        * this->is_motor_reversed_;

    const double rear_left_motor_delta_theta =
        static_cast<double>(rear_left_delta_enc)
        * ENC_TO_RAD
        * this->is_motor_reversed_;

    const double rear_right_motor_delta_theta =
        static_cast<double>(rear_right_delta_enc)
        * ENC_TO_RAD
        * this->is_motor_reversed_;


    // Keep wheel delta values for diagnostic / legacy use.
    // Unit is now consistently [rad].
    this->front_left_motor_dtheta_ =
        front_left_motor_delta_theta;

    this->front_right_motor_dtheta_ =
        front_right_motor_delta_theta;

    this->rear_left_motor_dtheta_ =
        rear_left_motor_delta_theta;

    this->rear_right_motor_dtheta_ =
        rear_right_motor_delta_theta;


    // ---------------------------------------------------------
    // 7. Mecanum forward kinematics
    //
    // wheel_radius_     : mm
    // wheel_base_       : mm
    // wheel_seperate_   : mm
    //
    // dtx, dty          : m
    // dtz               : rad
    // ---------------------------------------------------------
    const double wheel_radius_m =
        this->wheel_radius_ / 1000.0;

    const double rotation_radius =
        0.5
        * (this->wheel_base_
           + this->wheel_seperate_);

    const double dtx =
        (wheel_radius_m / 4.0)
        * (
            front_left_motor_delta_theta
            + front_right_motor_delta_theta
            + rear_left_motor_delta_theta
            + rear_right_motor_delta_theta
        );

    const double dty =
        (wheel_radius_m / 4.0)
        * (
            -front_left_motor_delta_theta
            + front_right_motor_delta_theta
            + rear_left_motor_delta_theta
            - rear_right_motor_delta_theta
        );

    const double dtz =
        (this->wheel_radius_
         / (4.0 * rotation_radius))
        * (
            -front_left_motor_delta_theta
            + front_right_motor_delta_theta
            - rear_left_motor_delta_theta
            + rear_right_motor_delta_theta
        );


    // ---------------------------------------------------------
    // 8. Body displacement -> odom/world frame
    // ---------------------------------------------------------
    const double yaw =
        this->w_z_;

    this->x_pos_ +=
        dtx * std::cos(yaw)
        - dty * std::sin(yaw);

    this->y_pos_ +=
        dtx * std::sin(yaw)
        + dty * std::cos(yaw);

    this->w_z_ += dtz;


    // ---------------------------------------------------------
    // 9. Normalize yaw [-pi, pi]
    // ---------------------------------------------------------
    if(this->w_z_ >= M_PI)
    {
        this->w_z_ -= 2.0 * M_PI;
    }
    else if(this->w_z_ <= -M_PI)
    {
        this->w_z_ += 2.0 * M_PI;
    }


    // ---------------------------------------------------------
    // 10. Body velocity
    // ---------------------------------------------------------
    const double vbx =
        dtx / dt_sec;

    const double vby =
        dty / dt_sec;

    const double wbz =
        dtz / dt_sec;

    this->vbx_ =
        std::isnan(vbx)
        ? 0.0
        : vbx;

    this->vby_ =
        std::isnan(vby)
        ? 0.0
        : vby;

    this->wbz_ =
        std::isnan(wbz)
        ? 0.0
        : wbz;


    // ---------------------------------------------------------
    // 11. Commit sample time and publish
    // ---------------------------------------------------------
    this->prev_time_ = now;

    this->PublishOdomMsg();
}

void MacanumMotorDriver::CalOdomByRPM()
{
    //calculate odometry
    //must set hardware value - wheel radius
    ArduinoCommand cmd;

    cmd.command = 'z';
    cmd.args.clear();

    this->motor_Driver_->Read(cmd);

    this->front_left_motor_rpm_ = motor_Driver_->ReadRPM(1);
    this->front_right_motor_rpm_ = motor_Driver_->ReadRPM(4);    //actually arduino motor 4
    this->rear_left_motor_rpm_ = motor_Driver_->ReadRPM(2);    //actually arduino motor 2
    this->rear_right_motor_rpm_ = motor_Driver_->ReadRPM(3);
    
    // printf("%lf, %lf, %lf, %lf\r\n",front_left_motor_rpm_, front_left_motor_rpm_, rear_left_motor_rpm_,rear_right_motor_rpm_);

    double rpm_data_to_theta = (M_PI/180.0f)*(360.0f)*(1.0f/60.0f); //rad *deg* rps //60rpm -> 1rev/s ->360deg/s->2*Pi/s

    double front_left_motor_delta_theta = rpm_data_to_theta*this->front_left_motor_rpm_*this->is_motor_reversed_;
    double front_right_motor_delta_theta = rpm_data_to_theta*this->front_right_motor_rpm_*this->is_motor_reversed_;
    double rear_left_motor_delta_theta = rpm_data_to_theta*this->rear_left_motor_rpm_*this->is_motor_reversed_;
    double rear_right_motor_delta_theta = rpm_data_to_theta*this->rear_right_motor_rpm_*this->is_motor_reversed_;

    double dtx = (this->wheel_radius_/(4.0f* 1000.0f))
                                    *(front_left_motor_delta_theta + front_right_motor_delta_theta 
                                    + rear_right_motor_delta_theta + rear_left_motor_delta_theta);  //makes mm to m
    double dty = (this->wheel_radius_/(4.0f* 1000.0f))
                                *(-1*front_left_motor_delta_theta + front_right_motor_delta_theta 
                                -1*rear_right_motor_delta_theta + rear_left_motor_delta_theta);  //makes mm to m

    double dtz = (this->wheel_radius_/((4.0f))*(1.0f/(0.5*(this->wheel_base_ + this->wheel_seperate_))))
                                *(-1*front_left_motor_delta_theta + front_right_motor_delta_theta 
                                +rear_right_motor_delta_theta -1*rear_left_motor_delta_theta);  //makes mm to m




    double vbx = dtx*cos(this->w_z_)-dty*sin(this->w_z_);
    double vby = dtx*sin(this->w_z_)+dty*cos(this->w_z_);
    double wbz = dtz;

    this->vbx_ = vbx;
    this->vby_ = vby;
    this->wbz_ = wbz;


    auto now = std::chrono::steady_clock::now();

    const double dt_sec =
        std::chrono::duration<double>(
            now - this->prev_time_).count();

    if(dt_sec >= 10)
    {
        vbx *=dt_sec/1000;
        vby *=dt_sec/1000;
        wbz *=dt_sec/1000;

        if(isnan(vbx) != 0)
            vbx = 0.0f;

        if(isnan(vby) != 0)
            vby = 0.0f;

        if(isnan(wbz) != 0)
            wbz = 0.0f;

        this->x_pos_ += vbx;                     
        this->y_pos_ += vby;
        this->w_z_ += wbz;

        if( this->w_z_ >= M_PI)
            this->w_z_ -=2*M_PI;
        else if ( this->w_z_ <= -M_PI)
            this->w_z_ +=2*M_PI;


        // if(test_cnt++>=100)
        // {
        //     printf("xpos = %lf, ypos = %lf, angle =  %lf, dt = %lf\r\n", x_pos_,y_pos_,w_z_, dt);

        //     test_cnt = 0;
        // }

        this->prev_time_ = now;

        this->PublishOdomMsg();
    }


    return;
}

void MacanumMotorDriver::JointStatePubWorker()
{
    sensor_msgs::msg::JointState js;

    js.header.stamp = this->get_clock()->now();

    js.name.push_back("front_left_wheel_joint");
    js.name.push_back("rear_left_wheel_joint");
    js.name.push_back("front_right_wheel_joint");
    js.name.push_back("rear_right_wheel_joint");

    constexpr double ENC_TO_RAD =
        (2.0 * M_PI) / MOTOR_ENC_CNT;

    const double front_left_pos =
        static_cast<double>(this->front_left_motor_prev_enc_) *
        ENC_TO_RAD *
        this->is_motor_reversed_;

    const double rear_left_pos =
        static_cast<double>(this->rear_left_motor_prev_enc_) *
        ENC_TO_RAD *
        this->is_motor_reversed_;

    const double front_right_pos =
        static_cast<double>(this->front_right_motor_prev_enc_) *
        ENC_TO_RAD *
        this->is_motor_reversed_;

    const double rear_right_pos =
        static_cast<double>(this->rear_right_motor_prev_enc_) *
        ENC_TO_RAD *
        this->is_motor_reversed_;

    js.position.push_back(front_left_pos);
    js.position.push_back(rear_left_pos);
    js.position.push_back(front_right_pos);
    js.position.push_back(rear_right_pos);

    this->joint_state_pubs_->publish(js);
}

void MacanumMotorDriver::PublishOdomMsg()
{
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    // Position
    odom.pose.pose.position.x = this->x_pos_;
    odom.pose.pose.position.y = this->y_pos_;
    odom.pose.pose.position.z = 0.0;

    // Yaw -> Quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, this->w_z_);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Body velocity
    odom.twist.twist.linear.x = this->vbx_;
    odom.twist.twist.linear.y = this->vby_;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = this->wbz_;

    this->odom_pubs_->publish(odom);
}

void MacanumMotorDriver::BroadCastOdomMsg()
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    t.transform.translation.x = this->x_pos_; 
    t.transform.translation.y = this->y_pos_; 
    t.transform.translation.z = this->z_pos_; 


    tf2::Quaternion q;

    q.setRPY(this->w_x_, this->w_y_, this->w_z_);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();

    t.transform.rotation.w = q.w();


    this->odom_tf_bc_->sendTransform(t);

    return;
}


//When recved msg, save the queue;
void MacanumMotorDriver::SubscribeCmdVelMsg(
    const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
    ArduinoCommand cmd;

    // ROS Twist
    // linear  : m/s
    // angular : rad/s
    const double vbx = cmd_vel->linear.x;
    const double vby = cmd_vel->linear.y;
    const double wbz = cmd_vel->angular.z;

    // wheel_base_, wheel_seperate_ are full dimensions [mm]
    // distance from robot center to wheel [m]
    const double rotation_radius =
        0.0005 * (this->wheel_base_ + this->wheel_seperate_);

    // Mecanum inverse kinematics
    // result: wheel linear velocity [m/s]
    const double motor_speed_1 =
        vbx - vby - wbz * rotation_radius;

    const double motor_speed_2 =
        vbx + vby + wbz * rotation_radius;

    const double motor_speed_3 =
        vbx - vby + wbz * rotation_radius;

    const double motor_speed_4 =
        vbx + vby - wbz * rotation_radius;

    // wheel circumference [m]
    const double wheel_length =
        2.0 * M_PI * this->wheel_radius_ / 1000.0;

    // wheel linear velocity [m/s]
    // -> rev/s
    // -> RPM
    const double rpm_from_vel =
        60.0 / wheel_length;

    const float m1_rpm =
        static_cast<float>(
            motor_speed_1 *
            rpm_from_vel *
            this->is_motor_reversed_);

    const float m2_rpm =
        static_cast<float>(
            motor_speed_2 *
            rpm_from_vel *
            this->is_motor_reversed_);

    const float m3_rpm =
        static_cast<float>(
            motor_speed_3 *
            rpm_from_vel *
            this->is_motor_reversed_);

    const float m4_rpm =
        static_cast<float>(
            motor_speed_4 *
            rpm_from_vel *
            this->is_motor_reversed_);

    cmd.command = 'm';

    // Arduino motor mapping
    cmd.args.push_back(m1_rpm); // Arduino motor 1
    cmd.args.push_back(m4_rpm); // Arduino motor 2
    cmd.args.push_back(m3_rpm); // Arduino motor 3
    cmd.args.push_back(m2_rpm); // Arduino motor 4

    if(this->motor_Driver_->Write(cmd) < 0)
    {
        printf("motor driver data send failed\r\n");
    }
}

