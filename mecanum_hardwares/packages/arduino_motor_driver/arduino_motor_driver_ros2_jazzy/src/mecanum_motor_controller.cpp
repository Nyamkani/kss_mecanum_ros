#include "ArduinoMotorDriver/mecanum_motor_controller.hpp"


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

    if(this->motor_Driver_->Initialize(this->serial_port, this->serial_baudrate) < 0)
        return -1; 

    ArduinoCommand cmd;

    cmd.command = 'r';
    cmd.args.clear();

    this->motor_Driver_->Write(cmd);

    cmd.command = 'e';
    cmd.args.clear();

    this->motor_Driver_->Read(cmd);

    this->front_left_motor_prev_enc_ = motor_Driver_->ReadEncoder(1);
    this->front_right_motor_prev_enc_ = motor_Driver_->ReadEncoder(4);    //actually arduino motor 4
    this->rear_left_motor_prev_enc_ = motor_Driver_->ReadEncoder(2);    //actually arduino motor 2
    this->rear_right_motor_prev_enc_ = motor_Driver_->ReadEncoder(3);
        

    this->prev_time_ = std::chrono::system_clock::now();

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
    // this->CalOdomByEncoder();

    this->CalOdomByRPM();

    return;
}


void MacanumMotorDriver::CalOdomByEncoder()
{
 //calculate odometry
    //must set hardware value - wheel radius
    // if(this->test_flag == false)
    // {
        ArduinoCommand cmd;

        cmd.command = 'e'; 
        cmd.args.clear();

        if(this->motor_Driver_->Read(cmd) <0)
            return ;

        int front_left_motor_enc = motor_Driver_->ReadEncoder(1);
        int front_right_motor_enc = motor_Driver_->ReadEncoder(4);    //actually arduino motor 4
        int rear_left_motor_enc = motor_Driver_->ReadEncoder(2);    //actually arduino motor 2
        int rear_right_motor_enc = motor_Driver_->ReadEncoder(3);
        
        // this->test_flag = true;
        // printf("%d, %d, %d, %d\r\n",front_left_motor_enc, front_right_motor_enc, rear_left_motor_enc,rear_right_motor_enc);
        // return;
    // }
    // else if(this->test_flag)
    // {
        // double rpm_data_to_theta = (M_PI/180.0f)*(360.0f)*(1.0f/60.0f); //rad *deg* rps //60rpm -> 1rev/s ->360deg/s->2*Pi/s

        double front_left_motor_delta_theta = (double)(front_left_motor_enc - this->front_left_motor_prev_enc_)/MOTOR_ENC_CNT;
        double front_right_motor_delta_theta = (double)(front_right_motor_enc - this->front_right_motor_prev_enc_)/MOTOR_ENC_CNT;
        double rear_left_motor_delta_theta = (double)(rear_left_motor_enc - this->rear_left_motor_prev_enc_)/MOTOR_ENC_CNT;
        double rear_right_motor_delta_theta = (double)(rear_right_motor_enc - this->rear_right_motor_prev_enc_)/MOTOR_ENC_CNT;


        this->front_left_motor_prev_enc_ = front_left_motor_enc;
        this->front_right_motor_prev_enc_ = front_right_motor_enc;
        this->rear_left_motor_prev_enc_ = rear_left_motor_enc;
        this->rear_right_motor_prev_enc_ = rear_right_motor_enc;

        this->front_left_motor_dtheta_ = front_left_motor_delta_theta;
        this->front_right_motor_dtheta_ = front_right_motor_delta_theta;
        this->rear_left_motor_dtheta_ = rear_left_motor_delta_theta;
        this->rear_right_motor_dtheta_ = rear_right_motor_delta_theta;

        //now this is rad
        front_left_motor_delta_theta *= (2*M_PI*this->is_motor_reversed_);
        front_right_motor_delta_theta *= (2*M_PI*this->is_motor_reversed_);
        rear_left_motor_delta_theta *= (2*M_PI*this->is_motor_reversed_);
        rear_right_motor_delta_theta *= (2*M_PI*this->is_motor_reversed_);

        double dtx = (this->wheel_radius_/(4.0f* 1000.0f))
                                    *(front_left_motor_delta_theta + front_right_motor_delta_theta 
                                    + rear_right_motor_delta_theta + rear_left_motor_delta_theta);  //makes mm to m
        double dty = (this->wheel_radius_/(4.0f* 1000.0f))
                                    *(-1*front_left_motor_delta_theta + front_right_motor_delta_theta 
                                    -1*rear_right_motor_delta_theta + rear_left_motor_delta_theta);  //makes mm to m
        
        double dtz = (this->wheel_radius_/((4.0f))*(1.0f/(0.5*(this->wheel_base_ + this->wheel_seperate_))))
                                    *(-1*front_left_motor_delta_theta + front_right_motor_delta_theta 
                                    +rear_right_motor_delta_theta -1*rear_left_motor_delta_theta);  //makes mm to m
        


        this->x_pos_ += dtx*cos(this->w_z_)-dty*sin(this->w_z_);
        this->y_pos_ += dtx*sin(this->w_z_)+dty*cos(this->w_z_);

        this->w_z_ += dtz;

        if( this->w_z_ >= M_PI)
            this->w_z_ -=2*M_PI;
        else if ( this->w_z_ <= -M_PI)
            this->w_z_ +=2*M_PI;

        auto now = std::chrono::system_clock::now();
        auto millisec = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->prev_time_);
        double dt = millisec.count();

        if(dt >= 10)
        {
            dtx /=dt;
            dty /=dt;
            dtz /=dt;

            if(isnan(dtx) != 0)
                dtx = 0.0f;

            if(isnan(dty) != 0)
                dty = 0.0f;

            if(isnan(dtz) != 0)
                dtz = 0.0f;

            this->vbx_ = dtx;                     
            this->vby_ = dty;
            this->wbz_ = dtz;

            if(test_cnt++>=100)
            {
                printf("xpos = %lf, ypos = %lf, angle =  %lf, dt = %lf\r\n", x_pos_,y_pos_,w_z_, dt);

                test_cnt = 0;
            }


            this->prev_time_ = now;

            this->PublishOdomMsg();
        }


        // vbx = (vbx*sin(wbz) + vby*(cos(wbz)-1))/wbz;
        // vby = (vby*sin(wbz) + vbx*(1-cos(wbz))) /wbz;

        // // double current_angle = this->wbz_*(180.0f/(M_PI));

        // double final_vbx = vbx*cos(this->w_z_)-vby*sin(this->w_z_);
        // double final_vby = vbx*sin(this->w_z_)+vby*cos(this->w_z_);
        // double final_wbz = wbz;

        
        // this->x_pos_ += final_vbx*dt.count();
        // this->y_pos_ += final_vby*dt.count();

        // this->w_z_ += final_wbz*dt.count();

        // printf("%lf,%lf, %lf\r\n", x_pos_,y_pos_,w_z_);

        // this->PublishOdomMsg();

        // this->BroadCastOdomMsg();


    //     this->test_flag = false;
    // }

    return;
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


    auto now = std::chrono::system_clock::now();
    auto millisec = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->prev_time_);
    double dt = millisec.count();

    if(dt >= 10)
    {
        vbx *=dt/1000;
        vby *=dt/1000;
        wbz *=dt/1000;

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
    // ArduinoCommand cmd;

    // cmd.command = 'z';
    // cmd.args.clear();

    // this->motor_Driver_->Read(cmd);

    // this->front_left_motor_rpm_ = motor_Driver_->ReadRPM(1);
    // this->front_right_motor_rpm_ = motor_Driver_->ReadRPM(4);    //actually arduino motor 4
    // this->rear_left_motor_rpm_ = motor_Driver_->ReadRPM(2);    //actually arduino motor 2
    // this->rear_right_motor_rpm_ = motor_Driver_->ReadRPM(3);

    sensor_msgs::msg::JointState js;

    js.header.stamp = this->get_clock()->now();
 
    js.name.resize(4);
    js.name .push_back("front_left_wheel_joint");
    js.name .push_back("rear_left_wheel_joint");
    js.name .push_back("front_right_wheel_joint");
    js.name .push_back("rear_right_wheel_joint");

    // double rpm_to_rad = 2*(double)M_PI/60; 

    // double motor_vel_1 = rpm_to_rad*((double)(this->front_left_motor_rpm_))*this->is_motor_reversed_;
    // double motor_vel_2 = rpm_to_rad*((double)(this->front_right_motor_rpm_))*this->is_motor_reversed_;
    // double motor_vel_3 = rpm_to_rad*((double)(this->rear_left_motor_rpm_))*this->is_motor_reversed_;
    // double motor_vel_4 = rpm_to_rad*((double)(this->rear_right_motor_rpm_))*this->is_motor_reversed_;

    double motor_vel_1 = this->front_left_motor_dtheta_*this->is_motor_reversed_;
    double motor_vel_2 = this->front_right_motor_dtheta_*this->is_motor_reversed_;
    double motor_vel_3 = this->rear_left_motor_dtheta_*this->is_motor_reversed_;
    double motor_vel_4 = this->rear_right_motor_dtheta_*this->is_motor_reversed_;


    js.position.resize(4);
    js.position .push_back(motor_vel_1);
    js.position .push_back(motor_vel_2);
    js.position .push_back(motor_vel_3);
    js.position .push_back(motor_vel_4);

    this->joint_state_pubs_->publish(js);

    return;
}

void MacanumMotorDriver::PublishOdomMsg()
{
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = this->x_pos_;
    odom.pose.pose.position.y = this->y_pos_;

    odom.pose.pose.orientation.z = this->w_z_;

    odom.twist.twist.linear.x = this->vbx_;
    odom.twist.twist.linear.y = this->vby_;
    odom.twist.twist.angular.z = this->wbz_;

    this->odom_pubs_->publish(odom);

    return;
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
void MacanumMotorDriver::SubscribeCmdVelMsg(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
    ArduinoCommand cmd;

    double vbx = cmd_vel->linear.x; //due to i want mm standards
    double vby = cmd_vel->linear.y; //due to i want mm standards
    // cmd_vel->linear.z

    // cmd_vel->angular.x
    // cmd_vel->angular.y
    double wbz = cmd_vel->angular.z; //this is rads.

    // //something calculate vel to each wheels
    double motor_speed_1 = wbz*0.0005*(-this->wheel_base_ - this->wheel_seperate_) + vbx - vby;

    //actually arduino motor 4 
    double motor_speed_2 = wbz*0.0005*(this->wheel_base_ + this->wheel_seperate_) + vbx + vby;

    double motor_speed_3 = wbz*0.0005*(this->wheel_base_ + this->wheel_seperate_) + vbx - vby;

    //actually arduino motor 2
    double motor_speed_4 = wbz*0.0005*(-this->wheel_base_ - this->wheel_seperate_) + vbx + vby;

    double wheel_length = 2*M_PI*this->wheel_radius_/1000.0f; 

    double pwm_data_from_vel = (144.0f)/wheel_length; 

    int m1s = motor_speed_1*pwm_data_from_vel*this->is_motor_reversed_;
    int m2s = motor_speed_2*pwm_data_from_vel*this->is_motor_reversed_;    //actually arduino motor 4 
    int m3s = motor_speed_3*pwm_data_from_vel*this->is_motor_reversed_;
    int m4s = motor_speed_4*pwm_data_from_vel*this->is_motor_reversed_;    //actually arduino motor 2

    // printf("send rpm %d,%d,%d,%d\r\n", m1s,m2s,m3s,m4s);

    cmd.command = 'm';
    cmd.args.push_back(m1s);//motor 1   
    cmd.args.push_back(m4s);//motor 2   
    cmd.args.push_back(m3s);//motor 3   
    cmd.args.push_back(m2s);//motor 4   

    if(this->motor_Driver_->Write(cmd)<0)
        printf("motor driver data send failed \r\n");

    return;
}

