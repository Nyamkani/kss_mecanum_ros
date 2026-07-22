#include "kssekf/kssekf_node.hpp"


const char node_name[] = "KssEKF_node";


KssEKF::KssEKF() : Node(node_name)
{
    this->declare_parameter("odom_pub_time", 30);
    this->odom_pub_time_ = this->get_parameter("odom_pub_time").as_int();


    // this->imu_msg_subs_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "/imu",
    //     rclcpp::QoS(rclcpp::KeepLast(10)),
    //     std::bind(&KssEKF::SubImuMsg, this, std::placeholders::_1)
    // );
    // this->odom_vel_subs_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/odom",
    //     rclcpp::QoS(rclcpp::KeepLast(10)),
    //     std::bind(&KssEKF::SubOdomTwistMsg, this, std::placeholders::_1)
    // );

    this->odom_tf_bc_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->odom_bc_cast_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->odom_bc_cast_time_),
        std::bind(&KssEKF::BroadCastOdomFilterMsg, this)
    );

    this->odom_filtered_pubs_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_filtered_test", rclcpp::QoS(rclcpp::KeepLast(10)));

    this->odom_filtered_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->odom_pub_time_),
        std::bind(&KssEKF::PubOdomFilteredworker, this)
    );

    this->imu_sub_.subscribe(this, "/imu");
    this->odom_sub_.subscribe(this, "/odom");


    this->sync_ = new Sync(MySyncPolicy(10), this->imu_sub_, this->odom_sub_);
    this->sync_->registerCallback(std::bind(&KssEKF::FusionCallback, this, std::placeholders::_1, std::placeholders::_2));

}

KssEKF::~KssEKF()
{
    // Destructor implementation if needed
}

int KssEKF::Initialize()
{
    // Initialize EKF instance
    if(this->ekf_ != nullptr)
    {
        delete this->ekf_;
        this->ekf_ = nullptr;
    }
    this->ekf_ = new EKF();
    
    if(this->ekf_ == nullptr)
    {
        #if EKF_DEBUG_MODE
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize EKF instance.");
        #endif

        return -1;
    }

    // #if EKF_DEBUG_MODE
    RCLCPP_INFO(this->get_logger(), "EKF instance initialized successfully.");
    // #endif

    return 0;
}

// void KssEKF::SubOdomTwistMsg(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//     if (!msg) return;

//     rclcpp::Time curr_time = msg->header.stamp;

//     if (!prevent_time_zero) {
//         this->prev_time_ = curr_time;
//         this->prevent_time_zero = true;
//         return;
//     }

//     double dt = (curr_time - this->prev_time_).seconds();
//     if (dt <= 0 || dt > 1.0) {
//         RCLCPP_WARN(this->get_logger(), "Invalid dt (%.3f) in odometry; skipping update.", dt);
//         return;
//     }

//     // EKF predict: full 3D velocity
//     this->ekf_->predict(
//         Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z),
//         msg->twist.twist.angular.z,  // TODO: 변경 가능 (full omega vector)
//         dt
//     );

//     this->prev_time_ = curr_time;

//     return;
// }

// void KssEKF::SubImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//     // Process the received IMU message
//     if (msg)
//     {
//     #if EKF_DEBUG_MODE
//         RCLCPP_INFO(this->get_logger(), "Received IMU message: orientation.x=%f, orientation.y=%f, orientation.z=%f, orientation.w=%f",
//                         msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
//     #endif
//         // You can store or process the IMU data as needed
//         this->ekf_->updateIMU(
//             Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
//             Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z)
//         );

//     }

//     return;
// }

void KssEKF::PubOdomFilteredworker()
{
    // Publish the filtered odometry data
    if (this->ekf_ != nullptr)
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        // Get the EKF state
        State state = this->ekf_->getState();

        // Fill in the odometry message
        odom_msg.pose.pose.position.x = state.position.x();
        odom_msg.pose.pose.position.y = state.position.y();
        odom_msg.pose.pose.position.z = state.position.z();
        
        odom_msg.pose.pose.orientation.x = state.orientation.x();
        odom_msg.pose.pose.orientation.y = state.orientation.y();
        odom_msg.pose.pose.orientation.z = state.orientation.z();
        odom_msg.pose.pose.orientation.w = state.orientation.w();

        odom_msg.twist.twist.linear.x = state.velocity.x();
        odom_msg.twist.twist.linear.y = state.velocity.y();
        odom_msg.twist.twist.linear.z = state.velocity.z();

        // Publish the message
        this->odom_filtered_pubs_->publish(odom_msg);
    }

    return;
}


void KssEKF::BroadCastOdomFilterMsg()
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    // Get the EKF state
    State state = this->ekf_->getState();

    t.transform.translation.x = state.position.x();
    t.transform.translation.y = state.position.y();
    t.transform.translation.z = state.position.z();

    Quaterniond q = state.orientation.normalized();  // 정규화 보장
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    this->odom_tf_bc_->sendTransform(t);

    return;
}

void KssEKF::FusionCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    // 시간 차 계산

    rclcpp::Time curr_time = imu_msg->header.stamp;

    if (last_time_.nanoseconds() != 0) 
    {
    //   double dt = (msg_time(imu_msg->header.stamp) - last_time_).seconds();


      double dt = (curr_time - last_time_).seconds();

        // Predict 단계: odometry velocity + angular z + dt
        Eigen::Vector3d odom_vel(odom_msg->twist.twist.linear.x,
                            odom_msg->twist.twist.linear.y,
                            odom_msg->twist.twist.linear.z);

        Eigen::Vector3d omega(odom_msg->twist.twist.angular.x,
                        odom_msg->twist.twist.angular.y,
                        odom_msg->twist.twist.angular.z);

    //   double omega_z = odom_msg->twist.twist.angular.z;

      ekf_->predict(odom_vel, omega, dt);

      // Update 단계: imu 가속도, 각속도
      Eigen::Vector3d accel(imu_msg->linear_acceleration.x,
                            imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z);

      Eigen::Vector3d gyro(imu_msg->angular_velocity.x,
                           imu_msg->angular_velocity.y,
                           imu_msg->angular_velocity.z);

      ekf_->updateIMU(accel, gyro);
    }

    this->last_time_ = curr_time;

    publishFilteredOdometry();
  }

  void KssEKF::publishFilteredOdometry() 
  {
    State state = ekf_->getState();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = state.position.x();
    odom_msg.pose.pose.position.y = state.position.y();
    odom_msg.pose.pose.position.z = state.position.z();

    odom_msg.pose.pose.orientation.x = state.orientation.x();
    odom_msg.pose.pose.orientation.y = state.orientation.y();
    odom_msg.pose.pose.orientation.z = state.orientation.z();
    odom_msg.pose.pose.orientation.w = state.orientation.w();

    odom_msg.twist.twist.linear.x = state.velocity.x();
    odom_msg.twist.twist.linear.y = state.velocity.y();
    odom_msg.twist.twist.linear.z = state.velocity.z();

    this->odom_filtered_pubs_->publish(odom_msg);
  }
