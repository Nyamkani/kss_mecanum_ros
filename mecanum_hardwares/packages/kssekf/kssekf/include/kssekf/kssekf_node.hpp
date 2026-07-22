#include <thread>
#include <mutex>
#include <memory>
#include <chrono>


//for using matrix lib.
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

//for using Node
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

//for using msgs
// #include "std_msgs/msg/string.hpp"
// #include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
//tf2
// #include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//msg filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


// #include <sensor_msgs/msg/joint_state.hpp>

// #inlcude "std_msgs`"

#include "kssekf.hpp"

#define EKF_DEBUG_MODE 0

using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, nav_msgs::msg::Odometry>;
using Sync = message_filters::Synchronizer<MySyncPolicy>;

class KssEKF : public rclcpp::Node
{
    public:
        int Initialize();
        KssEKF();
        virtual ~KssEKF();

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_vel_subs_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_msg_subs_;

        //For Sending motor status data
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_pubs_;

        //for broadcast odom
        std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_bc_;

        size_t odom_pub_time_ = 30;
        size_t odom_bc_cast_time_ = odom_pub_time_;

        rclcpp::TimerBase::SharedPtr odom_filtered_pub_timer_;
        rclcpp::TimerBase::SharedPtr odom_bc_cast_timer_;

        //for ekf
        EKF* ekf_ = nullptr;
        // std::unique_ptr<EKF> ekf_ = nullptr;

        bool prevent_time_zero = false;
        rclcpp::Time prev_time_;
        //for subs odom msgs

        void SubOdomTwistMsg(const nav_msgs::msg::Odometry::SharedPtr msg);

        //for subs imu msgs
        void SubImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg);
        
        //for publish filtered odom
        void PubOdomFilteredworker();

        void BroadCastOdomFilterMsg();

        void FusionCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

        void publishFilteredOdometry();

    public:

    private:
            // std::unique_ptr<Sync> sync_;
        Sync* sync_ = nullptr;
        // std::unique_ptr<Sync> sync_;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;

        rclcpp::Time last_time_;



};

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include "sensor_msgs/msg/imu.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "kssekf/kssekf.hpp"

// using sensor_msgs::msg::Imu;
// using nav_msgs::msg::Odometry;
// using namespace message_filters;

// class KssEKFNode : public rclcpp::Node {
// public:
//   KssEKFNode() : Node("kss_ekf_node") {
//     ekf_ = std::make_unique<EKF>();

//     imu_sub_.subscribe(this, "/imu");
//     odom_sub_.subscribe(this, "/odom");

//     sync_.reset(new Sync(MySyncPolicy(10), imu_sub_, odom_sub_));
//     sync_->registerCallback(std::bind(&KssEKFNode::sensorCallback, this, std::placeholders::_1, std::placeholders::_2));

//     odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_filtered", 10);
//   }

// private:
//   using MySyncPolicy = sync_policies::ApproximateTime<Imu, Odometry>;
//   std::unique_ptr<Sync> sync_;

//   message_filters::Subscriber<Imu> imu_sub_;
//   message_filters::Subscriber<Odometry> odom_sub_;

//   std::unique_ptr<EKF> ekf_;

//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

//   rclcpp::Time last_time_;

//   void sensorCallback(const Imu::ConstSharedPtr imu_msg, const Odometry::ConstSharedPtr odom_msg) {
//     // 시간 차 계산
//     if (last_time_.nanoseconds() != 0) {
//       double dt = (imu_msg->header.stamp - last_time_).seconds();

//       // Predict 단계: odometry velocity + angular z + dt
//       Eigen::Vector3d odom_vel(odom_msg->twist.twist.linear.x,
//                                odom_msg->twist.twist.linear.y,
//                                odom_msg->twist.twist.linear.z);
//       double omega_z = odom_msg->twist.twist.angular.z;

//       ekf_->predict(odom_vel, omega_z, dt);

//       // Update 단계: imu 가속도, 각속도
//       Eigen::Vector3d accel(imu_msg->linear_acceleration.x,
//                             imu_msg->linear_acceleration.y,
//                             imu_msg->linear_acceleration.z);

//       Eigen::Vector3d gyro(imu_msg->angular_velocity.x,
//                            imu_msg->angular_velocity.y,
//                            imu_msg->angular_velocity.z);

//       ekf_->updateIMU(accel, gyro);
//     }

//     last_time_ = imu_msg->header.stamp;

//     publishFilteredOdometry();
//   }

//   void publishFilteredOdometry() {
//     State state = ekf_->getState();

//     nav_msgs::msg::Odometry odom_msg;
//     odom_msg.header.stamp = this->get_clock()->now();
//     odom_msg.header.frame_id = "odom";
//     odom_msg.child_frame_id = "base_footprint";

//     odom_msg.pose.pose.position.x = state.position.x();
//     odom_msg.pose.pose.position.y = state.position.y();
//     odom_msg.pose.pose.position.z = state.position.z();

//     odom_msg.pose.pose.orientation.x = state.orientation.x();
//     odom_msg.pose.pose.orientation.y = state.orientation.y();
//     odom_msg.pose.pose.orientation.z = state.orientation.z();
//     odom_msg.pose.pose.orientation.w = state.orientation.w();

//     odom_msg.twist.twist.linear.x = state.velocity.x();
//     odom_msg.twist.twist.linear.y = state.velocity.y();
//     odom_msg.twist.twist.linear.z = state.velocity.z();

//     odom_pub_->publish(odom_msg);
//   }
// };