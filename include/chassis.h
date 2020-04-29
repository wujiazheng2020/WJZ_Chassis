/*
 * Copyright 2020 WJZ_Chassis, Jiazheng Wu
 *
 * Licensed under the MIT License
 */

#ifndef WJZ_CHASSIS_CHASSIS_H
#define WJZ_CHASSIS_CHASSIS_H

#include "ros/ros.h"
#include "ros/console.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <WJZ_Chassis/Control_info.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/asio.hpp>
#include <string>
#include "special_define.h"

#define  DiffRobot 1
#define  Ackermann 4

namespace wjz_chassis{
    class wheel_pose2d{
    public:
        wheel_pose2d();
        ~wheel_pose2d();
        R8 x;
        R8 y;
        R8 th;
        R8 v;
    };

    class Chassis_cpp{
    public:
        Chassis_cpp();
        ~Chassis_cpp();
        void run();
        void Ros_Param_Reading();
        void USB_Setting();
        //from stm32 or CAN
        void Chassis_Reader_Function();
        void Control_info_callback(const WJZ_Chassis::Control_info::ConstPtr& ctl_msg);
    private:
        ros::NodeHandle node_;
        ros::NodeHandle private_nh_;
        ros::Subscriber control_sub;
        ros::Publisher odom_pub;
        boost::thread *Read_chassis_thread;
        boost::mutex control_lock;
        bool Chassis_Reader_Running;

        I4 mode_;
        U4 seq;
        I4 baud_;
        R8 pre_time;
        R8 wheel_base;
        wheel_pose2d orgin_pose;
        wheel_pose2d now_pose;
        std::string USB_COM_;
        boost::asio::serial_port *io_controller;
        boost::asio::io_service io_service;
    };
}

#endif //WJZ_CHASSIS_CHASSIS_H