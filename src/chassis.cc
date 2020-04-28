#include "chassis.h"

namespace wjz_chassis{

    wheel_pose2d::wheel_pose2d(){
        x  = 0.0;
        y  = 0.0;
        th = 0.0;
    }

    wheel_pose2d::~wheel_pose2d(){}

    Chassis_cpp::Chassis_cpp(){
        Chassis_Reader_Running = true;
        seq = 0;
    }

    Chassis_cpp::~Chassis_cpp(){

    }

    void Chassis_cpp::Ros_Param_Reading(){
        private_nh_.param<std::string>("USB_COM", USB_COM_, "/dev/ttyUSB0");
        private_nh_.param<int>("Baud", baud_, 115200);
        private_nh_.param<double>("Orgin_x", orgin_pose.x, 0.0);
        private_nh_.param<double>("Orgin_y", orgin_pose.y, 0.0);
        private_nh_.param<double>("Orgin_th", orgin_pose.th, 0.0);
        private_nh_.param<double>("wheel_base", wheel_base, 2.0); // if Ackermann steering
        private_nh_.param<int>("mode", mode_, 1);
    }

    void Chassis_cpp::USB_Setting(){
        try{
            io_controller = new boost::asio::serial_port(io_service,USB_COM_);
            io_controller->set_option(boost::asio::serial_port::baud_rate(baud_));
            ROS_WARN("Open USB Ok!");
        }catch(...){
            ROS_ERROR("Open USB Error Once!");
        }
    }

    void Chassis_cpp::Control_info_callback(const WJZ_Chassis::Control_info::ConstPtr& ctl_msg){
        control_lock.lock();
        X1 mode = ctl_msg->mode;
        R8 buffer[5];
        //1-3 for diff robot
        if(mode == 1){
            buffer[0] = mode;
            buffer[1] = 0.0;
            buffer[2] = ctl_msg->v;
            buffer[3] = ctl_msg->w;
        }
        if(mode == 2){
            buffer[0] = ctl_msg->d_t; //usually 0.1
            buffer[1] = ctl_msg->d_x;
            buffer[2] = ctl_msg->d_y;
            buffer[3] = ctl_msg->d_z;
        }
        if(mode == 3){
            buffer[0] = mode;
            buffer[1] = 0.0;
            buffer[2] = ctl_msg->left_v;
            buffer[3] = ctl_msg->right_v;
        }
        //4 for auto car,Ackermann steering
        if(mode == 4){
            buffer[0] = mode;
            buffer[1] = 0.0;
            buffer[2] = ctl_msg->a;
            buffer[3] = ctl_msg->steer;
        }
        for(int i = 0;i<=3;i++){
            buffer[4] += floor(buffer[i]/1000);
        }
        io_controller->write_some(boost::asio::buffer(buffer));
        control_lock.unlock();
    }

    void Chassis_cpp::Chassis_Reader_Function(){
        now_pose.x  += orgin_pose.x;
        now_pose.y  += orgin_pose.y;
        now_pose.th += orgin_pose.th; //init
        while(Chassis_Reader_Running){
            if(io_controller == nullptr){
                ROS_WARN("Serial port get null");
            }
            bool receive_right = false;
            I1 tmp_buffer[50];
            //for auto car we use USBCAN , for diff robot we read from stm32, you shoule write code to stm32
            auto buffersize = io_controller->read_some(boost::asio::buffer(tmp_buffer));
            if(buffersize<34){
                ROS_WARN("no thing");
                break;
            }
            if(buffersize>=34){
                I1* data = tmp_buffer+0;
                U4 head = *(U2*)(data+0);
                if(head != 0xAF12BCDA){
                    break;
                }
                if(mode_ == DiffRobot){
                    R8 v       = *(R8*)(data+2);
                    R8 w       = *(R8*)(data+10);
                    R8 left_v  = *(R8*)(data+18);
                    R8 right_v = *(R8*)(data+26);
                    I4 check   = *(I4*)(data+34);
                    if( floor((v+w+left_v+right_v)/1000) == check){
                        receive_right = true;
                    }

                    if(!receive_right){
                        ROS_WARN("receive ERROR,correct it at slam node according to matching");
                    }
                    R8 dt = ros::Time::now().toSec() - pre_time;
                    now_pose.x  += v*dt*cos(now_pose.th);
                    now_pose.y  += v*dt*sin(now_pose.th);
                    now_pose.th += w*dt;
                    geometry_msgs::Quaternion th_q = tf::createQuaternionMsgFromYaw(now_pose.th);

                    geometry_msgs::TransformStamped odom_trans;
                    odom_trans.header.stamp = ros::Time::now();
                    odom_trans.header.frame_id = "odom";
                    odom_trans.child_frame_id = "base_link";
                    odom_trans.transform.translation.x = now_pose.x;
                    odom_trans.transform.translation.y = now_pose.y;
                    odom_trans.transform.translation.z = 0.0;
                    odom_trans.transform.rotation = th_q;
                    tf::TransformBroadcaster odom_broadcaster;
                    odom_broadcaster.sendTransform(odom_trans);

                    nav_msgs::Odometry odom_now;
                    odom_now.header.stamp = ros::Time::now();
                    odom_now.header.seq = seq++;
                    odom_now.header.frame_id = "odom";
                    odom_now.pose.pose.position.x  = now_pose.x;
                    odom_now.pose.pose.position.y  = now_pose.y;
                    odom_now.pose.pose.position.z  = 0.0;
                    odom_now.pose.pose.orientation = th_q;
                    odom_now.twist.twist.linear.x  = v*cos(now_pose.th);
                    odom_now.twist.twist.linear.y  = v*sin(now_pose.th);
                    odom_now.twist.twist.angular.z = w;

                    pre_time = ros::Time::now().toSec();
                } else if(mode_ == Ackermann){
                    R8 a_e     = *(R8*)(data+2);  //estimation a,given by USBCAN or ECU
                    R8 a_c     = *(R8*)(data+10); //chassis a
                    R8 steer   = *(R8*)(data+18);
                    R8 v       = *(R8*)(data+26);
                    I4 check   = *(I4*)(data+34);
                    if( floor((a_e+a_c+steer+v)/1000) == check){
                        receive_right = true;
                    }

                    if(!receive_right){
                        ROS_WARN("receive ERROR,correct it at slam node according to matching");
                    }
                    R8 dt = ros::Time::now().toSec() - pre_time;
                    now_pose.x  += v*dt*cos(now_pose.th);
                    now_pose.y  += v*dt*sin(now_pose.th);
                    now_pose.th += v/wheel_base*tan(steer)*dt;
                    geometry_msgs::Quaternion th_q = tf::createQuaternionMsgFromYaw(now_pose.th);

                    geometry_msgs::TransformStamped odom_trans;
                    odom_trans.header.stamp = ros::Time::now();
                    odom_trans.header.frame_id = "odom";
                    odom_trans.child_frame_id = "base_link";
                    odom_trans.transform.translation.x = now_pose.x;
                    odom_trans.transform.translation.y = now_pose.y;
                    odom_trans.transform.translation.z = 0.0;
                    odom_trans.transform.rotation = th_q;
                    tf::TransformBroadcaster odom_broadcaster;
                    odom_broadcaster.sendTransform(odom_trans);

                    nav_msgs::Odometry odom_now;
                    odom_now.header.stamp = ros::Time::now();
                    odom_now.header.seq = seq++;
                    odom_now.header.frame_id = "odom";
                    odom_now.pose.pose.position.x  = now_pose.x;
                    odom_now.pose.pose.position.y  = now_pose.y;
                    odom_now.pose.pose.position.z  = 0.0;
                    odom_now.pose.pose.orientation = th_q;
                    odom_now.twist.twist.linear.x  = v*cos(now_pose.th);
                    odom_now.twist.twist.linear.y  = v*sin(now_pose.th);

                    pre_time = ros::Time::now().toSec();
                }
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }

    void Chassis_cpp::run(){
        Ros_Param_Reading();
        USB_Setting();
        control_sub = node_.subscribe<WJZ_Chassis::Control_info>("/control_to_base", 10, &Chassis_cpp::Control_info_callback, this);
        Odom_pub    = node_.advertise<nav_msgs::Odometry>("/wheel_odom",10,true);
        Read_chassis_thread = new boost::thread(&Chassis_cpp::Chassis_Reader_Function,this);
    }
}