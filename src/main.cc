#include "chassis.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "chassis_wjz");
    wjz_chassis::Chassis_cpp Chassis_main;
    Chassis_main.run();
    ros::spin();
}
