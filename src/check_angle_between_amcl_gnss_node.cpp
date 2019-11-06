/*
    This package subscribes
        - nav_msgs/Odometry
        - sensor_msgs/NavSatFix
    and fuses, publishes
        - nav_msgs/Odometry
    message.
*/

#include <ros/ros.h>
#include "check_angle_between_amcl_gnss.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_angle_between_amcl_gnss_node");
    
    NavsatfixOdometryEKF noen;
    noen.run();
    
    return 0;
}
