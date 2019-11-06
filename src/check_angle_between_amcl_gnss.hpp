#ifndef NAVSATFIX_ODOMETRY_EKF_HPP
#define NAVSATFIX_ODOMETRY_EKF_HPP

#include <utility>
#include <fstream>
#include <vector>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>

#include "lla.hpp"

class NavsatfixOdometryEKF {
public:
    NavsatfixOdometryEKF();

    void run();
private:
    tf2_ros::Buffer tf_buffer_;
    void subCallbackAMCL(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
    void subCallbackNavsatfix(const sensor_msgs::NavSatFix::ConstPtr&);

    void dumpMeasurements();

    std::string map_frame_id_ {"map"};
    std::string gps_frame_id_ {"gps"};

    std::ofstream amcl_file_, navsatfix_file_, gnss_lla_file_;
    LLA initial_lla_; // is used to calculate relative position from recent lla measurement
    LLA recent_lla_;
    geometry_msgs::Point recent_gnss_xy_;
    
    geometry_msgs::Point transformLLAtoOdomFrame(const LLA&) const;

    geometry_msgs::PoseWithCovarianceStamped recent_amcl_;
};

#endif