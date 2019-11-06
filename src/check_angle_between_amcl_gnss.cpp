#include "check_angle_between_amcl_gnss.hpp"
#include "utility.hpp"

#include <iostream>
#include <typeinfo>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

NavsatfixOdometryEKF::NavsatfixOdometryEKF()
{
    ros::NodeHandle nhp{"~"};
    map_frame_id_ = getParamFromRosParam<std::string>(nhp, "map_frame_id");
    gps_frame_id_ = getParamFromRosParam<std::string>(nhp, "gps_frame_id");

    amcl_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_amcl"));
    if (not amcl_file_.is_open())
        ros_exception("amcl file cannot be opened");
    navsatfix_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_navsatfix"));
    if (not navsatfix_file_.is_open())
        ros_exception("navsatfix file could not be opened");
    gnss_lla_file_.open(getParamFromRosParam<std::string>(nhp, "dump_file_path_gnss_lla"));
    if (not gnss_lla_file_.is_open())
        ros_exception("gnss_lla file could not open");

    ros::NodeHandle nh;
    // Get initial LLA from rosparam
    double lat {getParamFromRosParam<double>(nh, "gnss_ini_lat")};
    double lon {getParamFromRosParam<double>(nh, "gnss_ini_lon")};
    double alt {getParamFromRosParam<double>(nh, "gnss_ini_alt")};

    initial_lla_ = LLA{lat, lon, alt};
    recent_lla_ = initial_lla_;
}

void NavsatfixOdometryEKF::run()
{
    ros::NodeHandle nhp {"~"};
    std::string amcl_topic {getParamFromRosParam<std::string>(nhp, "amcl_topic")};
    std::string navsatfix_topic {getParamFromRosParam<std::string>(nhp, "navsatfix_topic")};

    ros::NodeHandle nh;
    ros::Subscriber sub_amcl {nh.subscribe(amcl_topic, 1, &NavsatfixOdometryEKF::subCallbackAMCL, this)};
    ros::Subscriber sub_navsatfix {nh.subscribe(navsatfix_topic, 1, &NavsatfixOdometryEKF::subCallbackNavsatfix, this)};

    tf2_ros::TransformListener tf_listener {tf_buffer_}; // listening starts

    ros::Rate rate {10};
    while (ros::ok())
    {
        dumpMeasurements();

        ros::spinOnce();
        rate.sleep();
    }
}

void NavsatfixOdometryEKF::subCallbackAMCL(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl)
{
    recent_amcl_ = *amcl;
}

void NavsatfixOdometryEKF::subCallbackNavsatfix(const sensor_msgs::NavSatFix::ConstPtr &navsatfix)
{
    recent_lla_ = LLA {*navsatfix};

    recent_gnss_xy_ = transformLLAtoOdomFrame(recent_lla_);
}

geometry_msgs::Point NavsatfixOdometryEKF::transformLLAtoOdomFrame(const LLA &lla) const
{
    // lla -> enu
    geometry_msgs::Point pos {CalcRelativePosition(initial_lla_, recent_lla_)};

    // enu -> odom
    while (true)
    {
        try
        {
            geometry_msgs::TransformStamped transformStamped {tf_buffer_.lookupTransform(map_frame_id_, gps_frame_id_, ros::Time(0))};
            geometry_msgs::Point ret;
            tf2::doTransform(pos, ret, transformStamped);
            return ret;
        }
        catch (const tf2::TransformException &e)
        {
            std::cout << e.what() << std::endl;
            continue;
        }
    }
}

void NavsatfixOdometryEKF::dumpMeasurements()
{
    double x {recent_amcl_.pose.pose.position.x};
    double y {recent_amcl_.pose.pose.position.y};
    amcl_file_ << x << " " << y << std::endl;

    navsatfix_file_ << recent_gnss_xy_.x << " " << recent_gnss_xy_.y << std::endl;

    gnss_lla_file_ << std::setprecision(20) << recent_lla_.latitude() << " " << recent_lla_.longitude() << " " << recent_lla_.altitude() << std::endl;

}